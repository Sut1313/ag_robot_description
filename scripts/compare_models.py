#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""compare_models.py —— 逐项核对"简化版 URDF"没有动到不该动的东西

对比对象:
    AG_robot.urdf.xacro          原版 (SolidWorks 导出的网格几何)
    AG_robot_simple.urdf.xacro   简化版 (box/cylinder 图元)

检查项与判定:
    A 关节     name/type/parent/child/origin/axis/limit/dynamics   完全一致 (不一致=失败)
    B 惯量     质量/质心/完整惯性张量                               完全一致 (不一致=失败)
    C 传感器+控制器  <gazebo> / <gazebo reference> / <ros2_control>  完全一致 (不一致=失败)
    D 每 link 几何包围盒(link 自身坐标系内)                          只报告偏差(单位 mm)
    E 整车包围盒(按关节树 FK 累到 base_footprint, 关节角全 0)        只报告偏差
    F 规模     网格三角形数 / 图元个数                               只报告

    A/B/C 是"不许变"的三样; D/E 是"尺寸相当"的量化证据, 只打印不判失败
    (简化本来就会带来几毫米~几厘米的差, 见每条注释)。

用法:
    source ~/ztools/ag_env.sh              # 需要 xacro 与 ag_robot_description
    python3 scripts/compare_models.py                      # 雷达 0° 姿态
    python3 scripts/compare_models.py --pitch 25            # 换姿态再比一次
    python3 scripts/compare_models.py --full /path/a.xacro --simple /path/b.xacro

退出码: 0 = A/B/C 全一致; 1 = 有不一致(详见打印的差异); 2 = 运行环境问题
"""
import argparse
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

TOL_J = 1e-9          # 关节/惯量比较容差(数值上"同一个数")
TOL_REPORT_MM = 0.05  # 报告里小于这个(mm)的差视为 0

# ─────────────────────────── URDF 展开与解析 ───────────────────────────


def expand(xacro_file, pitch, use_gazebo=True):
    """用 xacro 展开成 URDF 字符串并解析成 ElementTree"""
    cmd = ['xacro', str(xacro_file),
           'use_gazebo:=%s' % ('true' if use_gazebo else 'false'),
           'lidar_pitch:=%s' % pitch]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        sys.exit('xacro 失败 (%s):\n%s' % (' '.join(cmd), p.stderr.strip()))
    if p.stderr.strip():
        print('  [xacro 警告] %s' % p.stderr.strip().replace('\n', '\n  '))
    return ET.fromstring(p.stdout)


def v3(s, default=None):
    if s is None:
        return None if default is None else np.array(default, float)
    return np.array([float(x) for x in s.split()], float)


def rpy_to_mat(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp,     cp * sr,                cp * cr]])


def tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_mat(rpy)
    T[:3, 3] = xyz
    return T


# ─────────────────────────── 网格 / 图元 尺寸 ───────────────────────────

_mesh_cache = {}


def stl_stats(path):
    """(三角形数, lo, hi) —— 二进制或 ASCII STL, 单位与文件一致"""
    if str(path) in _mesh_cache:
        return _mesh_cache[str(path)]
    data = Path(path).read_bytes()
    n = struct.unpack('<I', data[80:84])[0]
    if len(data) == 84 + n * 50:                                  # 二进制: 用长度自校验
        rec = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
        v = rec[:, 12:48].copy().view('<f4').reshape(-1, 3).astype(np.float64)
        n_tri = n
    else:                                                         # ASCII
        vs = [ln.split()[1:4] for ln in data.decode('ascii', 'ignore').splitlines()
              if ln.strip().startswith('vertex')]
        v = np.array(vs, float)
        n_tri = len(v) // 3
    out = (n_tri, v.min(axis=0), v.max(axis=0))
    _mesh_cache[str(path)] = out
    return out


def dae_stats(path):
    """(三角形数, lo, hi) —— 官方 .dae (COLLADA):
       包围盒取所有 *-positions-array 里的点, 三角形数取 <triangles count="N"> 之和"""
    key = 'dae:' + str(path)
    if key in _mesh_cache:
        return _mesh_cache[key]
    import re
    txt = Path(path).read_text(errors='ignore')
    n_tri = sum(int(m) for m in re.findall(r'<triangles[^>]*count="(\d+)"', txt))
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for m in re.finditer(r'<float_array[^>]*id="[^"]*positions-array"[^>]*>(.*?)</float_array>',
                         txt, re.S):
        arr = np.array(m.group(1).split(), dtype=np.float64)
        if arr.size == 0 or arr.size % 3:
            continue
        pts = arr.reshape(-1, 3)
        lo = np.minimum(lo, pts.min(axis=0))
        hi = np.maximum(hi, pts.max(axis=0))
    out = (n_tri, lo, hi)
    _mesh_cache[key] = out
    return out


def mesh_stats(path):
    return dae_stats(path) if Path(path).suffix.lower() == '.dae' else stl_stats(path)


def resolve_mesh(uri, pkg_share):
    """package://ag_robot_description/meshes/x.stl -> 真实路径"""
    rel = uri.split('package://', 1)[-1]
    parts = rel.split('/', 1)
    return Path(pkg_share) / (parts[1] if len(parts) > 1 else parts[0])


def geom_bbox(el, pkg_share):
    """几何体在**自身坐标系**里的包围盒 (lo, hi)"""
    if el is None:
        return np.zeros(3), np.zeros(3)
    m = el.find('mesh')
    if m is not None:
        path = resolve_mesh(m.get('filename'), pkg_share)
        sc = v3(m.get('scale'), [1, 1, 1])
        n_tri, lo, hi = mesh_stats(path)
        return lo * sc, hi * sc
    b = el.find('box')
    if b is not None:
        s = v3(b.get('size'))
        return -s / 2, s / 2
    c = el.find('cylinder')
    if c is not None:
        r, l = float(c.get('radius')), float(c.get('length'))
        return np.array([-r, -r, -l / 2]), np.array([r, r, l / 2])
    s = el.find('sphere')
    if s is not None:
        r = float(s.get('radius'))
        return -np.array([r] * 3), np.array([r] * 3)
    return np.zeros(3), np.zeros(3)


def corners(lo, hi):
    return np.array([[x, y, z] for x in (lo[0], hi[0])
                     for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def link_geom_bbox(link_el, pkg_share):
    """link 的 visual+collision 几何在 link 坐标系里的并集包围盒"""
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    n_g = 0
    for tag in ('visual', 'collision'):
        for el in link_el.findall(tag):
            o = el.find('origin')
            xyz = v3(o.get('xyz'), [0, 0, 0]) if o is not None else np.zeros(3)
            rpy = v3(o.get('rpy'), [0, 0, 0]) if o is not None else np.zeros(3)
            glo, ghi = geom_bbox(el.find('geometry'), pkg_share)
            T = tf(xyz, rpy)
            pts = (T[:3, :3] @ corners(glo, ghi).T).T + T[:3, 3]
            lo = np.minimum(lo, pts.min(axis=0))
            hi = np.maximum(hi, pts.max(axis=0))
            n_g += 1
    return lo, hi, n_g


def link_mesh_tris(link_el, pkg_share):
    """link 里所有网格三角形数 (visual + collision 分开数, 因为渲染/物理各算一次)"""
    n_vis = n_col = 0
    for el in link_el.findall('visual'):
        m = el.find('geometry/mesh')
        if m is not None:
            n_vis += mesh_stats(resolve_mesh(m.get('filename'), pkg_share))[0]
    for el in link_el.findall('collision'):
        m = el.find('geometry/mesh')
        if m is not None:
            n_col += mesh_stats(resolve_mesh(m.get('filename'), pkg_share))[0]
    return n_vis, n_col


def link_n_prim(link_el):
    """(visual 图元数, collision 图元数)"""
    return (len(link_el.findall('visual')), len(link_el.findall('collision')))


# ─────────────────────────── 关节 / 惯量 / 传感器 对比 ───────────────────────────

J_ATTRS = [('origin', ['xyz', 'rpy']), ('axis', ['xyz']),
           ('limit', ['lower', 'upper', 'effort', 'velocity']),
           ('dynamics', ['damping', 'friction'])]


def num_of(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def cmp_joint(a, b, name, errs, notes):
    if a.get('type') != b.get('type'):
        errs.append('关节 %s: type %s != %s' % (name, a.get('type'), b.get('type')))
    for tag, attrs in (('parent', ['link']), ('child', ['link'])) + tuple(J_ATTRS):
        ea, eb = a.find(tag), b.find(tag)
        if (ea is None) != (eb is None):
            errs.append('关节 %s: <%s> 只在一边存在' % (name, tag))
            continue
        if ea is None:
            continue
        for at in attrs:
            va, vb = ea.get(at), eb.get(at)
            if va == vb:
                continue
            fa, fb = num_of(va), num_of(vb)
            if fa is not None and fb is not None and abs(fa - fb) <= TOL_J:
                notes.append('关节 %s %s@%s: 数值相同但写法不同 (%s vs %s)' % (name, tag, at, va, vb))
            else:
                errs.append('关节 %s %s@%s: %s != %s' % (name, tag, at, va, vb))


def cmp_inertial(a, b, name, errs):
    ia, ib = a.find('inertial'), b.find('inertial')
    if (ia is None) != (ib is None):
        errs.append('link %s: <inertial> 只在一边存在' % name)
        return
    if ia is None:
        return
    for tag, attrs in (('origin', ['xyz', 'rpy']), ('mass', ['value']), ('inertia', None)):
        ea, eb = ia.find(tag), ib.find(tag)
        if (ea is None) != (eb is None):
            errs.append('link %s inertial/%s 只在一边存在' % (name, tag))
            continue
        if ea is None:
            continue
        ats = attrs if attrs else sorted(set(ea.attrib) | set(eb.attrib))
        for at in ats:
            va, vb = ea.get(at), eb.get(at)
            fa, fb = num_of(va), num_of(vb)
            if fa is not None and fb is not None:
                if abs(fa - fb) > TOL_J:
                    errs.append('link %s inertial/%s@%s: %s != %s' % (name, tag, at, va, vb))
            elif va != vb:
                errs.append('link %s inertial/%s@%s: %s != %s' % (name, tag, at, va, vb))


def canon(el):
    """把元素序列化成稳定字符串(属性按名字排序), 用于比对传感器/控制器段"""
    import copy
    e = copy.deepcopy(el)
    for x in e.iter():
        x.attrib = {k: v for k, v in sorted(x.attrib.items())}
        if x.text and not x.text.strip():
            x.text = None
        if x.tail and not x.tail.strip():
            x.tail = None
    return ET.tostring(e, encoding='unicode')


def gz_sections(root):
    out = {}
    for el in root.findall('gazebo'):
        ref = el.get('reference', '<model>')
        for sub in (el.findall('sensor') or [None]):
            key = 'gazebo[%s]/%s' % (ref, sub.get('name') if sub is not None else el.get('name') or '')
            out[key] = canon(sub if sub is not None else el)
    for el in root.findall('ros2_control'):
        out['ros2_control[%s]' % el.get('name')] = canon(el)
    return out


def fk_transforms(root):
    """{link: T_link<-root}, 关节角全 0 (只按 joint origin 走)"""
    parent_of, T_of = {}, {}
    for j in root.findall('joint'):
        pl, cl = j.find('parent').get('link'), j.find('child').get('link')
        o = j.find('origin')
        xyz = v3(o.get('xyz'), [0, 0, 0]) if o is not None else np.zeros(3)
        rpy = v3(o.get('rpy'), [0, 0, 0]) if o is not None else np.zeros(3)
        parent_of[cl] = pl
        T_of[cl] = tf(xyz, rpy)
    links = [l.get('name') for l in root.findall('link')]
    roots = [l for l in links if l not in parent_of]
    out = {}
    for l in links:
        chain, cur = [], l
        while cur in parent_of:
            chain.append(cur)
            cur = parent_of[cur]
        T = np.eye(4)
        for c in reversed(chain):
            T = T @ T_of[c]
        out[l] = T
    return out, roots


# ─────────────────────────── 主流程 ───────────────────────────

def main():
    ap = argparse.ArgumentParser(description='核对简化版 URDF 与原版的一致性/尺寸')
    default_pkg = Path(__file__).resolve().parent.parent
    ap.add_argument('--full', default=str(default_pkg / 'urdf' / 'AG_robot.urdf.xacro'))
    ap.add_argument('--simple', default=str(default_pkg / 'urdf' / 'AG_robot_simple.urdf.xacro'))
    ap.add_argument('--pitch', default='0', help='雷达倾角(度), 两个模型用同一个值')
    ap.add_argument('--no-gazebo', action='store_true', help='不展开 use_gazebo:=true')
    args = ap.parse_args()

    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('ag_robot_description')
    except Exception:
        pkg_share = str(default_pkg)

    print('原版   : %s' % args.full)
    print('简化版 : %s' % args.simple)
    print('包路径 : %s   (雷达倾角 %s°, use_gazebo=%s)\n'
          % (pkg_share, args.pitch, not args.no_gazebo))

    full = expand(args.full, args.pitch, not args.no_gazebo)
    simple = expand(args.simple, args.pitch, not args.no_gazebo)

    errs, notes = [], []

    # ── A. 关节 ──
    jf = {j.get('name'): j for j in full.findall('joint')}
    js = {j.get('name'): j for j in simple.findall('joint')}
    if set(jf) != set(js):
        errs.append('关节集合不同: 只原版有 %s / 只简化版有 %s'
                    % (sorted(set(jf) - set(js)), sorted(set(js) - set(jf))))
    for n in sorted(set(jf) & set(js)):
        cmp_joint(jf[n], js[n], n, errs, notes)

    # ── B. 惯量 ──
    lf = {l.get('name'): l for l in full.findall('link')}
    ls = {l.get('name'): l for l in simple.findall('link')}
    if set(lf) != set(ls):
        errs.append('link 集合不同: 只原版有 %s / 只简化版有 %s'
                    % (sorted(set(lf) - set(ls)), sorted(set(ls) - set(lf))))
    for n in sorted(set(lf) & set(ls)):
        cmp_inertial(lf[n], ls[n], n, errs)

    # ── C. 传感器与控制器段 ──
    gf, gs = gz_sections(full), gz_sections(simple)
    if set(gf) != set(gs):
        errs.append('gazebo/ros2_control 段不同: 只原版有 %s / 只简化版有 %s'
                    % (sorted(set(gf) - set(gs)), sorted(set(gs) - set(gf))))
    for k in sorted(set(gf) & set(gs)):
        if gf[k] != gs[k]:
            errs.append('段 %s 内容不一致:\n    原版   %s\n    简化版 %s' % (k, gf[k], gs[k]))

    # ── 质量合计 ──
    def total_mass(root):
        return sum(float(l.find('inertial/mass').get('value'))
                   for l in root.findall('link') if l.find('inertial/mass') is not None)
    m_f, m_s = total_mass(full), total_mass(simple)
    if abs(m_f - m_s) > 1e-9:
        errs.append('整车质量不同: %.4f vs %.4f kg' % (m_f, m_s))

    # ── D. 每 link 几何包围盒 ──
    print('=' * 108)
    print('D. 每个 link 的几何包围盒 (link 自身坐标系, 单位 mm; Δ = 简化版 - 原版)')
    print('=' * 108)
    print('%-22s %-27s %-27s %-22s' % ('link', '原版 dx dy dz', '简化版 dx dy dz', 'Δ尺寸 | Δ中心'))
    print('-' * 108)
    worst = (0.0, '')
    for n in sorted(set(lf) & set(ls)):
        lof, hif, _ = link_geom_bbox(lf[n], pkg_share)
        los, his, _ = link_geom_bbox(ls[n], pkg_share)
        if not np.isfinite(lof).all() and not np.isfinite(los).all():
            continue                                               # 两边都没几何(如 tool0)
        dsize = (his - los) - (hif - lof)
        dcen = (his + los) / 2 - (hif + lof) / 2
        deg = ''
        if abs(dsize).max() > 1e-9 or abs(dcen).max() > 1e-9:
            deg = ' (原) / (简化)'
        print('%-22s [%7.1f %7.1f %7.1f] mm     [%7.1f %7.1f %7.1f] mm     Δ [%6.1f %6.1f %6.1f] | [%6.1f %6.1f %6.1f]'
              % (n, *(hif - lof) * 1e3, *(his - los) * 1e3, *(dsize * 1e3), *(dcen * 1e3)))
        w = float(np.abs(dsize * 1e3).max())
        if w > worst[0]:
            worst = (w, n)
    print('-' * 108)
    print('最大尺寸偏差: %.1f mm (%s)' % worst)

    # ── E. 整车包围盒 (FK, 关节角全 0) ──
    def model_bbox(root):
        T, roots = fk_transforms(root)
        if len(roots) != 1:
            print('  (关节树有 %d 个根: %s)' % (len(roots), roots))
        lo = np.full(3, np.inf)
        hi = np.full(3, -np.inf)
        per = {}
        for n, el in ((l.get('name'), l) for l in root.findall('link')):
            a, b, _ = link_geom_bbox(el, pkg_share)
            if not np.isfinite(a).all():
                continue
            T4 = T[n]
            pts = (T4[:3, :3] @ corners(a, b).T).T + T4[:3, 3]
            per[n] = (pts.min(axis=0), pts.max(axis=0))
            lo = np.minimum(lo, pts.min(axis=0))
            hi = np.maximum(hi, pts.max(axis=0))
        return lo, hi, roots[0], per

    print()
    print('=' * 108)
    print('E. 整车包围盒 (从 %s 出发做 FK, 关节角全 0, 单位 mm)' % 'base_footprint')
    print('=' * 108)
    lof, hif, rootf, per_f = model_bbox(full)
    los, his, roots_, per_s = model_bbox(simple)
    print('(注: 这里把每个 link 的几何包围盒按 8 个角点一起旋转后取并集, 比真实外形略保守;')
    print('     两个模型用的是同一套算法, 所以对比 Δ 有效, 绝对值别当整车外形尺寸用)')
    print('%-10s %-34s %-34s %s' % ('方向', '原版 [min, max]', '简化版 [min, max]', 'Δ长度'))
    for i, ax in enumerate('XYZ'):
        print('%-10s [%8.1f, %8.1f] 长 %7.1f   [%8.1f, %8.1f] 长 %7.1f   %+7.1f mm'
              % (ax, lof[i] * 1e3, hif[i] * 1e3, (hif[i] - lof[i]) * 1e3,
                 los[i] * 1e3, his[i] * 1e3, (his[i] - los[i]) * 1e3,
                 ((his[i] - los[i]) - (hif[i] - lof[i])) * 1e3))
    print('-' * 108)
    print('整车外形: 原版 %.1f × %.1f × %.1f mm  ->  简化版 %.1f × %.1f × %.1f mm'
          % (*(hif - lof) * 1e3, *(his - los) * 1e3))
    # 每个方向上是哪个 link 顶到最外沿
    for tag, per, lo_, hi_ in (('原版', per_f, lof, hif), ('简化版', per_s, los, his)):
        drivers = []
        for i, ax in enumerate('XYZ'):
            for idx, edge, bound in ((0, 'min', lo_), (1, 'max', hi_)):
                names = [nm for nm, pp in per.items() if abs(bound[i] - pp[idx][i]) < 1e-9]
                if len(names) == 1:
                    drivers.append('%s%s=%s' % (ax, edge, names[0]))
                elif names:
                    drivers.append('%s%s=%s(共%d个)' % (ax, edge, names[0], len(names)))
        print('   %s 整车尺寸由这些 link 决定: %s' % (tag, '  '.join(drivers)))

    # ── F. 规模 ──
    print()
    print('=' * 108)
    print('F. 规模 (渲染与物理各算一遍网格; 图元不产生三角形)')
    print('=' * 108)
    vf_vis = vf_col = 0
    pf_vis = pf_col = 0
    for n in sorted(set(lf) & set(ls)):
        a, b = link_mesh_tris(lf[n], pkg_share)
        vf_vis += a
        vf_col += b
        a2, b2 = link_n_prim(ls[n])
        pf_vis += a2
        pf_col += b2
    print('原版  : visual 网格 %10s 面 + collision 网格 %10s 面 = %10s 面'
          % (format(vf_vis, ','), format(vf_col, ','), format(vf_vis + vf_col, ',')))
    print('        (Gazebo 里这 2 份都要加载: 渲染用 visual, 物理与雷达射线用 collision)')
    print('简化版: visual 图元 %10d 个 + collision 图元 %10d 个 = %10d 个, 三角形 0 面'
          % (pf_vis, pf_col, pf_vis + pf_col))

    # ── 结论 ──
    print()
    print('=' * 108)
    if notes:
        print('说明(不算失败):')
        for n in notes:
            print('  · %s' % n)
    if errs:
        print('❌ A/B/C 有 %d 处不一致 —— 简化版动到了不该动的东西:' % len(errs))
        for e in errs:
            print('  · %s' % e)
        return 1
    print('✅ A(关节) B(惯量) C(传感器+控制器) 三项与原版**完全一致**, 整车质量 %.3f kg 不变。'
          % m_f)
    print('   D/E 的几何偏差见上表: 整车外形尺寸偏差在 %.1f mm 以内。' % abs((his - los) - (hif - lof)).max())
    return 0


if __name__ == '__main__':
    sys.exit(main())
