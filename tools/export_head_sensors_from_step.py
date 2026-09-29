#!/usr/bin/env python3
"""Export the head, RealSense and VLP-16 meshes from the SolidWorks STEP.

This is a development utility, not a runtime dependency of the ROS package.
It needs OpenCascade's Python bindings (the ``OCP`` package).  The exported
STLs use millimetres, matching the existing chassis / lift meshes in this
package; URDF must therefore load them with scale="0.001 0.001 0.001".

The source assembly uses the SolidWorks root coordinates.  Existing robot
meshes use a base-local coordinate system obtained by rotating the assembly
180 degrees about Z and placing the chassis assembly origin at (0, 0, 0).
This script derives that translation from the chassis occurrence instead of
hard-coding it, then writes each new mesh in its own URDF link frame.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_ColorRGBA
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.XCAFPrs import XCAFPrs, XCAFPrs_IndexedDataMapOfShapeStyle
from OCP.gp import gp_Trsf


CHASSIS_NAME = "顶配独立悬挂四驱车"
HEAD_NAME = "头盔_原始备份_20260925_084324"
CAMERA_NAME = "realsense_d435"
LIDAR_NAME = "VLP-16"

# The CAD VLP-16 body is 72 mm high.  Keep velodyne_link at the body centre,
# as in the previous URDF, so the simulated scan plane remains +10 mm in Z.
LIDAR_HALF_HEIGHT_MM = 36.0


def label_name(label: TDF_Label) -> str:
    attribute = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attribute):
        return attribute.Get().ToExtString()
    return ""


def referred_label(component: TDF_Label) -> TDF_Label:
    referred = TDF_Label()
    if not XCAFDoc_ShapeTool.GetReferredShape_s(component, referred):
        raise RuntimeError(f"assembly label {label_name(component)!r} is not a reference")
    return referred


def component_origin(component: TDF_Label) -> tuple[float, float, float]:
    translation = XCAFDoc_ShapeTool.GetLocation_s(component).Transformation().TranslationPart()
    return translation.X(), translation.Y(), translation.Z()


def trsf_from_rows(rows: tuple[tuple[float, float, float, float], ...]) -> gp_Trsf:
    result = gp_Trsf()
    result.SetValues(*(value for row in rows for value in row))
    return result


def apply_point(
    rows: tuple[tuple[float, float, float, float], ...],
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z = point
    return tuple(row[0] * x + row[1] * y + row[2] * z + row[3] for row in rows)


def compose(
    left: tuple[tuple[float, float, float, float], ...],
    right: tuple[tuple[float, float, float, float], ...],
) -> tuple[tuple[float, float, float, float], ...]:
    """Compose two 3x4 rigid transforms, returning left(right(point))."""
    l4 = (*left, (0.0, 0.0, 0.0, 1.0))
    r4 = (*right, (0.0, 0.0, 0.0, 1.0))
    return tuple(
        tuple(sum(l4[i][k] * r4[k][j] for k in range(4)) for j in range(4))
        for i in range(3)
    )


def inverse_rigid(
    rows: tuple[tuple[float, float, float, float], ...]
) -> tuple[tuple[float, float, float, float], ...]:
    rotation = tuple(tuple(rows[i][j] for j in range(3)) for i in range(3))
    translation = tuple(rows[i][3] for i in range(3))
    rotation_t = tuple(tuple(rotation[j][i] for j in range(3)) for i in range(3))
    inverse_translation = tuple(
        -sum(rotation_t[i][j] * translation[j] for j in range(3)) for i in range(3)
    )
    return tuple((*rotation_t[i], inverse_translation[i]) for i in range(3))


def shape_metrics(shape) -> dict[str, object]:
    bounds = Bnd_Box()
    # Add_s includes imported STEP shape tolerances in the box.  The VLP-16
    # carries a 4.255 mm face tolerance, which would incorrectly report a
    # 111.811 mm body instead of the actual 103.300 mm mesh.  AddOptimal with
    # useShapeTolerance=False returns the physical surface extents.
    BRepBndLib.AddOptimal_s(shape, bounds, False, False)
    xmin, ymin, zmin, xmax, ymax, zmax = bounds.Get()
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, properties, True, False, False)
    centre = properties.CentreOfMass()
    return {
        "bounds_mm": [xmin, ymin, zmin, xmax, ymax, zmax],
        "size_mm": [xmax - xmin, ymax - ymin, zmax - zmin],
        "volume_mm3": properties.Mass(),
        "centre_of_volume_mm": [centre.X(), centre.Y(), centre.Z()],
    }


def cad_surface_color(referred: TDF_Label) -> list[float] | None:
    styles = XCAFPrs_IndexedDataMapOfShapeStyle()
    XCAFPrs.CollectStyleSettings_s(referred, TopLoc_Location(), styles)
    for index in range(1, styles.Extent() + 1):
        style = styles.FindFromIndex(index)
        if style.IsSetColorSurf():
            rgba: Quantity_ColorRGBA = style.GetColorSurfRGBA()
            rgb = rgba.GetRGB()
            return [rgb.Red(), rgb.Green(), rgb.Blue(), rgba.Alpha()]
    return None


def write_binary_stl(shape, path: Path, linear_deflection_mm: float) -> None:
    # The collision geometry is primitive-based in URDF.  This mesh is only
    # for rendering, so 0.15 rad / configurable chord error is a good balance
    # between faithful curves and Gazebo load time.
    mesher = BRepMesh_IncrementalMesh(shape, linear_deflection_mm, False, 0.15, True)
    mesher.Perform()
    if not mesher.IsDone():
        raise RuntimeError(f"OpenCascade failed to mesh {path.name}")
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    temporary = path.with_suffix(path.suffix + ".tmp")
    if not writer.Write(shape, os.fspath(temporary)):
        raise RuntimeError(f"OpenCascade failed to write {path}")
    os.replace(temporary, path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("step", type=Path, help="SolidWorks STEP assembly")
    parser.add_argument("output_dir", type=Path, help="package meshes directory")
    parser.add_argument(
        "--linear-deflection-mm",
        type=float,
        default=0.10,
        help="visual mesh chord error in millimetres (default: 0.10)",
    )
    parser.add_argument(
        "--metadata",
        type=Path,
        help="optional JSON report path (defaults to output_dir/head_sensors_cad.json)",
    )
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    metadata_path = args.metadata or args.output_dir / "head_sensors_cad.json"
    with args.step.open("rb") as source_file:
        source_sha256 = hashlib.file_digest(source_file, "sha256").hexdigest()

    application = XCAFApp_Application.GetApplication_s()
    document = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    application.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), document)

    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)
    if reader.ReadFile(os.fspath(args.step)) != IFSelect_RetDone:
        raise RuntimeError(f"cannot read STEP file: {args.step}")
    if not reader.Transfer(document):
        raise RuntimeError(f"cannot transfer STEP assembly: {args.step}")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    free_shapes = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free_shapes)
    if free_shapes.Length() != 1:
        raise RuntimeError(f"expected one root assembly, got {free_shapes.Length()}")

    components = TDF_LabelSequence()
    if not XCAFDoc_ShapeTool.GetComponents_s(free_shapes.Value(1), components, False):
        raise RuntimeError("STEP root is not an assembly")

    by_name: dict[str, TDF_Label] = {}
    for index in range(1, components.Length() + 1):
        component = components.Value(index)
        name = label_name(referred_label(component))
        if name in {CHASSIS_NAME, HEAD_NAME, CAMERA_NAME, LIDAR_NAME}:
            by_name[name] = component
    missing = {CHASSIS_NAME, HEAD_NAME, CAMERA_NAME, LIDAR_NAME} - by_name.keys()
    if missing:
        raise RuntimeError(f"required STEP components are missing: {sorted(missing)}")

    chassis_origin_step = component_origin(by_name[CHASSIS_NAME])
    # base = Rz(pi) * step + translation; make the chassis occurrence origin 0.
    step_to_base = (
        (-1.0, 0.0, 0.0, chassis_origin_step[0]),
        (0.0, -1.0, 0.0, chassis_origin_step[1]),
        (0.0, 0.0, 1.0, -chassis_origin_step[2]),
    )

    head_origin_base = apply_point(step_to_base, component_origin(by_name[HEAD_NAME]))
    camera_origin_base = apply_point(step_to_base, component_origin(by_name[CAMERA_NAME]))
    lidar_mount_origin_base = apply_point(step_to_base, component_origin(by_name[LIDAR_NAME]))
    lidar_link_origin_base = (
        lidar_mount_origin_base[0],
        lidar_mount_origin_base[1],
        lidar_mount_origin_base[2] + LIDAR_HALF_HEIGHT_MM,
    )

    # link -> base transforms.  The head and lidar retain the internal CAD/base
    # axes.  camera_link follows REP-103: +X forward, +Y left, +Z up.  Since
    # vehicle forward is -Y in the internal base coordinates, that is Rz(-90°).
    head_to_base = (
        (1.0, 0.0, 0.0, head_origin_base[0]),
        (0.0, 1.0, 0.0, head_origin_base[1]),
        (0.0, 0.0, 1.0, head_origin_base[2]),
    )
    camera_to_base = (
        (0.0, 1.0, 0.0, camera_origin_base[0]),
        (-1.0, 0.0, 0.0, camera_origin_base[1]),
        (0.0, 0.0, 1.0, camera_origin_base[2]),
    )
    lidar_to_base = (
        (1.0, 0.0, 0.0, lidar_link_origin_base[0]),
        (0.0, 1.0, 0.0, lidar_link_origin_base[1]),
        (0.0, 0.0, 1.0, lidar_link_origin_base[2]),
    )

    exports = (
        ("head", HEAD_NAME, "head_shell.stl", head_to_base),
        # The SolidWorks product label says "realsense_d435", while the
        # installed hardware was confirmed to be a D435i.  The enclosure
        # geometry in this STEP is used verbatim and the exported asset is
        # named for the actual device.
        ("camera", CAMERA_NAME, "realsense_d435i.stl", camera_to_base),
        ("lidar", LIDAR_NAME, "vlp16.stl", lidar_to_base),
    )
    report: dict[str, object] = {
        "source_step": os.fspath(args.step.resolve()),
        "source_step_sha256": source_sha256,
        "source_root": label_name(free_shapes.Value(1)),
        "units": "millimetres",
        "notes": [
            "STEP source contains no material density or mass properties.",
            "STEP component label is realsense_d435; the physical model was "
            "confirmed by the user as D435i.",
            "XCAF colors are linear RGB; URDF cad_head_surface uses the "
            "equivalent sRGB 0.792157 0.819608 0.933333.",
        ],
        "chassis_origin_step_mm": chassis_origin_step,
        "step_to_base": step_to_base,
        "cad_surface_color_space": "linear RGB",
        "cad_surface_color_rgba": {},
        "links": {},
    }

    for key, source_name, filename, link_to_base in exports:
        component = by_name[source_name]
        world_shape = XCAFDoc_ShapeTool.GetShape_s(component)
        world_to_link = compose(inverse_rigid(link_to_base), step_to_base)
        local_shape = BRepBuilderAPI_Transform(
            world_shape, trsf_from_rows(world_to_link), True, False
        ).Shape()
        output_path = args.output_dir / filename
        write_binary_stl(local_shape, output_path, args.linear_deflection_mm)
        report["cad_surface_color_rgba"][key] = cad_surface_color(
            referred_label(component)
        )
        report["links"][key] = {
            "source_component": source_name,
            "mesh": filename,
            "component_origin_step_mm": component_origin(component),
            "origin_base_mm": [row[3] for row in link_to_base],
            "rpy_base_rad": [0.0, 0.0, -math.pi / 2.0 if key == "camera" else 0.0],
            **shape_metrics(local_shape),
        }
        print(f"wrote {output_path}")

    temporary_metadata = metadata_path.with_suffix(metadata_path.suffix + ".tmp")
    temporary_metadata.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    os.replace(temporary_metadata, metadata_path)
    print(f"wrote {metadata_path}")


if __name__ == "__main__":
    main()
