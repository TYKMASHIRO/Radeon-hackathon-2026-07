"""Convert the local 3d66 joystick/throttle FBX into a MuJoCo XML include.

Run with Blender:
```
blender --background --python apps/convert_fbx_controls.py
```
"""

from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
FBX_PATH = REPO_ROOT / "3d66.com_JDH5455235936.fbx"
OUTPUT_DIR = PROJECT_ROOT / "models/derived/cockpit/imported_3d66"
MESH_DIR = OUTPUT_DIR / "meshes"
OBJ_PATH = MESH_DIR / "3d66_JDH5455235936_controls.obj"
XML_PATH = OUTPUT_DIR / "fbx_controls.xml"
REPORT_PATH = PROJECT_ROOT / "reports/fbx_controls_conversion_report.json"
TARGET_MAX_DIMENSION_M = 0.45
WORLD_OFFSET_M = (0.24, 0.0, 0.62)
THROTTLE_PIVOT_M = (-0.12, 0.30, 0.00)
STICK_PIVOT_M = (0.06, 0.0, 0.08)
THROTTLE_SITE_LOCAL_M = (0.02, 0.0, 0.07)


def _values(values: tuple[float, ...] | list[float]) -> str:
    return " ".join(f"{float(value):.10g}" for value in values)


def _write_mjcf_xml(
    obj_path: Path,
    xml_path: Path,
    stick_site_local_m: tuple[float, float, float],
) -> None:
    root = ET.Element("mujoco", {"model": "imported_3d66_joystick_throttle"})
    ET.SubElement(root, "compiler", {"angle": "radian"})
    ET.SubElement(root, "option", {"gravity": "0 0 -9.81"})
    asset = ET.SubElement(root, "asset")
    meshes = {
        "fbx_controls_static_mesh": MESH_DIR / "3d66_controls_static.obj",
        "fbx_throttle_moving_mesh": MESH_DIR / "3d66_throttle_moving.obj",
        "fbx_stick_static_mesh": MESH_DIR / "3d66_stick_static.obj",
        "fbx_stick_moving_mesh": MESH_DIR / "3d66_stick_moving.obj",
        "imported_3d66_controls_mesh": obj_path,
    }
    for name, mesh_path in meshes.items():
        ET.SubElement(
            asset,
            "mesh",
            {
                "name": name,
                "file": Path(mesh_path.relative_to(xml_path.parent)).as_posix(),
            },
        )
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", {"name": "imported_controls_key", "pos": "-0.4 -0.6 1.8"})
    body = ET.SubElement(
        world,
        "body",
        {
            "name": "imported_3d66_joystick_throttle",
            "pos": _values(WORLD_OFFSET_M),
        },
    )
    ET.SubElement(
        body,
        "geom",
        {
            "name": "fbx_controls_static_visual",
            "type": "mesh",
            "mesh": "fbx_controls_static_mesh",
            "contype": "0",
            "conaffinity": "0",
            "rgba": "0.18 0.18 0.17 1",
        },
    )
    ET.SubElement(
        body,
        "geom",
        {
            "name": "fbx_stick_static_visual",
            "type": "mesh",
            "mesh": "fbx_stick_static_mesh",
            "contype": "0",
            "conaffinity": "0",
            "rgba": "0.16 0.17 0.17 1",
        },
    )
    throttle = ET.SubElement(body, "body", {"name": "fbx_throttle_handle_body", "pos": _values(THROTTLE_PIVOT_M)})
    ET.SubElement(
        throttle,
        "inertial",
        {
            "pos": _values(THROTTLE_SITE_LOCAL_M),
            "mass": "0.25",
            "diaginertia": "0.001 0.001 0.001",
        },
    )
    ET.SubElement(
        throttle,
        "joint",
        {
            "name": "throttle_joint",
            "type": "slide",
            "axis": "1 0 0",
            "range": "0 0.16",
            "damping": "4.0",
            "frictionloss": "2.0",
        },
    )
    ET.SubElement(
        throttle,
        "geom",
        {
            "name": "fbx_throttle_visual",
            "type": "mesh",
            "mesh": "fbx_throttle_moving_mesh",
            "density": "0",
            "contype": "0",
            "conaffinity": "0",
            "rgba": "0.21 0.20 0.18 1",
        },
    )
    ET.SubElement(
        throttle,
        "site",
        {
            "name": "throttle_force_site",
            "pos": _values(THROTTLE_SITE_LOCAL_M),
            "size": "0.012",
        },
    )
    ET.SubElement(throttle, "geom", {"name": "throttle_thumb_switch", "type": "box", "pos": "0.035 -0.055 0.085", "size": "0.012 0.006 0.008", "density": "0", "contype": "0", "conaffinity": "0", "rgba": "0.05 0.18 0.35 1"})
    ET.SubElement(throttle, "geom", {"name": "throttle_index_switch", "type": "box", "pos": "0.055 0.045 0.09", "size": "0.010 0.006 0.006", "density": "0", "contype": "0", "conaffinity": "0", "rgba": "0.35 0.06 0.05 1"})
    ET.SubElement(throttle, "site", {"name": "throttle_thumb_switch_site", "pos": "0.035 -0.063 0.085", "size": "0.006", "rgba": "0.05 0.18 0.35 1"})
    ET.SubElement(throttle, "site", {"name": "throttle_index_switch_site", "pos": "0.055 0.053 0.09", "size": "0.006", "rgba": "0.35 0.06 0.05 1"})

    stick_roll = ET.SubElement(body, "body", {"name": "fbx_stick_roll_frame", "pos": _values(STICK_PIVOT_M)})
    ET.SubElement(
        stick_roll,
        "geom",
        {
            "name": "fbx_stick_roll_hub",
            "type": "sphere",
            "size": "0.015",
            "mass": "0.02",
            "contype": "0",
            "conaffinity": "0",
            "rgba": "0.16 0.17 0.17 1",
        },
    )
    ET.SubElement(
        stick_roll,
        "joint",
        {
            "name": "stick_roll_joint",
            "type": "hinge",
            "axis": "1 0 0",
            "range": "-0.3 0.3",
            "stiffness": "5.0",
            "damping": "0.15",
            "frictionloss": "0.05",
        },
    )
    stick_pitch = ET.SubElement(stick_roll, "body", {"name": "fbx_stick_pitch_frame"})
    ET.SubElement(
        stick_pitch,
        "inertial",
        {
            "pos": _values(tuple(0.5 * value for value in stick_site_local_m)),
            "mass": "0.30",
            "diaginertia": "0.001 0.001 0.001",
        },
    )
    ET.SubElement(
        stick_pitch,
        "joint",
        {
            "name": "stick_pitch_joint",
            "type": "hinge",
            "axis": "0 1 0",
            "range": "-0.3 0.3",
            "stiffness": "5.0",
            "damping": "0.15",
            "frictionloss": "0.05",
        },
    )
    ET.SubElement(
        stick_pitch,
        "geom",
        {
            "name": "fbx_stick_visual",
            "type": "mesh",
            "mesh": "fbx_stick_moving_mesh",
            "density": "0",
            "contype": "0",
            "conaffinity": "0",
            "rgba": "0.12 0.13 0.13 1",
        },
    )
    ET.SubElement(
        stick_pitch,
        "site",
        {
            "name": "stick_force_site",
            "pos": _values(stick_site_local_m),
            "size": "0.012",
        },
    )
    ET.SubElement(
        world,
        "camera",
        {
            "name": "imported_3d66_controls_view",
            "pos": "-0.55 -0.80 0.95",
            "xyaxes": "0.82 -0.57 0 0.28 0.40 0.87",
            "fovy": "50",
        },
    )
    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "jointpos", {"name": "stick_roll_position", "joint": "stick_roll_joint"})
    ET.SubElement(sensor, "jointvel", {"name": "stick_roll_velocity", "joint": "stick_roll_joint"})
    ET.SubElement(sensor, "jointpos", {"name": "stick_pitch_position", "joint": "stick_pitch_joint"})
    ET.SubElement(sensor, "jointvel", {"name": "stick_pitch_velocity", "joint": "stick_pitch_joint"})
    ET.SubElement(sensor, "force", {"name": "stick_force", "site": "stick_force_site"})
    ET.SubElement(sensor, "torque", {"name": "stick_torque", "site": "stick_force_site"})
    ET.SubElement(sensor, "jointpos", {"name": "throttle_position", "joint": "throttle_joint"})
    ET.SubElement(sensor, "jointvel", {"name": "throttle_velocity", "joint": "throttle_joint"})
    ET.SubElement(sensor, "force", {"name": "throttle_force", "site": "throttle_force_site"})
    xml_path.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)


def _run_blender_conversion() -> dict:
    try:
        import bpy
        from mathutils import Vector
    except ImportError as exc:
        raise RuntimeError("This converter must be run inside Blender Python.") from exc

    if not FBX_PATH.is_file():
        raise FileNotFoundError(f"FBX source not found: {FBX_PATH}")

    MESH_DIR.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.import_scene.fbx(filepath=str(FBX_PATH))

    mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not mesh_objects:
        raise RuntimeError(f"No mesh objects were imported from {FBX_PATH}")

    all_corners = [
        obj.matrix_world @ Vector(corner)
        for obj in mesh_objects
        for corner in obj.bound_box
    ]
    source_mins = [min(corner[index] for corner in all_corners) for index in range(3)]
    source_maxs = [max(corner[index] for corner in all_corners) for index in range(3)]
    source_dimensions = [
        source_maxs[index] - source_mins[index] for index in range(3)
    ]
    max_dimension = max(source_dimensions)
    if max_dimension <= 0.0:
        raise RuntimeError("Imported FBX mesh has a zero-size bounding box")

    scale = TARGET_MAX_DIMENSION_M / max_dimension
    center_x = 0.5 * (source_mins[0] + source_maxs[0])
    center_y = 0.5 * (source_mins[1] + source_maxs[1])
    min_z = source_mins[2]
    normalized_mins = [
        (source_mins[0] - center_x) * scale,
        (source_mins[1] - center_y) * scale,
        0.0,
    ]
    normalized_maxs = [
        (source_maxs[0] - center_x) * scale,
        (source_maxs[1] - center_y) * scale,
        (source_maxs[2] - min_z) * scale,
    ]
    normalized_dimensions = [
        normalized_maxs[index] - normalized_mins[index] for index in range(3)
    ]
    object_groups = {
        "controls_static": [],
        "throttle_moving": [],
        "stick_static": [],
        "stick_moving": [],
    }
    object_group: dict[str, str] = {}
    for obj in mesh_objects:
        corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
        object_center = [
            0.5 * (min(corner[index] for corner in corners) + max(corner[index] for corner in corners))
            for index in range(3)
        ]
        local_center_x = (object_center[0] - center_x) * scale
        local_center_z = (object_center[2] - min_z) * scale
        if local_center_x < 0.0:
            group = "throttle_moving" if local_center_z > THROTTLE_PIVOT_M[2] + 0.03 else "controls_static"
        else:
            group = "stick_moving" if local_center_z > STICK_PIVOT_M[2] + 0.03 else "stick_static"
        object_groups[group].append(obj.name)
        object_group[obj.name] = group

    def normalized_target(vertex: Vector, matrix: object) -> Vector:
        world = matrix @ vertex
        normalized = Vector(
            (
                (world.x - center_x) * scale,
                (world.y - center_y) * scale,
                (world.z - min_z) * scale,
            )
        )
        # Blender's X-forward/Z-up OBJ export maps (x, y, z) to
        # MuJoCo coordinates (y, -x, z).
        return Vector((normalized.y, -normalized.x, normalized.z))

    group_points: dict[str, list[Vector]] = {
        group: [] for group in object_groups
    }
    for obj in mesh_objects:
        matrix = obj.matrix_world.copy()
        group = object_group[obj.name]
        group_points[group].extend(
            normalized_target(vertex.co, matrix)
            for vertex in obj.data.vertices
        )

    def quantile(values: list[float], fraction: float) -> float:
        ordered = sorted(values)
        position = fraction * (len(ordered) - 1)
        lower = int(position)
        upper = min(lower + 1, len(ordered) - 1)
        blend = position - lower
        return (1.0 - blend) * ordered[lower] + blend * ordered[upper]

    def mean_points(points: list[Vector]) -> Vector:
        if not points:
            raise RuntimeError("Cannot average an empty FBX vertex group")
        total = Vector((0.0, 0.0, 0.0))
        for point in points:
            total += point
        return total / len(points)

    throttle_points = group_points["throttle_moving"]
    throttle_grip_threshold = quantile(
        [point.z for point in throttle_points],
        0.50,
    )
    throttle_grip_source = mean_points(
        [
            point
            for point in throttle_points
            if point.z >= throttle_grip_threshold
        ]
    )
    throttle_source_pivot = (
        throttle_grip_source - Vector(THROTTLE_SITE_LOCAL_M)
    )

    stick_points = group_points["stick_moving"]
    stick_static_points = group_points["stick_static"]
    stick_bottom_threshold = quantile(
        [point.z for point in stick_points],
        0.10,
    )
    stick_bottom = mean_points(
        [
            point
            for point in stick_points
            if point.z <= stick_bottom_threshold
        ]
    )
    overlap_low = max(
        min(point.z for point in stick_points),
        min(point.z for point in stick_static_points),
    )
    overlap_high = min(
        max(point.z for point in stick_points),
        max(point.z for point in stick_static_points),
    )
    if overlap_low >= overlap_high:
        raise RuntimeError("Stick static and moving meshes have no pivot overlap")
    stick_source_pivot = Vector(
        (
            stick_bottom.x,
            stick_bottom.y,
            0.5 * (overlap_low + overlap_high),
        )
    )
    stick_grip_threshold = quantile(
        [point.z for point in stick_points],
        0.80,
    )
    stick_grip_source = mean_points(
        [
            point
            for point in stick_points
            if point.z >= stick_grip_threshold
        ]
    )
    stick_site_local = stick_grip_source - stick_source_pivot

    throttle_target_pivot = Vector(THROTTLE_PIVOT_M)
    stick_target_pivot = Vector(STICK_PIVOT_M)
    group_offsets = {
        "controls_static": throttle_target_pivot - throttle_source_pivot,
        "throttle_moving": -throttle_source_pivot,
        "stick_static": stick_target_pivot - stick_source_pivot,
        "stick_moving": -stick_source_pivot,
    }

    def target_to_blender(target: Vector) -> Vector:
        return Vector((-target.y, target.x, target.z))

    for obj in mesh_objects:
        group = object_group[obj.name]
        mesh = obj.data.copy()
        matrix = obj.matrix_world.copy()
        offset = group_offsets[group]
        for vertex in mesh.vertices:
            target = normalized_target(vertex.co, matrix) + offset
            vertex.co = target_to_blender(target)
        obj.data = mesh
        obj.location = (0.0, 0.0, 0.0)
        obj.rotation_euler = (0.0, 0.0, 0.0)
        obj.scale = (1.0, 1.0, 1.0)

    def export_group(group: str, filename: str) -> object:
        bpy.ops.object.select_all(action="DESELECT")
        selected = [obj for obj in bpy.context.scene.objects if obj.name in object_groups[group]]
        if not selected:
            raise RuntimeError(f"No FBX objects classified into {group}")
        for obj in selected:
            obj.select_set(True)
        bpy.context.view_layer.objects.active = selected[0]
        if len(selected) > 1:
            bpy.ops.object.join()
        joined_group = bpy.context.view_layer.objects.active
        joined_group.name = f"{group}_joined"
        if hasattr(bpy.ops.wm, "obj_export"):
            bpy.ops.wm.obj_export(
                filepath=str(MESH_DIR / filename),
                export_selected_objects=True,
                export_materials=False,
                export_triangulated_mesh=True,
                forward_axis="X",
                up_axis="Z",
            )
        else:
            bpy.ops.export_scene.obj(
                filepath=str(MESH_DIR / filename),
                use_selection=True,
                use_materials=False,
                use_triangles=True,
                axis_forward="X",
                axis_up="Z",
            )
        return joined_group

    joined_groups = {
        "controls_static": export_group(
            "controls_static",
            "3d66_controls_static.obj",
        ),
        "throttle_moving": export_group(
            "throttle_moving",
            "3d66_throttle_moving.obj",
        ),
        "stick_static": export_group(
            "stick_static",
            "3d66_stick_static.obj",
        ),
        "stick_moving": export_group(
            "stick_moving",
            "3d66_stick_moving.obj",
        ),
    }

    for group, obj in joined_groups.items():
        if group not in {"throttle_moving", "stick_moving"}:
            continue
        target_pivot = (
            throttle_target_pivot
            if group == "throttle_moving"
            else stick_target_pivot
        )
        for vertex in obj.data.vertices:
            local_target = Vector(
                (vertex.co.y, -vertex.co.x, vertex.co.z)
            )
            vertex.co = target_to_blender(local_target + target_pivot)

    bpy.ops.object.select_all(action="SELECT")
    if len(joined_groups) > 1:
        bpy.ops.object.join()
    joined = bpy.context.view_layer.objects.active
    joined.name = "imported_3d66_controls_mesh"
    bpy.ops.object.select_all(action="DESELECT")
    joined.select_set(True)
    bpy.context.view_layer.objects.active = joined
    if hasattr(bpy.ops.wm, "obj_export"):
        bpy.ops.wm.obj_export(
            filepath=str(OBJ_PATH),
            export_selected_objects=True,
            export_materials=False,
            export_triangulated_mesh=True,
            forward_axis="X",
            up_axis="Z",
        )
    else:
        bpy.ops.export_scene.obj(
            filepath=str(OBJ_PATH),
            use_selection=True,
            use_materials=False,
            use_triangles=True,
            axis_forward="X",
            axis_up="Z",
        )

    stick_site_local_m = tuple(float(value) for value in stick_site_local)
    _write_mjcf_xml(OBJ_PATH, XML_PATH, stick_site_local_m)
    report = {
        "source_fbx": str(FBX_PATH),
        "source_size_bytes": FBX_PATH.stat().st_size,
        "output_obj": str(OBJ_PATH),
        "output_xml": str(XML_PATH),
        "mesh_object_count_imported": len(mesh_objects),
        "joined_mesh_name": joined.name,
        "object_groups": object_groups,
        "source_bounding_box_min": source_mins,
        "source_bounding_box_max": source_maxs,
        "source_dimensions": source_dimensions,
        "normalization": {
            "target_max_dimension_m": TARGET_MAX_DIMENSION_M,
            "scale_applied": scale,
            "bounding_box_min_m": normalized_mins,
            "bounding_box_max_m": normalized_maxs,
            "dimensions_m": normalized_dimensions,
            "world_offset_m": WORLD_OFFSET_M,
            "throttle_pivot_m": THROTTLE_PIVOT_M,
            "stick_pivot_m": STICK_PIVOT_M,
        },
        "alignment": {
            "obj_axis_transform": "(x, y, z) -> (y, -x, z)",
            "throttle_source_pivot_m": list(throttle_source_pivot),
            "throttle_target_pivot_m": list(throttle_target_pivot),
            "throttle_site_local_m": list(THROTTLE_SITE_LOCAL_M),
            "stick_source_pivot_m": list(stick_source_pivot),
            "stick_target_pivot_m": list(stick_target_pivot),
            "stick_site_local_m": list(stick_site_local),
            "stick_static_is_world_fixed": True,
        },
        "mjcf_status": (
            "The XML imports split FBX-derived OBJ meshes as articulated MuJoCo "
            "bodies. throttle_joint, stick_roll_joint, and stick_pitch_joint are "
            "the control joints driven by the humanoid pilot sequence."
        ),
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


def main() -> None:
    report = _run_blender_conversion()
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
