"""Inspect mesh object bounds inside the local 3d66 FBX with Blender."""

from __future__ import annotations

import json
from pathlib import Path

import bpy
from mathutils import Vector


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
FBX_PATH = REPO_ROOT / "3d66.com_JDH5455235936.fbx"
REPORT_PATH = PROJECT_ROOT / "reports/fbx_controls_object_report.json"


def main() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.import_scene.fbx(filepath=str(FBX_PATH))

    objects: list[dict] = []
    for obj in bpy.context.scene.objects:
        if obj.type != "MESH":
            continue
        corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
        mins = [min(corner[index] for corner in corners) for index in range(3)]
        maxs = [max(corner[index] for corner in corners) for index in range(3)]
        dims = [maxs[index] - mins[index] for index in range(3)]
        center = [0.5 * (mins[index] + maxs[index]) for index in range(3)]
        objects.append(
            {
                "name": obj.name,
                "vertex_count": len(obj.data.vertices),
                "face_count": len(obj.data.polygons),
                "center": center,
                "dimensions": dims,
                "bounding_box_min": mins,
                "bounding_box_max": maxs,
            }
        )

    objects.sort(key=lambda item: (item["center"][1], item["center"][0], item["center"][2]))
    report = {
        "source_fbx": str(FBX_PATH),
        "mesh_object_count": len(objects),
        "objects": objects,
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
