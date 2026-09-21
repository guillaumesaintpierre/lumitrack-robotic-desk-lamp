import bpy
import json
import sys
from pathlib import Path
from mathutils import Vector


# ============================================================
# Paths
# ============================================================

PROJECT_ROOT = Path.cwd()

FBX_PATH = (
    PROJECT_ROOT
    / "mechanical"
    / "reference"
    / "FORSA_Work_lamp_white_2015.FBX"
)

REPORT_PATH = (
    PROJECT_ROOT
    / "mechanical"
    / "reference"
    / "forsa_model_report.json"
)


# ============================================================
# Utilities
# ============================================================

def clear_scene():
    bpy.ops.object.select_all(
        action="SELECT"
    )

    bpy.ops.object.delete(
        use_global=False
    )


def world_bounding_box(obj):
    """
    Return world-space bounding box coordinates.
    """

    corners = [
        obj.matrix_world @ Vector(corner)
        for corner in obj.bound_box
    ]

    xs = [p.x for p in corners]
    ys = [p.y for p in corners]
    zs = [p.z for p in corners]

    minimum = Vector(
        (
            min(xs),
            min(ys),
            min(zs),
        )
    )

    maximum = Vector(
        (
            max(xs),
            max(ys),
            max(zs),
        )
    )

    dimensions = (
        maximum
        - minimum
    )

    centre = (
        minimum
        + maximum
    ) / 2.0

    return (
        minimum,
        maximum,
        dimensions,
        centre,
    )


def vector_to_list(vector):
    return [
        float(vector.x),
        float(vector.y),
        float(vector.z),
    ]


# ============================================================
# Main
# ============================================================

def main():

    if not FBX_PATH.exists():

        raise FileNotFoundError(
            f"FBX file not found:\n{FBX_PATH}"
        )

    print()
    print(
        "=== LumiTrack FORSA FBX inspection ==="
    )
    print()

    print(
        f"Loading:\n{FBX_PATH}"
    )

    clear_scene()

    bpy.ops.import_scene.fbx(
        filepath=str(FBX_PATH)
    )

    bpy.context.view_layer.update()

    objects = []

    scene_min = Vector(
        (
            float("inf"),
            float("inf"),
            float("inf"),
        )
    )

    scene_max = Vector(
        (
            float("-inf"),
            float("-inf"),
            float("-inf"),
        )
    )

    mesh_objects = [
        obj
        for obj in bpy.context.scene.objects
        if obj.type == "MESH"
    ]

    print()
    print(
        f"Mesh objects: {len(mesh_objects)}"
    )
    print()

    for obj in mesh_objects:

        (
            minimum,
            maximum,
            dimensions,
            centre,
        ) = world_bounding_box(
            obj
        )

        scene_min.x = min(
            scene_min.x,
            minimum.x,
        )

        scene_min.y = min(
            scene_min.y,
            minimum.y,
        )

        scene_min.z = min(
            scene_min.z,
            minimum.z,
        )

        scene_max.x = max(
            scene_max.x,
            maximum.x,
        )

        scene_max.y = max(
            scene_max.y,
            maximum.y,
        )

        scene_max.z = max(
            scene_max.z,
            maximum.z,
        )

        data = {
            "name": obj.name,

            "location": vector_to_list(
                obj.matrix_world.translation
            ),

            "dimensions": vector_to_list(
                dimensions
            ),

            "bbox_center": vector_to_list(
                centre
            ),

            "bbox_min": vector_to_list(
                minimum
            ),

            "bbox_max": vector_to_list(
                maximum
            ),
        }

        objects.append(
            data
        )

        print(
            f"{obj.name}"
        )

        print(
            "  dimensions:",
            [
                round(v, 4)
                for v
                in vector_to_list(
                    dimensions
                )
            ],
        )

        print(
            "  center:",
            [
                round(v, 4)
                for v
                in vector_to_list(
                    centre
                )
            ],
        )

    scene_dimensions = (
        scene_max
        - scene_min
    )

    report = {
        "source": (
            "Third-party FORSA reference model"
        ),

        "fbx_file": (
            FBX_PATH.name
        ),

        "mesh_count": len(
            mesh_objects
        ),

        "scene_bbox_min": vector_to_list(
            scene_min
        ),

        "scene_bbox_max": vector_to_list(
            scene_max
        ),

        "scene_dimensions": vector_to_list(
            scene_dimensions
        ),

        "objects": objects,
    }

    REPORT_PATH.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with REPORT_PATH.open(
        "w",
        encoding="utf-8",
    ) as file:

        json.dump(
            report,
            file,
            indent=2,
        )

    print()
    print(
        "=== Scene dimensions ==="
    )

    print(
        [
            round(v, 4)
            for v
            in vector_to_list(
                scene_dimensions
            )
        ]
    )

    print()
    print(
        f"Report saved to:\n{REPORT_PATH}"
    )


if __name__ == "__main__":
    main()
    