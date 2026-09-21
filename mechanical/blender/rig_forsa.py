import bpy
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

OUTPUT_BLEND = (
    PROJECT_ROOT
    / "mechanical"
    / "blender"
    / "lumitrack_forsa_rigged.blend"
)


# ============================================================
# Configuration
# ============================================================

SCALE_FACTOR = 0.81

BASE_OBJECT = "ChamferCyl001"

YAW_REFERENCE = "Cylinder004"

HEAD_PITCH_REFERENCE = "Object005"

HEAD_OBJECT = "Sphere002"


# ============================================================
# Scene utilities
# ============================================================

def clear_scene():

    bpy.ops.object.select_all(
        action="SELECT"
    )

    bpy.ops.object.delete(
        use_global=False
    )


def create_empty(
    name,
    location,
):

    bpy.ops.object.empty_add(
        type="PLAIN_AXES",
        location=location,
    )

    empty = bpy.context.active_object

    empty.name = name

    empty.empty_display_size = 0.05

    return empty


def parent_keep_transform(
    child,
    parent,
):

    matrix_world = (
        child.matrix_world.copy()
    )

    child.parent = parent

    child.matrix_world = matrix_world


# ============================================================
# Main
# ============================================================

def main():

    print()
    print(
        "=== LumiTrack FORSA rig creation ==="
    )
    print()

    if not FBX_PATH.exists():

        raise FileNotFoundError(
            f"FBX file not found: {FBX_PATH}"
        )

    clear_scene()

    # --------------------------------------------------------
    # Import
    # --------------------------------------------------------

    bpy.ops.import_scene.fbx(
        filepath=str(FBX_PATH)
    )

    bpy.context.view_layer.update()

    print(
        f"Imported: {FBX_PATH.name}"
    )

    # --------------------------------------------------------
    # Uniform scaling
    # --------------------------------------------------------

    for obj in bpy.context.scene.objects:

        obj.scale *= SCALE_FACTOR

    bpy.context.view_layer.update()

    # --------------------------------------------------------
    # Required objects
    # --------------------------------------------------------

    yaw_reference = bpy.data.objects.get(
        YAW_REFERENCE
    )

    pitch_reference = bpy.data.objects.get(
        HEAD_PITCH_REFERENCE
    )

    base_object = bpy.data.objects.get(
        BASE_OBJECT
    )

    head_object = bpy.data.objects.get(
        HEAD_OBJECT
    )

    if yaw_reference is None:
        raise RuntimeError(
            f"Missing object: {YAW_REFERENCE}"
        )

    if pitch_reference is None:
        raise RuntimeError(
            f"Missing object: {HEAD_PITCH_REFERENCE}"
        )

    if base_object is None:
        raise RuntimeError(
            f"Missing object: {BASE_OBJECT}"
        )

    if head_object is None:
        raise RuntimeError(
            f"Missing object: {HEAD_OBJECT}"
        )

    # --------------------------------------------------------
    # Pivot locations
    # --------------------------------------------------------

    yaw_location = (
        yaw_reference.matrix_world.translation.copy()
    )

    pitch_location = (
        pitch_reference.matrix_world.translation.copy()
    )

    print()
    print(
        "Yaw pivot location:",
        tuple(
            round(v, 4)
            for v in yaw_location
        ),
    )

    print(
        "Head pitch pivot location:",
        tuple(
            round(v, 4)
            for v in pitch_location
        ),
    )

    # --------------------------------------------------------
    # Create pivots
    # --------------------------------------------------------

    yaw_pivot = create_empty(
        "LumiTrack_YawPivot",
        yaw_location,
    )

    pitch_pivot = create_empty(
        "LumiTrack_HeadPitchPivot",
        pitch_location,
    )

    # Head pitch pivot belongs to yaw structure
    parent_keep_transform(
        pitch_pivot,
        yaw_pivot,
    )

    # --------------------------------------------------------
    # Parent upper lamp to yaw pivot
    # --------------------------------------------------------

    #
    # Keep the physical base static.
    #
    # Everything sufficiently above the yaw pivot is treated
    # as part of the rotating upper assembly.
    #

    for obj in list(
        bpy.context.scene.objects
    ):

        if obj in (
            yaw_pivot,
            pitch_pivot,
            base_object,
        ):
            continue

        if obj.type != "MESH":
            continue

        z_center = (
            obj.matrix_world.translation.z
        )

        if (
            z_center
            >= yaw_location.z - 0.01
        ):

            parent_keep_transform(
                obj,
                yaw_pivot,
            )

    # --------------------------------------------------------
    # Parent head assembly to pitch pivot
    # --------------------------------------------------------

    #
    # Objects close to and beyond the head pivot in the
    # forward direction are assigned to the head assembly.
    #
    # This is a first geometric grouping that can later
    # be refined visually if needed.
    #

    for obj in list(
        bpy.context.scene.objects
    ):

        if obj.type != "MESH":
            continue

        center = (
            obj.matrix_world.translation
        )

        if (
            center.y
            >= pitch_location.y - 0.005
        ):

            parent_keep_transform(
                obj,
                pitch_pivot,
            )

    # Ensure pitch pivot itself remains child of yaw pivot
    parent_keep_transform(
        pitch_pivot,
        yaw_pivot,
    )

    # --------------------------------------------------------
    # Save
    # --------------------------------------------------------

    OUTPUT_BLEND.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    bpy.ops.wm.save_as_mainfile(
        filepath=str(
            OUTPUT_BLEND
        )
    )

    print()
    print(
        "=== Rig created ==="
    )

    print(
        f"Saved to:\n{OUTPUT_BLEND}"
    )

    print()
    print(
        "Important:"
    )

    print(
        "Yaw pivot rotation axis = Blender Z"
    )

    print(
        "Head pitch axis must now be verified visually."
    )


if __name__ == "__main__":
    main()

    