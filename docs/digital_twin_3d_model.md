# LumiTrack 3D Digital Twin Model

## Reference geometry

LumiTrack uses a third-party 3D model of the IKEA FORSÅ lamp
as a visual and geometric reference.

The model is not an official IKEA manufacturing CAD model and
is therefore not assumed to reproduce the physical lamp with
manufacturing-level accuracy.

Critical dimensions and joint ranges are verified independently
against the physical lamp or published dimensions where possible.

## Robotic architecture

The LumiTrack concept uses two actuated degrees of freedom:

1. Base yaw
2. Lamp-head pitch

The original spring-arm joints remain manually adjustable and are
treated as fixed during autonomous operation.

## Digital twin

The imported model is used to:

- improve geometric realism;
- estimate the nominal lamp-head position;
- define the base-yaw pivot;
- define the head-pitch pivot;
- create a visually realistic digital twin.

The current project implements and validates the robotic system
in simulation. No physical robotic prototype has been manufactured.

## Articulated joints

The imported FORSÅ reference model was articulated using two virtual joints.

### Base yaw

- Blender pivot: `LumiTrack_YawPivot`
- Reference mesh: `Cylinder004`
- Rotation axis: Blender Z
- Robotic coordinate: base yaw

### Head pitch

- Blender pivot: `LumiTrack_HeadPitchPivot`
- Reference mesh: `Object005`
- Rotation axis: Blender X
- Robotic coordinate: lamp-head pitch

## Reference geometry

After scaling the third-party FORSÅ model by 0.81:

- nominal head horizontal radius: approximately 0.080 m
- nominal head pivot height above desk: approximately 0.257 m

These values correspond to the fixed arm configuration represented by the
reference FBX model and are not manufacturing dimensions.