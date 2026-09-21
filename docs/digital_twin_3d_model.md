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
