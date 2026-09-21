# Actuator Selection

## Candidate actuator

The initial prototype uses two MG996R metal-gear servos.

Manufacturer specifications:

- operating voltage: 4.8–6.6 V
- stall torque at 4.8 V: 9.4 kgf·cm
- stall torque at 6.0 V: 11 kgf·cm
- mass: approximately 55 g

For preliminary design calculations:

11 kgf·cm ≈ 1.08 N·m

The mechanism is not designed to continuously operate near stall torque.

A conservative preliminary usable torque fraction of 30% is used for sizing.

---

## Base yaw transmission

Measured useful lamp-base rotation:

approximately 100 deg

Nominal servo range:

approximately 180 deg

Required transmission ratio:

R = 180 / 100 = 1.8

Preliminary concept:

- 20-tooth servo pulley
- approximately 36-tooth lamp-shaft pulley
- GT2 timing belt

The exact ratio will be selected after measuring the real servo travel.

Expected advantages:

- increased output torque
- reduced backlash compared with a long linkage
- simple mechanical limits
- servo can remain fixed relative to the lamp base

---

## Head pitch transmission

Desired initial robotic range:

approximately 120 deg

Required nominal ratio:

R = 180 / 120 = 1.5

A crank-linkage mechanism will connect the servo to the existing lamp-head joint.

Preliminary lever dimensions:

- servo horn radius: approximately 20 mm
- lamp-head lever radius: approximately 30 mm

The final geometry will be determined during CAD design.

---

## Validation required

Before finalizing the mechanical design:

1. Measure actual servo angular travel.
2. Verify available torque experimentally.
3. Measure critical FORSÅ dimensions.
4. Check mechanical interference throughout the workspace.
5. Define hard and software angle limits.
