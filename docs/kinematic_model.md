# Kinematic Model

## Coordinate System

The origin of the robot coordinate system is located on the base yaw axis at desk level.

- x: forward direction on the desk
- y: lateral direction on the desk
- z: vertical direction

A workspace target is represented by:

P = [x, y, z]

For objects located on the desk surface:

z ≈ 0

## Simplified Geometry

The spring-loaded lamp arms are manually positioned and remain fixed during autonomous operation.

The lamp-head pivot is therefore described using:

- r_h: horizontal distance from the base yaw axis
- z_h: height of the lamp-head pivot above the desk

## Base Yaw

For a target located at (x, y):

theta_base = atan2(y, x)

## Head Pitch

The radial target distance is:

rho = sqrt(x² + y²)

The horizontal distance from the lamp-head pivot to the target is:

d = rho - r_h

The required downward optical pitch angle is:

theta_pitch = atan2(z_h - z, d)

## Notes

The geometric model determines the desired optical orientation.

A separate actuator calibration will later convert these optical angles into physical servo commands.

The parameters r_h and z_h will be determined experimentally from the real lamp configuration.
