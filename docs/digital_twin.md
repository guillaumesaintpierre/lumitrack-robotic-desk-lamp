# Digital Twin

## Objective

The LumiTrack digital twin represents the proposed robotic conversion of an IKEA FORSÅ desk lamp.

The simulated system has two actuated degrees of freedom:

- base yaw
- lamp-head pitch

The spring-balanced lamp arms are treated as manually adjustable but fixed during autonomous operation.

## Geometry source

A third-party FBX model of the IKEA FORSÅ lamp is used as a geometric reference.

The FBX file is not distributed in this repository.

Manufacturer dimensions and the physical lamp are used to assess the plausibility of the reference geometry.

The model is intended for simulation and concept development rather than manufacturing-grade dimensional reconstruction.

## Simulation pipeline

Camera target

→ desk coordinate transformation

→ inverse kinematics

→ target joint angles

→ motion controller

→ digital twin

→ virtual illumination target

## Model parameters

The primary model parameters are:

- head horizontal radius
- head height
- base yaw range
- head pitch range
- actuator speed limits
- actuator acceleration limits

## Physical interpretation

The simulated joints correspond to potential servo-actuated mechanisms.

A belt transmission is considered for base yaw and a linkage mechanism for head pitch.

These mechanical systems are currently design concepts and have not been physically manufactured.
