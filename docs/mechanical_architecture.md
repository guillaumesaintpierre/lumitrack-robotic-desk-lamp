# Mechanical Architecture

## Overview

LumiTrack converts an IKEA FORSÅ desk lamp into a two-degree-of-freedom robotic system.

The original spring-balanced arm mechanism is preserved and remains manually adjustable.

Only two existing rotational joints are actuated:

1. Base yaw
2. Lamp-head pitch

No permanent modification of the original lamp is planned.

---

## Axis 1 — Base Yaw

The complete lamp arm rotates around the original vertical shaft located in the base.

Experimental characterization:

- Rotation effort: easy to moderate
- Approximate useful range with the original adjustment screw lightly tightened: 100 deg
- Maximum observed rotation when loosened significantly: approximately 360 deg
- Initial robotic operating range: -50 deg to +50 deg

The first mechanical concept uses a servo-driven crank-linkage mechanism.

A reversible split clamp will be attached to the rotating lamp shaft.
A radial arm on this clamp will be connected to the servo horn using a linkage.

The servo itself will remain fixed relative to the lamp base.

Advantages:

- Low part count
- Low cost
- Fully reversible
- No belt tensioning required
- Easy to manufacture using 3D printing
- Suitable for the required ~100 deg workspace

---

## Axis 2 — Head Pitch

The original lamp-head pivot will remain intact.

Experimental characterization:

- Rotation effort: moderate
- Approximate mechanical range: -90 deg to +90 deg
- Initial robotic operating range: -60 deg to +60 deg

A servo will be mounted on the upper lamp arm using a removable clamp.

A linkage will connect the servo horn to a removable attachment point near the lamp-head bracket.

---

## Design requirements

The robotic conversion must:

- require no drilling of the IKEA lamp;
- require no welding;
- avoid permanent adhesives;
- preserve manual adjustment of the spring-loaded arm;
- avoid any modification of the mains-voltage electrical circuit;
- allow the complete robotic system to be removed;
- keep the lamp functional as a normal desk lamp.

---

## Mechanical limits

Initial software limits:

Base yaw:

    -50 deg <= theta_base <= +50 deg

Head pitch:

    -60 deg <= theta_head <= +60 deg

These values will be updated after mechanical integration and calibration.

---

## Reference geometry

Preliminary geometry may be estimated using a third-party FORSÅ 3D model.

All dimensions affecting manufactured interfaces will be verified on the physical lamp before final CAD parts are printed.
