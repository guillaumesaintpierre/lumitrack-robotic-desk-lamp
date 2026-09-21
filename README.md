# LumiTrack — Vision-Guided Robotic Desk Lamp

LumiTrack is a two-degree-of-freedom robotic desk lamp based on an IKEA FORSÅ work lamp.

The goal is to transform an existing desk lamp into a useful robotic system capable of automatically illuminating the user's active workspace using computer vision.

## Main objectives

- Motorize the lamp base yaw axis
- Motorize the lamp head pitch axis
- Preserve the original spring-arm mechanism
- Avoid permanent modification of the IKEA lamp
- Detect the user's hand or active workspace using computer vision
- Automatically orient the light toward the target
- Implement embedded motor control using an ESP32
- Provide physical operating modes through push buttons

## Planned operating modes

- **AUTO** — track the user's active hand/workspace
- **READING** — detect and illuminate a notebook or document
- **HOME** — return the lamp to its default position
- **FREEZE** — hold the current lamp orientation

## System architecture

Camera → Python perception → target estimation → serial communication → ESP32 → motor control → IKEA FORSÅ

## Technologies

- Python
- OpenCV
- MediaPipe
- C++
- ESP32
- CAD / additive manufacturing
- Computer vision
- Mechatronics
- Robotics and control

## Project status

🚧 Work in progress