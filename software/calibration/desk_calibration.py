import json
from pathlib import Path

import cv2
import numpy as np


def compute_homography(
    image_points: np.ndarray,
    world_points: np.ndarray,
) -> np.ndarray:
    """
    Compute planar homography mapping image pixels to
    physical desk coordinates.

    Parameters
    ----------
    image_points:
        Array of shape (4, 2), containing pixel coordinates.

    world_points:
        Array of shape (4, 2), containing physical coordinates
        in metres.

    Returns
    -------
    H:
        3x3 homography matrix such that

            [X, Y, 1]^T ~ H [u, v, 1]^T
    """

    image_points = np.asarray(
        image_points,
        dtype=np.float32,
    )

    world_points = np.asarray(
        world_points,
        dtype=np.float32,
    )

    if image_points.shape != (4, 2):
        raise ValueError(
            "image_points must have shape (4, 2)"
        )

    if world_points.shape != (4, 2):
        raise ValueError(
            "world_points must have shape (4, 2)"
        )

    H = cv2.getPerspectiveTransform(
        image_points,
        world_points,
    )

    return H


def image_to_world(
    point_px,
    homography: np.ndarray,
) -> np.ndarray:
    """
    Convert one image pixel coordinate into physical desk
    coordinates in metres.
    """

    point = np.array(
        [
            [
                [
                    float(point_px[0]),
                    float(point_px[1]),
                ]
            ]
        ],
        dtype=np.float32,
    )

    transformed = cv2.perspectiveTransform(
        point,
        homography,
    )

    return transformed[0, 0].astype(float)


def world_to_robot(
    world_point,
    base_world,
) -> np.ndarray:
    """
    Express a world point relative to the lamp base.
    """

    world_point = np.asarray(
        world_point,
        dtype=float,
    )

    base_world = np.asarray(
        base_world,
        dtype=float,
    )

    return world_point - base_world


def image_to_robot(
    point_px,
    homography: np.ndarray,
    base_world,
) -> np.ndarray:
    """
    Full transformation:

        image pixel -> desk coordinate -> robot coordinate
    """

    world = image_to_world(
        point_px,
        homography,
    )

    return world_to_robot(
        world,
        base_world,
    )


def save_calibration(
    path,
    homography: np.ndarray,
    base_world,
    image_points,
    workspace_width_m: float,
    workspace_depth_m: float,
):
    """
    Save calibration parameters to JSON.
    """

    path = Path(path)

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    data = {
        "homography": (
            np.asarray(
                homography,
                dtype=float,
            ).tolist()
        ),
        "base_world": (
            np.asarray(
                base_world,
                dtype=float,
            ).tolist()
        ),
        "image_points": (
            np.asarray(
                image_points,
                dtype=float,
            ).tolist()
        ),
        "workspace_width_m": float(
            workspace_width_m
        ),
        "workspace_depth_m": float(
            workspace_depth_m
        ),
    }

    with path.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            data,
            file,
            indent=2,
        )


def load_calibration(path):
    """
    Load calibration data from JSON.
    """

    path = Path(path)

    with path.open(
        "r",
        encoding="utf-8",
    ) as file:
        data = json.load(file)

    data["homography"] = np.asarray(
        data["homography"],
        dtype=float,
    )

    data["base_world"] = np.asarray(
        data["base_world"],
        dtype=float,
    )

    data["image_points"] = np.asarray(
        data["image_points"],
        dtype=float,
    )

    return data

