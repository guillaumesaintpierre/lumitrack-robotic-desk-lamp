import numpy as np

from software.calibration.desk_calibration import (
    compute_homography,
    image_to_robot,
    image_to_world,
    world_to_robot,
)


def test_square_homography():

    image_points = np.array(
        [
            [0, 0],
            [100, 0],
            [100, 100],
            [0, 100],
        ],
        dtype=np.float32,
    )

    world_points = np.array(
        [
            [0.0, -0.3],
            [0.0, 0.3],
            [0.4, 0.3],
            [0.4, -0.3],
        ],
        dtype=np.float32,
    )

    H = compute_homography(
        image_points,
        world_points,
    )

    centre = image_to_world(
        (50, 50),
        H,
    )

    assert np.allclose(
        centre,
        [0.2, 0.0],
        atol=1e-6,
    )


def test_world_to_robot():

    world_point = np.array(
        [0.50, 0.10]
    )

    base_world = np.array(
        [0.10, -0.05]
    )

    robot_point = world_to_robot(
        world_point,
        base_world,
    )

    assert np.allclose(
        robot_point,
        [0.40, 0.15],
    )


def test_image_to_robot():

    image_points = np.array(
        [
            [0, 0],
            [100, 0],
            [100, 100],
            [0, 100],
        ],
        dtype=np.float32,
    )

    world_points = np.array(
        [
            [0.0, -0.3],
            [0.0, 0.3],
            [0.4, 0.3],
            [0.4, -0.3],
        ],
        dtype=np.float32,
    )

    H = compute_homography(
        image_points,
        world_points,
    )

    base_world = np.array(
        [0.10, 0.0]
    )

    result = image_to_robot(
        (50, 50),
        H,
        base_world,
    )

    assert np.allclose(
        result,
        [0.10, 0.0],
        atol=1e-6,
    )

    