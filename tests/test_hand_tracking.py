from software.vision.hand_tracking import normalized_to_pixel


def test_normalized_center():

    x, y = normalized_to_pixel(
        x_norm=0.5,
        y_norm=0.5,
        width=1000,
        height=800,
    )

    assert x == 500
    assert y == 400


def test_normalized_origin():

    x, y = normalized_to_pixel(
        x_norm=0.0,
        y_norm=0.0,
        width=640,
        height=480,
    )

    assert x == 0
    assert y == 0


def test_normalized_clipping():

    x, y = normalized_to_pixel(
        x_norm=2.0,
        y_norm=-1.0,
        width=640,
        height=480,
    )

    assert x == 639
    assert y == 0

    