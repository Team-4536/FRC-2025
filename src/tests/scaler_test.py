from utils import Scalar


def test_scaler_default():
    s = Scalar()

    assert s(0) == 0, "0 should always return zero"
    assert s.deadzone == 0.1, "default deadzone"
    assert s.exponent == 1, "default exponent"

    assert s(s.deadzone) == 0, "expecting 0 at deadzone threshold"
    assert s(-s.deadzone) == 0, "expecting 0 at deadzone threshold"

    assert s(s.deadzone + 0.01) > 0, "expecting > 0 beyond deadzone threshold"
    assert (
        s(-(s.deadzone + 0.01)) < 0
    ), "expecting < 0 beyond negative deadzone threshold"

    assert abs(s(0.55) - 0.5) < 0.0001
    assert abs(s(-0.55) + 0.5) < 0.0001

    assert s(1) == 1
    assert s(-1) == -1
