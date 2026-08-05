import numpy as np

from piper_x_face_follow.kalman import ConstantVelocityKalman3D


def test_kalman_smooths_measurements_and_predicts_through_short_dropout():
    filt = ConstantVelocityKalman3D(process_noise=0.5, measurement_noise=0.01)
    first = filt.update([0.0, 0.0, 0.5], 0.0)
    assert np.allclose(first, [0.0, 0.0, 0.5])
    filt.update([0.1, 0.0, 0.5], 0.1)
    estimate = filt.update([0.2, 0.0, 0.5], 0.2)
    assert 0.05 < estimate[0] < 0.2
    predicted = filt.estimate(0.3)
    assert predicted[0] >= estimate[0]
    assert filt.update_count == 3


def test_kalman_rejects_non_finite_input():
    filt = ConstantVelocityKalman3D()
    try:
        filt.update([float("nan"), 0.0, 0.0], 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("non-finite measurement must be rejected")


def test_kalman_gates_depth_spike_and_forecasts_motion():
    filt = ConstantVelocityKalman3D(
        process_noise=5.0,
        measurement_noise=0.001,
        innovation_gate_m=0.20,
        max_velocity_m_s=1.0,
    )
    filt.update([0.0, 0.0, 0.5], 0.0)
    filt.update([0.05, 0.0, 0.5], 0.1)
    before = filt.estimate(0.1)
    after = filt.update([0.05, 0.0, 2.5], 0.2)
    assert after[2] < 0.8
    assert filt.gated_update_count == 1
    forecast = filt.forecast(0.2, 0.08)
    assert forecast[0] >= after[0]
    assert np.linalg.norm(filt.state[3:]) <= 1.0 + 1e-9
