from piper_x_face_follow.depth_tracker import ConfirmedAlphaBeta1D


def test_depth_tracker_rejects_single_background_jump():
    tracker = ConfirmedAlphaBeta1D(jump_gate_m=0.12, jump_confirmations=2)
    assert tracker.update(0.50, 0.0)
    assert not tracker.update(1.40, 0.1)
    assert tracker.forecast(0.1) < 0.6
    assert tracker.rejected_count == 1


def test_depth_tracker_accepts_confirmed_range_change_and_smooth_motion():
    tracker = ConfirmedAlphaBeta1D(jump_gate_m=0.12, jump_confirmations=2)
    tracker.update(0.30, 0.0)
    assert not tracker.update(0.60, 0.1)
    assert tracker.update(0.61, 0.2)
    first = tracker.forecast(0.2)
    assert 0.30 < first < 0.50
    tracker.update(0.50, 0.3)
    tracker.update(0.45, 0.4)
    assert tracker.forecast(0.4) > 0.30
