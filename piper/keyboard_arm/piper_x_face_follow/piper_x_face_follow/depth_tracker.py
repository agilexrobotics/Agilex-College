"""Low-latency robust scalar range tracker for hand depth."""

import math


class ConfirmedAlphaBeta1D:
    """Alpha-beta tracking with confirmation for large depth discontinuities."""

    def __init__(
        self,
        alpha: float = 0.65,
        beta: float = 0.12,
        jump_gate_m: float = 0.12,
        jump_match_m: float = 0.06,
        jump_confirmations: int = 2,
        max_velocity_m_s: float = 0.8,
    ):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.jump_gate_m = float(jump_gate_m)
        self.jump_match_m = float(jump_match_m)
        self.jump_confirmations = max(1, int(jump_confirmations))
        self.max_velocity_m_s = float(max_velocity_m_s)
        self.value = None
        self.velocity = 0.0
        self.last_time = None
        self.last_measurement_time = None
        self.update_count = 0
        self.rejected_count = 0
        self._candidate = None
        self._candidate_count = 0

    @property
    def initialized(self) -> bool:
        return self.value is not None

    def update(self, measurement: float, now: float) -> bool:
        measurement = float(measurement)
        now = float(now)
        if not math.isfinite(measurement) or measurement <= 0.0:
            raise ValueError("depth measurement must be positive and finite")
        if not self.initialized:
            self.value = measurement
            self.last_time = now
            self.last_measurement_time = now
            self.update_count = 1
            return True

        dt = max(1.0e-3, min(now - self.last_time, 0.30))
        predicted = self.value + self.velocity * dt
        innovation = measurement - predicted
        if abs(innovation) > self.jump_gate_m:
            if (
                self._candidate is not None
                and abs(measurement - self._candidate) <= self.jump_match_m
            ):
                self._candidate_count += 1
                self._candidate = 0.5 * (self._candidate + measurement)
            else:
                self._candidate = measurement
                self._candidate_count = 1
            if self._candidate_count < self.jump_confirmations:
                self.rejected_count += 1
                return False
            measurement = self._candidate
            innovation = measurement - predicted
        self._candidate = None
        self._candidate_count = 0

        # Limit one accepted correction; sustained real movement continues to
        # converge, while a hand/background edge cannot reverse the robot.
        correction_limit = max(self.jump_gate_m, 0.5 * self.max_velocity_m_s * dt)
        innovation = max(-correction_limit, min(correction_limit, innovation))
        self.value = predicted + self.alpha * innovation
        velocity_measurement = innovation / dt
        self.velocity = (1.0 - self.beta) * self.velocity + self.beta * velocity_measurement
        self.velocity = max(
            -self.max_velocity_m_s,
            min(self.max_velocity_m_s, self.velocity),
        )
        self.last_time = now
        self.last_measurement_time = now
        self.update_count += 1
        return True

    def forecast(self, now: float, lead_sec: float = 0.0) -> float:
        if not self.initialized:
            raise RuntimeError("depth tracker has not received a measurement")
        horizon = max(0.0, min(float(now) - self.last_time + lead_sec, 0.25))
        return float(self.value + self.velocity * horizon)
