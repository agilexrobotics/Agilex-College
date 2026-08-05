"""Constant-velocity 3D Kalman filter for intermittent depth landmarks."""

import numpy as np


class ConstantVelocityKalman3D:
    def __init__(
        self,
        process_noise: float = 1.0,
        measurement_noise: float = 0.01,
        innovation_gate_m: float = float("inf"),
        max_velocity_m_s: float = float("inf"),
    ):
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.innovation_gate_m = float(innovation_gate_m)
        self.max_velocity_m_s = float(max_velocity_m_s)
        self.state = np.zeros(6)
        self.covariance = np.eye(6)
        self.last_time = None
        self.last_measurement_time = None
        self.update_count = 0
        self.gated_update_count = 0
        self.last_innovation_norm = 0.0

    @property
    def initialized(self) -> bool:
        return self.last_time is not None

    def _predict(self, now: float) -> None:
        if self.last_time is None:
            self.last_time = float(now)
            return
        dt = max(0.0, min(float(now) - self.last_time, 0.25))
        if dt <= 0.0:
            return
        transition = np.eye(6)
        transition[:3, 3:] = np.eye(3) * dt
        accel = np.vstack((np.eye(3) * (0.5 * dt * dt), np.eye(3) * dt))
        process = (accel @ accel.T) * self.process_noise
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process
        self.last_time = float(now)

    def update(self, measurement, now: float) -> np.ndarray:
        value = np.asarray(measurement, dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError("Kalman measurement must be three finite coordinates")
        if not self.initialized:
            self.state[:3] = value
            self.state[3:] = 0.0
            self.covariance = np.diag([0.02] * 3 + [0.25] * 3)
            self.last_time = float(now)
        else:
            self._predict(now)
            observation = np.hstack((np.eye(3), np.zeros((3, 3))))
            innovation = value - observation @ self.state
            self.last_innovation_norm = float(np.linalg.norm(innovation))
            if (
                np.isfinite(self.innovation_gate_m)
                and self.innovation_gate_m > 0.0
                and self.last_innovation_norm > self.innovation_gate_m
            ):
                # A depth edge can jump from the hand to the background by
                # metres.  Limit one update instead of accepting that spike or
                # freezing completely; repeated real motion still catches up.
                innovation *= self.innovation_gate_m / self.last_innovation_norm
                self.gated_update_count += 1
            noise = np.eye(3) * self.measurement_noise
            innovation_covariance = (
                observation @ self.covariance @ observation.T + noise
            )
            gain = self.covariance @ observation.T @ np.linalg.inv(
                innovation_covariance
            )
            self.state = self.state + gain @ innovation
            self.covariance = (
                np.eye(6) - gain @ observation
            ) @ self.covariance
            speed = float(np.linalg.norm(self.state[3:]))
            if (
                np.isfinite(self.max_velocity_m_s)
                and self.max_velocity_m_s > 0.0
                and speed > self.max_velocity_m_s
            ):
                self.state[3:] *= self.max_velocity_m_s / speed
        self.last_measurement_time = float(now)
        self.update_count += 1
        return self.state[:3].copy()

    def estimate(self, now: float) -> np.ndarray:
        if not self.initialized:
            raise RuntimeError("Kalman filter has not received a measurement")
        self._predict(now)
        return self.state[:3].copy()

    def forecast(self, now: float, lead_sec: float = 0.0) -> np.ndarray:
        """Estimate now and extrapolate without advancing filter time past now."""
        estimate = self.estimate(now)
        lead = max(0.0, min(float(lead_sec), 0.25))
        return estimate + self.state[3:] * lead
