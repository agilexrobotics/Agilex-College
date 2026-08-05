"""Pure safety and tracking policy helpers."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def deadband(value: float, width: float) -> float:
    if abs(value) <= width:
        return 0.0
    return math.copysign(abs(value) - width, value)


def ordered_joints(names: Sequence[str], positions: Sequence[float]) -> Optional[Tuple[float, ...]]:
    by_name = dict(zip(names, positions))
    if not all(name in by_name for name in JOINT_NAMES):
        return None
    values = tuple(float(by_name[name]) for name in JOINT_NAMES)
    return values if all(math.isfinite(value) for value in values) else None


def slew(current: Iterable[float], target: Iterable[float], max_step: float) -> Tuple[float, ...]:
    result = []
    for now, goal in zip(current, target):
        result.append(now + clamp(goal - now, -max_step, max_step))
    return tuple(result)


def enforce_j6_lock(command: Sequence[float], lock: float) -> Tuple[float, ...]:
    if len(command) != 6 or not all(math.isfinite(float(value)) for value in command):
        raise ValueError("joint command must contain six finite values")
    guarded = [float(value) for value in command]
    guarded[5] = float(lock)
    if guarded[5] != lock:
        raise RuntimeError("J6 guard failed")
    return tuple(guarded)


@dataclass(frozen=True)
class StableFace:
    u: float
    v: float
    span_u: float
    span_v: float


class FaceStabilityFilter:
    def __init__(self, sample_count: int, maximum_span_px: float):
        self._samples = deque(maxlen=max(2, int(sample_count)))
        self._maximum_span = float(maximum_span_px)

    def clear(self) -> None:
        self._samples.clear()

    def add(self, u: float, v: float) -> Optional[StableFace]:
        if not math.isfinite(u) or not math.isfinite(v):
            self.clear()
            return None
        self._samples.append((float(u), float(v)))
        if len(self._samples) < self._samples.maxlen:
            return None
        us = [item[0] for item in self._samples]
        vs = [item[1] for item in self._samples]
        span_u = max(us) - min(us)
        span_v = max(vs) - min(vs)
        if span_u > self._maximum_span or span_v > self._maximum_span:
            return None
        return StableFace(sum(us) / len(us), sum(vs) / len(vs), span_u, span_v)
