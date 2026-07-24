"""Scalar PID and vector PI controllers used by the existing algorithms."""

from __future__ import annotations

from typing import Optional, Sequence, Tuple, Union

VectorLike = Union[Sequence[float], Tuple[float, ...]]


class PID:
    """Stateful scalar PID controller."""

    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        integral_limit: Optional[float] = None,
        output_limit: Optional[float] = None,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    @staticmethod
    def _clamp(value: float, lower: Optional[float], upper: Optional[float]) -> float:
        if lower is not None:
            value = max(lower, value)
        if upper is not None:
            value = min(upper, value)
        return value

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def update(self, target: float, measured: float, dt: float) -> float:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        error = target - measured
        derivative = 0.0 if not self.initialized else (error - self.prev_error) / dt
        self.initialized = True
        self.prev_error = error
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = self._clamp(self.integral, -self.integral_limit, self.integral_limit)
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.output_limit is not None:
            output = self._clamp(output, -self.output_limit, self.output_limit)
        return output


class VectorPIController:
    """PI controller for two- or three-dimensional error vectors."""

    def __init__(self, kp: float, ki: float, integral_limit: float, output_limit: float) -> None:
        self.kp = kp
        self.ki = ki
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral: Tuple[float, ...] = ()

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _to_tuple(vector: VectorLike) -> Tuple[float, ...]:
        return tuple(float(value) for value in vector)

    def reset(self) -> None:
        self.integral = ()

    def update(self, error: VectorLike, dt: float) -> Tuple[float, ...]:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        error_vec = self._to_tuple(error)
        if not error_vec:
            raise ValueError("error vector must not be empty")
        if not self.integral:
            self.integral = tuple(0.0 for _ in error_vec)
        if len(self.integral) != len(error_vec):
            raise ValueError("error vector dimension changed during control")

        new_integral = []
        output = []
        for index, component in enumerate(error_vec):
            integral = self.integral[index] + component * dt
            integral = self._clamp(integral, -self.integral_limit, self.integral_limit)
            new_integral.append(integral)
            value = self.kp * component + self.ki * integral
            value = self._clamp(value, -self.output_limit, self.output_limit)
            output.append(value)
        self.integral = tuple(new_integral)
        return tuple(output)
