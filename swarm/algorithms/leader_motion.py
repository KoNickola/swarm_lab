"""Generate unsaturated leader reference velocities from experiment configuration."""

from __future__ import annotations

from math import hypot, isfinite
from typing import Optional, Sequence

from swarm.algorithms.pid import PID
from swarm.domain.config import (
    LeaderTargetVelocity,
    PIDConfig,
    TimePositionReference,
    TimeVelocityReference,
    XYCurveReference,
)
from swarm.domain.models import Vector3, VehicleState


class LeaderReferenceController:
    """Convert one fixed, time-based, or geometric reference into velocity."""

    _CURVE_DERIVATIVE_STEP_M = 1e-4

    def __init__(
        self,
        reference: LeaderTargetVelocity,
        position_pid: PIDConfig,
    ) -> None:
        if position_pid.output_limit is not None:
            raise ValueError(
                "leader position PID output_limit must be None"
            )
        self.reference = reference
        self.position_pid = position_pid
        self._position_controllers = tuple(
            self._build_position_pid() for _ in range(3)
        )
        self._curve_controller = self._build_position_pid()
        self._fixed_velocity: Optional[Vector3] = None
        if not isinstance(
            reference,
            (TimeVelocityReference, TimePositionReference, XYCurveReference),
        ):
            self._fixed_velocity = self._as_vector3(
                reference,
                "fixed leader velocity",
            )

    def _build_position_pid(self) -> PID:
        return PID(
            kp=self.position_pid.kp,
            ki=self.position_pid.ki,
            kd=self.position_pid.kd,
            integral_limit=self.position_pid.integral_limit,
            output_limit=None,
            integral_window_s=self.position_pid.integral_window_s,
        )

    @staticmethod
    def _as_vector3(value: Sequence[float], label: str) -> Vector3:
        try:
            vector = tuple(float(component) for component in value)
        except (TypeError, ValueError):
            raise ValueError("%s must return a numeric vector" % label) from None
        if len(vector) not in (2, 3):
            raise ValueError("%s must contain two or three values" % label)
        if not all(isfinite(component) for component in vector):
            raise ValueError("%s must contain only finite values" % label)
        if len(vector) == 2:
            return vector[0], vector[1], 0.0
        return vector[0], vector[1], vector[2]

    @staticmethod
    def _as_scalar(value: float, label: str) -> float:
        try:
            scalar = float(value)
        except (TypeError, ValueError):
            raise ValueError("%s must return a numeric scalar" % label) from None
        if not isfinite(scalar):
            raise ValueError("%s must return a finite value" % label)
        return scalar

    def _position_velocity(
        self,
        reference: TimePositionReference,
        elapsed_s: float,
        state: Optional[VehicleState],
        dt: float,
    ) -> Vector3:
        if state is None or not state.valid:
            return 0.0, 0.0, 0.0
        value = reference.function(elapsed_s)
        try:
            position = tuple(float(component) for component in value)
        except (TypeError, ValueError):
            raise ValueError(
                "time position function must return a numeric vector"
            ) from None
        if len(position) not in (2, 3):
            raise ValueError(
                "time position function must contain two or three values"
            )
        if not all(isfinite(component) for component in position):
            raise ValueError(
                "time position function must contain only finite values"
            )
        target = (
            position[0],
            position[1],
            position[2] if len(position) == 3 else state.z,
        )
        measured = state.position()
        output = tuple(
            controller.update(target[index], measured[index], dt)
            for index, controller in enumerate(self._position_controllers)
        )
        return output[0], output[1], output[2]

    def _curve_velocity(
        self,
        reference: XYCurveReference,
        state: Optional[VehicleState],
        dt: float,
    ) -> Vector3:
        if state is None or not state.valid:
            return 0.0, 0.0, 0.0

        curve_y = self._as_scalar(
            reference.function(state.x),
            "XY curve function",
        )
        derivative_step = self._CURVE_DERIVATIVE_STEP_M
        y_after = self._as_scalar(
            reference.function(state.x + derivative_step),
            "XY curve function",
        )
        y_before = self._as_scalar(
            reference.function(state.x - derivative_step),
            "XY curve function",
        )
        slope = (y_after - y_before) / (2.0 * derivative_step)
        if not isfinite(slope):
            raise ValueError("XY curve numerical slope must be finite")

        tangent_norm = hypot(1.0, slope)
        tangent_x = 1.0 / tangent_norm
        tangent_y = slope / tangent_norm
        normal_x = -tangent_y
        normal_y = tangent_x

        normal_error = (curve_y - state.y) * normal_y
        correction = self._curve_controller.update(normal_error, 0.0, dt)
        return (
            reference.speed_mps * tangent_x + correction * normal_x,
            reference.speed_mps * tangent_y + correction * normal_y,
            0.0,
        )

    def velocity(
        self,
        elapsed_s: float,
        state: Optional[VehicleState],
        dt: float,
    ) -> Vector3:
        """Return a raw reference velocity without applying a speed limit."""

        if self._fixed_velocity is not None:
            velocity = self._fixed_velocity
        elif isinstance(self.reference, TimeVelocityReference):
            velocity = self._as_vector3(
                self.reference.function(elapsed_s),
                "time velocity function",
            )
        elif isinstance(self.reference, TimePositionReference):
            velocity = self._position_velocity(
                self.reference,
                elapsed_s,
                state,
                dt,
            )
        elif isinstance(self.reference, XYCurveReference):
            velocity = self._curve_velocity(self.reference, state, dt)
        else:
            raise TypeError("unsupported leader reference: %r" % (self.reference,))
        return self._as_vector3(velocity, "leader reference velocity")

    def reset(self) -> None:
        """Clear all position-loop state retained by this reference."""

        for controller in self._position_controllers:
            controller.reset()
        self._curve_controller.reset()
