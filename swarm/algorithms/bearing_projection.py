"""Projection-matrix bearing formation algorithm."""

from __future__ import annotations

from math import atan2, hypot, isfinite, radians, sqrt
from typing import List, Mapping, Optional, Tuple

from swarm.algorithms.bearing import FormationTopology
from swarm.algorithms.leader_motion import LeaderReferenceController
from swarm.domain.config import (
    BEARING_MODE_ONLY,
    FormationControlConfig,
    LeaderTargetVelocity,
    PIDConfig,
    SUPPORTED_BEARING_MODES,
)
from swarm.domain.models import (
    BearingSample,
    MocapSnapshot,
    PlannerResult,
    Vector3,
    VehicleState,
    VelocityCommand,
)


class BearingProjectionFormationController:
    """Produce bearing corrections with mode-selected projection vectors."""

    _NORM_EPSILON = 1e-9

    def __init__(
        self,
        topology: FormationTopology,
        leader_target_velocities: Mapping[str, LeaderTargetVelocity],
        control: FormationControlConfig,
        command_speed_limit_mps: float,
        leader_position_pid: Optional[PIDConfig] = None,
    ) -> None:
        if command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        if control.bearing_mode not in SUPPORTED_BEARING_MODES:
            raise ValueError("unsupported bearing_mode: %r" % control.bearing_mode)
        if not all(
            isfinite(value)
            for value in (
                control.bearing_deadband_enter_deg,
                control.bearing_deadband_exit_deg,
            )
        ):
            raise ValueError("bearing deadband thresholds must be finite")
        if not (
            0.0
            <= control.bearing_deadband_enter_deg
            < control.bearing_deadband_exit_deg
            <= 180.0
        ):
            raise ValueError(
                "bearing deadband thresholds must satisfy "
                "0 <= enter < exit <= 180 degrees"
            )

        self.topology = topology
        self.leader_target_velocities = dict(leader_target_velocities)
        self.control = control
        self.command_speed_limit_mps = command_speed_limit_mps
        self._deadband_enter_rad = radians(control.bearing_deadband_enter_deg)
        self._deadband_exit_rad = radians(control.bearing_deadband_exit_deg)
        self._deadband_settled = {
            vehicle_id: False for vehicle_id in topology.vehicle_ids
        }
        self._elapsed_s = 0.0

        position_pid = leader_position_pid or PIDConfig(
            0.8,
            0.0,
            0.0,
            1.0,
            None,
        )
        self._leader_references = {
            vehicle_id: LeaderReferenceController(
                self.leader_target_velocities[vehicle_id],
                position_pid,
            )
            for index, vehicle_id in enumerate(self.topology.vehicle_ids)
            if self.topology.leader_mask[index]
        }

    @classmethod
    def _normalize(cls, vector: Vector3) -> Tuple[Vector3, bool]:
        norm = sqrt(sum(component * component for component in vector))
        if norm < cls._NORM_EPSILON:
            return (0.0, 0.0, 0.0), False
        return tuple(component / norm for component in vector), True

    def _control_vector(self, vector: Vector3) -> Tuple[Vector3, bool]:
        """Select a unit bearing or an unscaled relative vector."""

        if self.control.bearing_mode == BEARING_MODE_ONLY:
            return self._normalize(vector)
        return (
            (float(vector[0]), float(vector[1]), float(vector[2])),
            True,
        )

    @staticmethod
    def _relative_vector(source: VehicleState, target: VehicleState) -> Vector3:
        return target.x - source.x, target.y - source.y, target.z - source.z

    @staticmethod
    def _bearing_angle_error(desired: Vector3, actual: Vector3) -> float:
        desired_norm = sqrt(sum(component * component for component in desired))
        actual_norm = sqrt(sum(component * component for component in actual))
        if desired_norm < 1e-9 or actual_norm < 1e-9:
            return radians(180.0)
        desired_unit = tuple(component / desired_norm for component in desired)
        actual_unit = tuple(component / actual_norm for component in actual)
        dot = max(
            -1.0,
            min(
                1.0,
                sum(
                    desired_component * actual_component
                    for desired_component, actual_component in zip(
                        desired_unit,
                        actual_unit,
                    )
                ),
            ),
        )
        cross_x = desired_unit[1] * actual_unit[2] - desired_unit[2] * actual_unit[1]
        cross_y = desired_unit[2] * actual_unit[0] - desired_unit[0] * actual_unit[2]
        cross_z = desired_unit[0] * actual_unit[1] - desired_unit[1] * actual_unit[0]
        cross_norm = sqrt(
            cross_x * cross_x + cross_y * cross_y + cross_z * cross_z
        )
        return atan2(cross_norm, dot)

    @staticmethod
    def _legacy_error(desired: Vector3, actual: Vector3) -> Vector3:
        """Return the existing public error-vector representation."""

        return (
            actual[0] - desired[0],
            actual[1] - desired[1],
            actual[2] - desired[2],
        )

    def _edge_velocity(
        self,
        bearing_error: Vector3,
        actual_unit: Vector3,
    ) -> Vector3:
        """Apply the projection matrix and gain to one bearing error."""

        dot = sum(
            actual_component * error_component
            for actual_component, error_component in zip(
                actual_unit,
                bearing_error,
            )
        )
        return (
            self.control.projection_k_p
            * (bearing_error[0] - actual_unit[0] * dot),
            self.control.projection_k_p
            * (bearing_error[1] - actual_unit[1] * dot),
            self.control.projection_k_p
            * (bearing_error[2] - actual_unit[2] * dot),
        )

    def _vehicle_control(
        self,
        snapshot: MocapSnapshot,
        index: int,
        bearing_samples: List[BearingSample],
    ) -> Tuple[Vector3, Vector3, Optional[float]]:
        """Return mean bearing error, correction velocity, and angle error."""

        source_id = self.topology.vehicle_ids[index]
        source = snapshot.states.get(source_id)
        if source is None or not source.valid:
            zero = (0.0, 0.0, 0.0)
            return zero, zero, None

        legacy_total = [0.0, 0.0, 0.0]
        velocity_total = [0.0, 0.0, 0.0]
        error_edges = 0
        control_edges = 0
        max_angle_error = 0.0

        for target_index, target_id in enumerate(self.topology.vehicle_ids):
            if not self.topology.adjacency_matrix[index][target_index]:
                continue
            target = snapshot.states.get(target_id)
            if target is None or not target.valid:
                continue

            relative = self._relative_vector(source, target)
            actual, actual_valid = self._control_vector(relative)
            desired, desired_valid = self._control_vector(
                self.topology.bearing_matrix[index][target_index]
            )
            # Projection geometry always uses the measured unit direction.
            actual_unit, actual_direction_valid = self._normalize(relative)
            bearing_samples.append(
                BearingSample(source_id, target_id, desired, actual)
            )
            if not actual_valid or not desired_valid:
                continue

            legacy_error = self._legacy_error(desired, actual)
            for component in range(3):
                legacy_total[component] += legacy_error[component]
            error_edges += 1
            max_angle_error = max(
                max_angle_error,
                self._bearing_angle_error(desired, actual),
            )
            if actual_direction_valid:
                edge_velocity = self._edge_velocity(legacy_error, actual_unit)
                for component in range(3):
                    velocity_total[component] += edge_velocity[component]
                control_edges += 1

        if error_edges == 0:
            zero = (0.0, 0.0, 0.0)
            return zero, zero, None

        error_scale = 1.0 / error_edges
        velocity_scale = 1.0 / control_edges if control_edges else 0.0
        return (
            tuple(component * error_scale for component in legacy_total),
            tuple(component * velocity_scale for component in velocity_total),
            max_angle_error,
        )

    def _limit_correction_components(self, correction: Vector3) -> Vector3:
        """Apply the configured component limit to a correction velocity."""

        limit = self.control.output_limit
        return tuple(max(-limit, min(limit, component)) for component in correction)

    def _bearing_correction(
        self,
        vehicle_id: str,
        bearing_velocity: Vector3,
        max_angle_error: Optional[float],
    ) -> Vector3:
        """Apply angular hysteresis before the projection control law."""

        if not self.control.bearing_deadband_enabled or max_angle_error is None:
            return self._limit_correction_components(bearing_velocity)

        settled = self._deadband_settled[vehicle_id]
        if settled:
            if max_angle_error >= self._deadband_exit_rad:
                settled = False
        elif max_angle_error <= self._deadband_enter_rad:
            settled = True
        self._deadband_settled[vehicle_id] = settled

        if settled:
            return 0.0, 0.0, 0.0
        return self._limit_correction_components(bearing_velocity)

    def _leader_command(
        self,
        vehicle_id: str,
        state: Optional[VehicleState],
        bearing_velocity: Vector3,
        max_angle_error: Optional[float],
        dt: float,
    ) -> Vector3:
        preset = self._leader_references[vehicle_id].velocity(
            self._elapsed_s,
            state,
            dt,
        )
        if self.control.leader_bearing_enabled:
            bearing = self._bearing_correction(
                vehicle_id,
                bearing_velocity,
                max_angle_error,
            )
        else:
            bearing = (0.0, 0.0, 0.0)

        return self._limit_speed_3d(
            preset[0] + bearing[0],
            preset[1] + bearing[1],
            preset[2] + bearing[2],
        )

    def _follower_command(
        self,
        vehicle_id: str,
        bearing_velocity: Vector3,
        max_angle_error: Optional[float],
    ) -> Vector3:
        correction = self._bearing_correction(
            vehicle_id,
            bearing_velocity,
            max_angle_error,
        )
        return self._limit_speed_3d(*correction)

    def _limit_speed_3d(self, vx: float, vy: float, vz: float) -> Vector3:
        if not all(isfinite(component) for component in (vx, vy, vz)):
            raise ValueError("velocity command must contain only finite values")
        speed = hypot(vx, vy, vz)
        if speed <= self.command_speed_limit_mps or speed < self._NORM_EPSILON:
            return vx, vy, vz
        scale = self.command_speed_limit_mps / speed
        return vx * scale, vy * scale, vz * scale

    def step(self, snapshot: MocapSnapshot, dt: float) -> PlannerResult:
        result = PlannerResult(timestamp=snapshot.timestamp)
        valid_count = 0
        error_sum = 0.0
        max_error = 0.0

        for index, vehicle_id in enumerate(self.topology.vehicle_ids):
            state = snapshot.states.get(vehicle_id)
            valid = state is not None and state.valid
            error, bearing_velocity, max_angle_error = self._vehicle_control(
                snapshot,
                index,
                result.bearing_samples,
            )

            if self.topology.leader_mask[index]:
                vx, vy, vz = self._leader_command(
                    vehicle_id,
                    state,
                    bearing_velocity,
                    max_angle_error,
                    dt,
                )
            else:
                vx, vy, vz = self._follower_command(
                    vehicle_id,
                    bearing_velocity,
                    max_angle_error,
                )

            result.commands[vehicle_id] = VelocityCommand(
                vehicle_id=vehicle_id,
                vx=vx,
                vy=vy,
                vz=vz,
                error_vector=error,
                valid=valid,
                is_leader=self.topology.leader_mask[index],
            )
            if valid:
                valid_count += 1
                error_magnitude = sqrt(sum(component * component for component in error))
                error_sum += error_magnitude
                max_error = max(max_error, error_magnitude)

        result.ready = valid_count == len(self.topology.vehicle_ids)
        result.mean_error = error_sum / valid_count if valid_count else 0.0
        result.max_error = max_error
        self._elapsed_s += dt
        return result

    def reset(self) -> None:
        self._elapsed_s = 0.0
        for reference in self._leader_references.values():
            reference.reset()
        for vehicle_id in self._deadband_settled:
            self._deadband_settled[vehicle_id] = False
