"""Topology-driven vector PI bearing formation algorithm."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot, isfinite, radians, sqrt
from typing import List, Mapping, Optional, Sequence, Tuple

from swarm.algorithms.leader_motion import LeaderReferenceController
from swarm.algorithms.pid import VectorPIController
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


@dataclass(frozen=True)
class FormationTopology:
    """Directed matrix topology and desired world-frame bearings."""

    vehicle_ids: List[str]
    adjacency_matrix: List[List[int]]
    bearing_matrix: List[List[Vector3]]
    leader_mask: List[bool]
    follower_mask: List[bool]

    def __post_init__(self) -> None:
        count = len(self.vehicle_ids)
        if count == 0:
            raise ValueError("vehicle_ids must not be empty")
        if len(self.adjacency_matrix) != count or len(self.bearing_matrix) != count:
            raise ValueError("topology matrices must match vehicle_ids length")
        if len(self.leader_mask) != count or len(self.follower_mask) != count:
            raise ValueError("role masks must match vehicle_ids length")
        if any(len(row) != count for row in self.adjacency_matrix):
            raise ValueError("adjacency_matrix must be square")
        if any(len(row) != count for row in self.bearing_matrix):
            raise ValueError("bearing_matrix must be square")


class BearingOnlyFormationController:
    """Produce translational commands using the vector PI bearing law."""

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
        self._controllers = {
            vehicle_id: VectorPIController(
                kp=control.k_p,
                ki=control.k_i,
                integral_limit=control.integral_limit,
                output_limit=control.output_limit,
                integral_window_s=control.integral_window_s,
            )
            for vehicle_id in topology.vehicle_ids
        }

    @staticmethod
    def _normalize(x: float, y: float, z: float = 0.0) -> Vector3:
        norm = sqrt(x * x + y * y + z * z)
        if norm < 1e-9:
            return 0.0, 0.0, 0.0
        return x / norm, y / norm, z / norm

    def _control_vector(self, vector: Vector3) -> Vector3:
        """Select a unit bearing or an unscaled relative vector."""

        if self.control.bearing_mode == BEARING_MODE_ONLY:
            return self._normalize(*vector)
        return float(vector[0]), float(vector[1]), float(vector[2])

    @staticmethod
    def _bearing_error(desired: Vector3, actual: Vector3) -> Vector3:
        return actual[0] - desired[0], actual[1] - desired[1], actual[2] - desired[2]

    @staticmethod
    def _bearing_angle_error(desired: Vector3, actual: Vector3) -> float:
        """Return the unsigned angle between one desired and actual bearing."""

        desired_norm = sqrt(sum(component * component for component in desired))
        actual_norm = sqrt(sum(component * component for component in actual))
        if desired_norm < 1e-9 or actual_norm < 1e-9:
            return radians(180.0)
        desired_unit = tuple(component / desired_norm for component in desired)
        actual_unit = tuple(component / actual_norm for component in actual)
        dot = sum(
            desired_component * actual_component
            for desired_component, actual_component in zip(desired_unit, actual_unit)
        )
        cross_x = desired_unit[1] * actual_unit[2] - desired_unit[2] * actual_unit[1]
        cross_y = desired_unit[2] * actual_unit[0] - desired_unit[0] * actual_unit[2]
        cross_z = desired_unit[0] * actual_unit[1] - desired_unit[1] * actual_unit[0]
        cross_norm = sqrt(
            cross_x * cross_x + cross_y * cross_y + cross_z * cross_z
        )
        return atan2(cross_norm, max(-1.0, min(1.0, dot)))

    @staticmethod
    def _relative_vector(source: VehicleState, target: VehicleState) -> Vector3:
        return target.x - source.x, target.y - source.y, target.z - source.z

    def _vehicle_error(
        self,
        snapshot: MocapSnapshot,
        index: int,
        bearing_samples: List[BearingSample],
    ) -> Tuple[Vector3, Optional[float]]:
        source_id = self.topology.vehicle_ids[index]
        source = snapshot.states.get(source_id)
        if source is None or not source.valid:
            return (0.0, 0.0, 0.0), None

        total = [0.0, 0.0, 0.0]
        active_edges = 0
        max_angle_error = 0.0
        for target_index, target_id in enumerate(self.topology.vehicle_ids):
            if not self.topology.adjacency_matrix[index][target_index]:
                continue
            target = snapshot.states.get(target_id)
            if target is None or not target.valid:
                continue
            relative = self._relative_vector(source, target)
            actual = self._control_vector(relative)
            desired = self._control_vector(
                self.topology.bearing_matrix[index][target_index]
            )
            error = self._bearing_error(desired, actual)
            total[0] += error[0]
            total[1] += error[1]
            total[2] += error[2]
            active_edges += 1
            max_angle_error = max(
                max_angle_error,
                self._bearing_angle_error(desired, actual),
            )
            bearing_samples.append(BearingSample(source_id, target_id, desired, actual))
        if active_edges == 0:
            return (0.0, 0.0, 0.0), None
        return (
            total[0] / active_edges,
            total[1] / active_edges,
            total[2] / active_edges,
        ), max_angle_error

    def _bearing_correction(
        self,
        vehicle_id: str,
        error: Vector3,
        max_angle_error: Optional[float],
        dt: float,
    ) -> Vector3:
        """Apply angular hysteresis before the existing bearing PI controller."""

        controller = self._controllers[vehicle_id]
        if not self.control.bearing_deadband_enabled or max_angle_error is None:
            return controller.update(error, dt)

        settled = self._deadband_settled[vehicle_id]
        if settled:
            if max_angle_error >= self._deadband_exit_rad:
                settled = False
        elif max_angle_error <= self._deadband_enter_rad:
            settled = True
            controller.reset()
        self._deadband_settled[vehicle_id] = settled

        if settled:
            return 0.0, 0.0, 0.0
        return controller.update(error, dt)

    def _leader_command(
        self,
        vehicle_id: str,
        state: Optional[VehicleState],
        error: Vector3,
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
                error,
                max_angle_error,
                dt,
            )
        else:
            bearing = (0.0, 0.0, 0.0)

        # Both components share one vector limit so their relative direction
        # remains present when the physical command speed saturates.
        return self._limit_speed_3d(
            preset[0] + bearing[0],
            preset[1] + bearing[1],
            preset[2] + bearing[2],
        )

    def _follower_command(
        self,
        vehicle_id: str,
        error: Vector3,
        max_angle_error: Optional[float],
        dt: float,
    ) -> Vector3:
        correction = self._bearing_correction(
            vehicle_id,
            error,
            max_angle_error,
            dt,
        )
        return self._limit_speed_3d(*correction)

    def _limit_speed_3d(self, vx: float, vy: float, vz: float) -> Vector3:
        if not all(isfinite(component) for component in (vx, vy, vz)):
            raise ValueError("velocity command must contain only finite values")
        speed = hypot(vx, vy, vz)
        if speed <= self.command_speed_limit_mps or speed < 1e-9:
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
            error, max_angle_error = self._vehicle_error(
                snapshot,
                index,
                result.bearing_samples,
            )

            if self.topology.leader_mask[index]:
                vx, vy, vz = self._leader_command(
                    vehicle_id,
                    state,
                    error,
                    max_angle_error,
                    dt,
                )
            else:
                vx, vy, vz = self._follower_command(
                    vehicle_id,
                    error,
                    max_angle_error,
                    dt,
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
                error_magnitude = sqrt(error[0] * error[0] + error[1] * error[1] + error[2] * error[2])
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
        for controller in self._controllers.values():
            controller.reset()
        for vehicle_id in self._deadband_settled:
            self._deadband_settled[vehicle_id] = False


def build_topology_from_matrices(
    vehicle_ids: Sequence[str],
    adjacency_matrix: Sequence[Sequence[int]],
    bearing_matrix: Sequence[Sequence[Sequence[float]]],
    leader_mask: Optional[Sequence[bool]] = None,
) -> FormationTopology:
    count = len(vehicle_ids)
    if count == 0:
        raise ValueError("vehicle_ids must not be empty")
    if len(adjacency_matrix) != count or len(bearing_matrix) != count:
        raise ValueError("matrix dimensions must match vehicle_ids")

    adjacency = [[int(adjacency_matrix[i][j]) for j in range(count)] for i in range(count)]
    bearings: List[List[Vector3]] = []
    for i in range(count):
        row: List[Vector3] = []
        if len(bearing_matrix[i]) != count:
            raise ValueError("bearing_matrix must be square")
        for j in range(count):
            vector = bearing_matrix[i][j]
            if len(vector) < 2:
                raise ValueError("each desired bearing must contain at least x and y")
            row.append(
                (
                    float(vector[0]),
                    float(vector[1]),
                    float(vector[2]) if len(vector) > 2 else 0.0,
                )
            )
        bearings.append(row)

    if leader_mask is None:
        leaders = [False] * count
    else:
        if len(leader_mask) != count:
            raise ValueError("leader_mask must match vehicle_ids")
        leaders = [bool(value) for value in leader_mask]
    return FormationTopology(
        vehicle_ids=list(vehicle_ids),
        adjacency_matrix=adjacency,
        bearing_matrix=bearings,
        leader_mask=leaders,
        follower_mask=[not value for value in leaders],
    )
