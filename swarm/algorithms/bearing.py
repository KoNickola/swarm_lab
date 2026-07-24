"""Topology-driven bearing-only formation algorithm."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import List, Mapping, Optional, Sequence, Tuple

from swarm.algorithms.pid import VectorPIController
from swarm.domain.config import FormationControlConfig
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
    """Produce translational commands using the existing bearing-only law."""

    def __init__(
        self,
        topology: FormationTopology,
        leader_target_velocities: Mapping[str, Vector3],
        control: FormationControlConfig,
        command_speed_limit_mps: float,
    ) -> None:
        if command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        self.topology = topology
        self.leader_target_velocities = dict(leader_target_velocities)
        self.control = control
        self.command_speed_limit_mps = command_speed_limit_mps
        self._controllers = {
            vehicle_id: VectorPIController(
                kp=control.k_p,
                ki=control.k_i,
                integral_limit=control.integral_limit,
                output_limit=control.output_limit,
            )
            for vehicle_id in topology.vehicle_ids
        }

    @staticmethod
    def _normalize(x: float, y: float, z: float = 0.0) -> Vector3:
        norm = sqrt(x * x + y * y + z * z)
        if norm < 1e-9:
            return 0.0, 0.0, 0.0
        return x / norm, y / norm, z / norm

    @staticmethod
    def _bearing_error(desired: Vector3, actual: Vector3) -> Vector3:
        return actual[0] - desired[0], actual[1] - desired[1], actual[2] - desired[2]

    @staticmethod
    def _relative_vector(source: VehicleState, target: VehicleState) -> Vector3:
        return target.x - source.x, target.y - source.y, target.z - source.z

    def _vehicle_error(
        self,
        snapshot: MocapSnapshot,
        index: int,
        bearing_samples: List[BearingSample],
    ) -> Vector3:
        source_id = self.topology.vehicle_ids[index]
        source = snapshot.states.get(source_id)
        if source is None or not source.valid:
            return 0.0, 0.0, 0.0

        total = [0.0, 0.0, 0.0]
        active_edges = 0
        for target_index, target_id in enumerate(self.topology.vehicle_ids):
            if not self.topology.adjacency_matrix[index][target_index]:
                continue
            target = snapshot.states.get(target_id)
            if target is None or not target.valid:
                continue
            relative = self._relative_vector(source, target)
            actual = self._normalize(*relative)
            desired = self._normalize(*self.topology.bearing_matrix[index][target_index])
            error = self._bearing_error(desired, actual)
            total[0] += error[0]
            total[1] += error[1]
            total[2] += error[2]
            active_edges += 1
            bearing_samples.append(BearingSample(source_id, target_id, desired, actual))
        if active_edges == 0:
            return 0.0, 0.0, 0.0
        return total[0] / active_edges, total[1] / active_edges, total[2] / active_edges

    def _leader_command(self, vehicle_id: str) -> Vector3:
        vx, vy, vz = self.leader_target_velocities[vehicle_id]
        return self._limit_speed_3d(vx, vy, vz)

    def _follower_command(self, vehicle_id: str, error: Vector3, dt: float) -> Vector3:
        correction = self._controllers[vehicle_id].update(error, dt)
        return self._limit_speed_3d(*correction)

    def _limit_speed_3d(self, vx: float, vy: float, vz: float) -> Vector3:
        speed = sqrt(vx * vx + vy * vy + vz * vz)
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
            error = self._vehicle_error(snapshot, index, result.bearing_samples)

            if self.topology.leader_mask[index]:
                vx, vy, vz = self._leader_command(vehicle_id)
            else:
                vx, vy, vz = self._follower_command(vehicle_id, error, dt)

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
        return result

    def reset(self) -> None:
        for controller in self._controllers.values():
            controller.reset()


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
