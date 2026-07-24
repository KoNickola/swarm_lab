"""Typed configuration grouped by algorithm, experiment, and runtime concerns."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence, Tuple

from swarm.domain.models import Vector3


@dataclass(frozen=True)
class PIDConfig:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float


@dataclass(frozen=True)
class FormationControlConfig:
    k_p: float = 1.0
    k_i: float = 0.0
    integral_limit: float = 1.0
    output_limit: float = 0.4


@dataclass(frozen=True)
class StaticObstacleConfig:
    """Circular static obstacle expressed in the world frame."""

    obstacle_id: str
    x: float
    y: float
    radius_m: float


@dataclass(frozen=True)
class ObstacleAvoidanceConfig:
    """Parameters for the planar artificial-potential-field filter."""

    enabled: bool = False
    vehicle_radius_m: float = 0.03
    influence_distance_m: float = 0.1
    repulsive_gain: float = 0.0005
    min_distance_epsilon_m: float = 0.01
    avoid_other_vehicles: bool = True


@dataclass(frozen=True)
class GroundVehicleControlConfig:
    track_width_m: float = 0.053
    max_speed_mps: float = 0.3
    wheel_command_min: float = -100.0
    wheel_command_max: float = 100.0
    heading_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(3.2, 0.0, 0.15, 1.0, 11.32)
    )
    speed_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(1.2, 0.0, 0.5, 1.0, 0.3)
    )


@dataclass(frozen=True)
class VehicleConfig:
    vehicle_id: str
    role: str
    platform: str
    address: str


@dataclass(frozen=True)
class TopologyConfig:
    adjacency_matrix: Sequence[Sequence[int]]
    bearing_matrix: Sequence[Sequence[Sequence[float]]]
    leader_mask: Sequence[bool]


@dataclass(frozen=True)
class TelemetryConfig:
    console_enabled: bool = True
    console_hz: float = 10.0
    live_plot_enabled: bool = True
    live_plot_hz: float = 10.0
    live_plot_queue_size: int = 64
    live_plot_max_points: int = 2000
    record_enabled: bool = True
    record_hz: float = 5.0
    record_root: str = "runs"
    record_flush_interval_s: float = 1.0


@dataclass(frozen=True)
class RuntimeConfig:
    control_hz: float = 50.0
    mocap_prefix: str = "/vrpn_client_node"
    stop_on_loss_of_mocap: bool = True
    udp_port: int = 12345
    udp_timeout_s: float = 0.2
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    command_speed_limit_mps: float = 0.30


@dataclass(frozen=True)
class ExperimentConfig:
    name: str
    algorithm: str
    vehicles: Tuple[VehicleConfig, ...]
    topology: TopologyConfig
    leader_target_velocities: Mapping[str, Vector3]
    formation_control: FormationControlConfig = field(default_factory=FormationControlConfig)
    ground_vehicle_control: GroundVehicleControlConfig = field(default_factory=GroundVehicleControlConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    obstacle_avoidance: ObstacleAvoidanceConfig = field(default_factory=ObstacleAvoidanceConfig)
    static_obstacles: Tuple[StaticObstacleConfig, ...] = ()

    @property
    def vehicle_ids(self) -> Tuple[str, ...]:
        return tuple(vehicle.vehicle_id for vehicle in self.vehicles)

    def validate(self) -> None:
        """Reject inconsistent configuration before external resources start."""

        if not isinstance(self.algorithm, str) or not self.algorithm.strip():
            raise ValueError("algorithm must be a non-empty string")
        vehicle_ids = self.vehicle_ids
        if not vehicle_ids:
            raise ValueError("vehicles must not be empty")
        if len(set(vehicle_ids)) != len(vehicle_ids):
            raise ValueError("vehicle IDs must be unique")
        n = len(vehicle_ids)
        if len(self.topology.adjacency_matrix) != n or len(self.topology.bearing_matrix) != n:
            raise ValueError("topology dimensions must match vehicle count")
        if len(self.topology.leader_mask) != n:
            raise ValueError("leader mask must match vehicle count")
        for index, vehicle in enumerate(self.vehicles):
            is_leader = bool(self.topology.leader_mask[index])
            if is_leader and vehicle.vehicle_id not in self.leader_target_velocities:
                raise ValueError("missing target velocity for leader: %s" % vehicle.vehicle_id)
        if self.runtime.control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if self.runtime.command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        if self.ground_vehicle_control.max_speed_mps <= 0.0:
            raise ValueError("ground vehicle max_speed_mps must be positive")
        if self.runtime.command_speed_limit_mps > self.ground_vehicle_control.max_speed_mps:
            raise ValueError(
                "command_speed_limit_mps must not exceed ground vehicle max_speed_mps"
            )
        avoidance = self.obstacle_avoidance
        if avoidance.vehicle_radius_m < 0.0:
            raise ValueError("vehicle_radius_m must be non-negative")
        if avoidance.influence_distance_m <= 0.0:
            raise ValueError("influence_distance_m must be positive")
        if avoidance.repulsive_gain < 0.0:
            raise ValueError("repulsive_gain must be non-negative")
        if avoidance.min_distance_epsilon_m <= 0.0:
            raise ValueError("min_distance_epsilon_m must be positive")
        if avoidance.min_distance_epsilon_m >= avoidance.influence_distance_m:
            raise ValueError("min_distance_epsilon_m must be less than influence_distance_m")
        obstacle_ids = tuple(obstacle.obstacle_id for obstacle in self.static_obstacles)
        if len(set(obstacle_ids)) != len(obstacle_ids):
            raise ValueError("static obstacle IDs must be unique")
        for obstacle in self.static_obstacles:
            if not obstacle.obstacle_id:
                raise ValueError("static obstacle ID must not be empty")
            if obstacle.radius_m < 0.0:
                raise ValueError("static obstacle radius_m must be non-negative")
