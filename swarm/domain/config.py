"""Typed configuration grouped by algorithm, experiment, and runtime concerns."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Callable, Mapping, Optional, Sequence, Tuple, Union

from swarm.domain.models import Vector3


@dataclass(frozen=True)
class PIDConfig:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: Optional[float]
    integral_window_s: float = 1.0


VectorReferenceFunction = Callable[[float], Sequence[float]]
XYCurveFunction = Callable[[float], float]


@dataclass(frozen=True)
class TimeVelocityReference:
    """World-frame velocity generated from elapsed experiment time."""

    function: VectorReferenceFunction


@dataclass(frozen=True)
class TimePositionReference:
    """Absolute world-frame position generated from elapsed experiment time."""

    function: VectorReferenceFunction


@dataclass(frozen=True)
class XYCurveReference:
    """Static world-frame curve ``y=f(x)`` and signed tangential speed."""

    function: XYCurveFunction
    speed_mps: float


LeaderTargetVelocity = Union[
    Vector3,
    TimeVelocityReference,
    TimePositionReference,
    XYCurveReference,
]

CONTROL_SPACE_PLANAR_XY = "planar_xy"
CONTROL_SPACE_SPATIAL_XYZ = "spatial_xyz"
SUPPORTED_CONTROL_SPACES = (
    CONTROL_SPACE_PLANAR_XY,
    CONTROL_SPACE_SPATIAL_XYZ,
)


@dataclass(frozen=True)
class FormationControlConfig:
    k_p: float = 1.2
    k_i: float = 0.5
    integral_limit: float = 0.4
    integral_window_s: float = 2.0
    output_limit: float = 0.4
    leader_bearing_enabled: bool = False
    bearing_deadband_enabled: bool = False
    bearing_deadband_enter_deg: float = 1.5
    bearing_deadband_exit_deg: float = 3



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


# Global default for experiments that do not override controller_type.
# GROUND_VEHICLE_CONTROLLER_TYPE = "differential"
GROUND_VEHICLE_CONTROLLER_TYPE = "projected"
# GROUND_VEHICLE_CONTROLLER_TYPE = "mecanum"


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
    controller_type: str = GROUND_VEHICLE_CONTROLLER_TYPE
    mecanum_speed_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(1.5, 0.0, 0.0, 1.0, 0.3)
    )
    projected_speed_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(1.2, 0.5, 0.0, 1.0, 0.3)
    )
    projected_lookahead_m: float = 0.03
    projected_max_yaw_rate_rps: float = 11.32


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
    control_space: str
    vehicles: Tuple[VehicleConfig, ...]
    topology: TopologyConfig
    leader_target_velocities: Mapping[str, LeaderTargetVelocity]
    formation_control: FormationControlConfig = field(default_factory=FormationControlConfig)
    ground_vehicle_control: GroundVehicleControlConfig = field(default_factory=GroundVehicleControlConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    obstacle_avoidance: ObstacleAvoidanceConfig = field(default_factory=ObstacleAvoidanceConfig)
    static_obstacles: Tuple[StaticObstacleConfig, ...] = ()
    leader_position_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(0.8, 0.0, 0.0, 1.0, None)
    )

    @property
    def vehicle_ids(self) -> Tuple[str, ...]:
        return tuple(vehicle.vehicle_id for vehicle in self.vehicles)

    def validate(self) -> None:
        """Reject inconsistent configuration before external resources start."""

        if not isinstance(self.algorithm, str) or not self.algorithm.strip():
            raise ValueError("algorithm must be a non-empty string")
        if self.control_space not in SUPPORTED_CONTROL_SPACES:
            raise ValueError(
                "unsupported control_space %r; expected one of: %s"
                % (self.control_space, ", ".join(SUPPORTED_CONTROL_SPACES))
            )
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
        leader_ids = []
        for index, vehicle in enumerate(self.vehicles):
            is_leader = bool(self.topology.leader_mask[index])
            if is_leader and vehicle.vehicle_id not in self.leader_target_velocities:
                raise ValueError("missing target velocity for leader: %s" % vehicle.vehicle_id)
            if is_leader:
                leader_ids.append(vehicle.vehicle_id)
        for vehicle_id in leader_ids:
            target = self.leader_target_velocities[vehicle_id]
            if isinstance(
                target,
                (TimeVelocityReference, TimePositionReference, XYCurveReference),
            ):
                if not callable(target.function):
                    raise ValueError(
                        "leader reference function must be callable: %s" % vehicle_id
                    )
                if isinstance(target, XYCurveReference) and not isfinite(target.speed_mps):
                    raise ValueError(
                        "XY curve speed must be finite: %s" % vehicle_id
                    )
                continue
            try:
                fixed_velocity = tuple(float(value) for value in target)
            except (TypeError, ValueError):
                raise ValueError(
                    "leader target must be a 3D velocity or reference: %s"
                    % vehicle_id
                ) from None
            if len(fixed_velocity) != 3 or not all(
                isfinite(value) for value in fixed_velocity
            ):
                raise ValueError(
                    "leader fixed velocity must contain three finite values: %s"
                    % vehicle_id
                )
        position_pid = self.leader_position_pid
        position_pid_values = (
            position_pid.kp,
            position_pid.ki,
            position_pid.kd,
            position_pid.integral_limit,
            position_pid.integral_window_s,
        )
        if not all(isfinite(value) for value in position_pid_values):
            raise ValueError("leader position PID values must be finite")
        if min(position_pid.kp, position_pid.ki, position_pid.kd) < 0.0:
            raise ValueError("leader position PID gains must be non-negative")
        if position_pid.integral_limit < 0.0:
            raise ValueError(
                "leader position PID integral_limit must be non-negative"
            )
        if position_pid.integral_window_s <= 0.0:
            raise ValueError(
                "leader position PID integral_window_s must be positive"
            )
        if position_pid.output_limit is not None:
            raise ValueError(
                "leader position PID output_limit must be None; "
                "the combined leader velocity uses the shared speed limit"
            )
        if self.runtime.control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if self.runtime.command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        if self.ground_vehicle_control.controller_type not in (
            "differential",
            "mecanum",
            "projected",
        ):
            raise ValueError(
                "ground vehicle controller_type must be "
                "'differential', 'mecanum', or 'projected'"
            )
        if self.ground_vehicle_control.max_speed_mps <= 0.0:
            raise ValueError("ground vehicle max_speed_mps must be positive")
        if (
            not isfinite(self.ground_vehicle_control.projected_lookahead_m)
            or self.ground_vehicle_control.projected_lookahead_m <= 0.0
        ):
            raise ValueError("projected_lookahead_m must be positive and finite")
        if (
            not isfinite(self.ground_vehicle_control.projected_max_yaw_rate_rps)
            or self.ground_vehicle_control.projected_max_yaw_rate_rps <= 0.0
        ):
            raise ValueError(
                "projected_max_yaw_rate_rps must be positive and finite"
            )
        if self.runtime.command_speed_limit_mps > self.ground_vehicle_control.max_speed_mps:
            raise ValueError(
                "command_speed_limit_mps must not exceed ground vehicle max_speed_mps"
            )
        formation = self.formation_control
        if (
            not isfinite(formation.integral_window_s)
            or formation.integral_window_s <= 0.0
        ):
            raise ValueError("formation integral_window_s must be positive and finite")
        deadband_thresholds = (
            formation.bearing_deadband_enter_deg,
            formation.bearing_deadband_exit_deg,
        )
        if not all(isfinite(value) for value in deadband_thresholds):
            raise ValueError("bearing deadband thresholds must be finite")
        if formation.bearing_deadband_enter_deg < 0.0:
            raise ValueError("bearing_deadband_enter_deg must be non-negative")
        if (
            formation.bearing_deadband_enter_deg
            >= formation.bearing_deadband_exit_deg
        ):
            raise ValueError(
                "bearing_deadband_enter_deg must be less than "
                "bearing_deadband_exit_deg"
            )
        if formation.bearing_deadband_exit_deg > 180.0:
            raise ValueError("bearing_deadband_exit_deg must not exceed 180 degrees")
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
