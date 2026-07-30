"""Ground-vehicle chassis controllers and selection factory."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, pi, sqrt
from math import cos, sin
from typing import Dict, Tuple, Union

from swarm.algorithms.pid import PID
from swarm.domain.config import GroundVehicleControlConfig


@dataclass
class ControllerState:
    """Observable state retained by one ground-vehicle controller."""

    target_vx: float = 0.0
    target_vy: float = 0.0
    target_vz: float = 0.0
    measured_vx: float = 0.0
    measured_vy: float = 0.0
    measured_vz: float = 0.0
    target_heading: float = 0.0
    target_speed: float = 0.0
    measured_speed: float = 0.0
    heading_error: float = 0.0
    speed_cmd_mps: float = 0.0
    angle_cmd: float = 0.0
    left_speed_mps: float = 0.0
    right_speed_mps: float = 0.0


class VehicleController:
    """Convert a three-dimensional target velocity into four wheel commands."""

    def __init__(self, config: GroundVehicleControlConfig) -> None:
        self.config = config
        self.track_width = config.track_width_m
        self.heading_pid = PID(**vars(config.heading_pid))
        self.speed_pid = PID(**vars(config.speed_pid))
        self.state = ControllerState()

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def _wheel_command_from_speed(self, speed_mps: float) -> float:
        ratio = speed_mps / max(self.config.max_speed_mps, 1e-6)
        command = ratio * self.config.wheel_command_max
        return max(self.config.wheel_command_min, min(self.config.wheel_command_max, command))

    def reset(self) -> None:
        self.heading_pid.reset()
        self.speed_pid.reset()
        self.state = ControllerState()

    @staticmethod
    def _target_from_velocity(target_vx: float, target_vy: float) -> Tuple[float, float]:
        planar_speed = sqrt(target_vx * target_vx + target_vy * target_vy)
        if planar_speed < 1e-9:
            return 0.0, 0.0
        return planar_speed, atan2(target_vy, target_vx)

    def step(
        self,
        target_vx: float,
        target_vy: float,
        target_vz: float,
        yaw: float,
        measured_vx: float,
        measured_vy: float,
        measured_vz: float,
        dt: float,
    ) -> Dict[str, float]:
        """Run one unchanged ground-vehicle control update."""

        _ = target_vz, measured_vz
        self.state.target_vx = target_vx
        self.state.target_vy = target_vy
        self.state.target_vz = target_vz
        self.state.measured_vx = measured_vx
        self.state.measured_vy = measured_vy
        self.state.measured_vz = measured_vz

        target_speed, target_heading = self._target_from_velocity(target_vx, target_vy)
        stationary_target = target_speed == 0.0
        if stationary_target:
            target_heading = yaw
        self.state.target_heading = target_heading
        self.state.target_speed = target_speed

        measured_speed = sqrt(measured_vx * measured_vx + measured_vy * measured_vy)
        self.state.measured_speed = measured_speed
        if stationary_target:
            self.speed_pid.reset()
            speed_cmd_mps = 0.0
        else:
            speed_cmd_mps = self.speed_pid.update(target_speed, measured_speed, dt)

        heading_error = 0.0 if stationary_target else self._wrap_to_pi(yaw - target_heading)
        self.state.heading_error = heading_error
        if stationary_target:
            self.heading_pid.reset()
            angle_cmd = 0.0
        else:
            angle_cmd = self.heading_pid.update(0.0, heading_error, dt)
        self.state.angle_cmd = angle_cmd

        if abs(heading_error) >= (pi / 2):
            speed_cmd_limit = 0.0
        else:
            speed_cmd_limit = self.config.max_speed_mps * (1.0 - abs(heading_error) / (pi / 2))
        speed_cmd_mps = max(0, min(speed_cmd_mps, speed_cmd_limit))
        self.state.speed_cmd_mps = speed_cmd_mps

        half_track = self.track_width / 2.0
        left_speed_mps = speed_cmd_mps - angle_cmd * half_track
        right_speed_mps = speed_cmd_mps + angle_cmd * half_track
        self.state.left_speed_mps = left_speed_mps
        self.state.right_speed_mps = right_speed_mps

        left_command = self._wheel_command_from_speed(left_speed_mps)
        right_command = self._wheel_command_from_speed(right_speed_mps)
        return {
            "front_left": left_command,
            "front_right": right_command,
            "rear_left": left_command,
            "rear_right": right_command,
            "speed_mps": speed_cmd_mps,
            "steer_rad": angle_cmd,
            "target_heading": target_heading,
            "left_speed_mps": left_speed_mps,
            "right_speed_mps": right_speed_mps,
            "target_speed": target_speed,
            "measured_speed": measured_speed,
            "heading_error": heading_error,
        }

    @staticmethod
    def stop_command() -> Dict[str, float]:
        return {
            "front_left": 0.0,
            "front_right": 0.0,
            "rear_left": 0.0,
            "rear_right": 0.0,
            "speed_mps": 0.0,
            "steer_rad": 0.0,
            "target_heading": 0.0,
            "left_speed_mps": 0.0,
            "right_speed_mps": 0.0,
        }


class ProjectedVehicleController:
    """Map a world-frame velocity continuously into differential wheel speeds."""

    def __init__(self, config: GroundVehicleControlConfig) -> None:
        self.config = config
        self.track_width = config.track_width_m
        self.lookahead = config.projected_lookahead_m
        self.longitudinal_pid = PID(**vars(config.projected_speed_pid))
        self.state = ControllerState()

    @staticmethod
    def _world_to_body(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
        """Rotate a planar world-frame velocity into the chassis frame."""

        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        return (
            cos_yaw * vx + sin_yaw * vy,
            -sin_yaw * vx + cos_yaw * vy,
        )

    def _wheel_command_from_speed(self, speed_mps: float) -> float:
        ratio = speed_mps / max(self.config.max_speed_mps, 1e-6)
        if speed_mps >= 0.0:
            command = ratio * self.config.wheel_command_max
        else:
            command = -ratio * self.config.wheel_command_min
        return max(
            self.config.wheel_command_min,
            min(self.config.wheel_command_max, command),
        )

    def reset(self) -> None:
        self.longitudinal_pid.reset()
        self.state = ControllerState()

    def step(
        self,
        target_vx: float,
        target_vy: float,
        target_vz: float,
        yaw: float,
        measured_vx: float,
        measured_vy: float,
        measured_vz: float,
        dt: float,
    ) -> Dict[str, float]:
        """Run one continuous body-projection control update."""

        _ = target_vz, measured_vz
        self.state.target_vx = target_vx
        self.state.target_vy = target_vy
        self.state.target_vz = target_vz
        self.state.measured_vx = measured_vx
        self.state.measured_vy = measured_vy
        self.state.measured_vz = measured_vz

        target_body_vx, target_body_vy = self._world_to_body(
            target_vx,
            target_vy,
            yaw,
        )
        measured_body_vx, _ = self._world_to_body(
            measured_vx,
            measured_vy,
            yaw,
        )
        planar_target_speed = sqrt(
            target_vx * target_vx + target_vy * target_vy
        )
        stationary_target = planar_target_speed < 1e-9
        target_forward = max(0.0, target_body_vx)
        heading_error = (
            0.0
            if stationary_target
            else atan2(target_body_vy, target_body_vx)
        )
        target_heading = (
            yaw
            if stationary_target
            else atan2(target_vy, target_vx)
        )

        self.state.target_heading = target_heading
        self.state.target_speed = target_forward
        self.state.measured_speed = measured_body_vx
        self.state.heading_error = heading_error

        if stationary_target:
            self.longitudinal_pid.reset()
            speed_cmd_mps = 0.0
            angle_cmd = 0.0
        else:
            if target_forward <= 1e-9:
                self.longitudinal_pid.reset()
                speed_cmd_mps = 0.0
            else:
                speed_cmd_mps = self.longitudinal_pid.update(
                    target_forward,
                    measured_body_vx,
                    dt,
                )
                speed_cmd_mps = max(0.0, speed_cmd_mps)
            angle_cmd = planar_target_speed * heading_error / self.lookahead
            max_yaw_rate = self.config.projected_max_yaw_rate_rps
            angle_cmd = max(-max_yaw_rate, min(max_yaw_rate, angle_cmd))

        half_track = self.track_width / 2.0
        left_speed_mps = speed_cmd_mps - angle_cmd * half_track
        right_speed_mps = speed_cmd_mps + angle_cmd * half_track
        max_wheel_speed = max(abs(left_speed_mps), abs(right_speed_mps))
        if max_wheel_speed > self.config.max_speed_mps:
            scale = self.config.max_speed_mps / max_wheel_speed
            left_speed_mps *= scale
            right_speed_mps *= scale
            speed_cmd_mps *= scale
            angle_cmd *= scale

        self.state.speed_cmd_mps = speed_cmd_mps
        self.state.angle_cmd = angle_cmd
        self.state.left_speed_mps = left_speed_mps
        self.state.right_speed_mps = right_speed_mps

        return {
            "front_left": self._wheel_command_from_speed(left_speed_mps),
            "front_right": self._wheel_command_from_speed(right_speed_mps),
            "rear_left": self._wheel_command_from_speed(left_speed_mps),
            "rear_right": self._wheel_command_from_speed(right_speed_mps),
            "speed_mps": speed_cmd_mps,
            "steer_rad": angle_cmd,
            "target_heading": target_heading,
            "left_speed_mps": left_speed_mps,
            "right_speed_mps": right_speed_mps,
            "target_speed": target_forward,
            "measured_speed": measured_body_vx,
            "heading_error": heading_error,
        }

    @staticmethod
    def stop_command() -> Dict[str, float]:
        return VehicleController.stop_command()


class MecanumVehicleController:
    """Convert a world-frame target velocity into mecanum wheel commands."""

    def __init__(self, config: GroundVehicleControlConfig) -> None:
        self.config = config
        self.longitudinal_pid = PID(**vars(config.mecanum_speed_pid))
        self.lateral_pid = PID(**vars(config.mecanum_speed_pid))
        self.state = ControllerState()

    @staticmethod
    def _world_to_body(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
        """Rotate a planar world-frame velocity into the chassis frame."""

        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        return (
            cos_yaw * vx + sin_yaw * vy,
            -sin_yaw * vx + cos_yaw * vy,
        )

    def _wheel_command_from_speed(self, speed_mps: float) -> float:
        ratio = speed_mps / max(self.config.max_speed_mps, 1e-6)
        if speed_mps >= 0.0:
            command = ratio * self.config.wheel_command_max
        else:
            command = -ratio * self.config.wheel_command_min
        return max(
            self.config.wheel_command_min,
            min(self.config.wheel_command_max, command),
        )

    def reset(self) -> None:
        self.longitudinal_pid.reset()
        self.lateral_pid.reset()
        self.state = ControllerState()

    def step(
        self,
        target_vx: float,
        target_vy: float,
        target_vz: float,
        yaw: float,
        measured_vx: float,
        measured_vy: float,
        measured_vz: float,
        dt: float,
    ) -> Dict[str, float]:
        """Run one mecanum chassis control update."""

        _ = target_vz, measured_vz
        self.state.target_vx = target_vx
        self.state.target_vy = target_vy
        self.state.target_vz = target_vz
        self.state.measured_vx = measured_vx
        self.state.measured_vy = measured_vy
        self.state.measured_vz = measured_vz

        target_speed = sqrt(target_vx * target_vx + target_vy * target_vy)
        measured_speed = sqrt(measured_vx * measured_vx + measured_vy * measured_vy)
        stationary_target = target_speed < 1e-9
        # Mecanum mode has no target-yaw input in the shared controller interface.
        target_heading = yaw
        self.state.target_heading = target_heading
        self.state.target_speed = target_speed
        self.state.measured_speed = measured_speed

        target_body_vx, target_body_vy = self._world_to_body(
            target_vx,
            target_vy,
            yaw,
        )
        measured_body_vx, measured_body_vy = self._world_to_body(
            measured_vx,
            measured_vy,
            yaw,
        )

        if stationary_target:
            self.longitudinal_pid.reset()
            self.lateral_pid.reset()
            command_body_vx = 0.0
            command_body_vy = 0.0
        else:
            command_body_vx = self.longitudinal_pid.update(
                target_body_vx,
                measured_body_vx,
                dt,
            )
            command_body_vy = self.lateral_pid.update(
                target_body_vy,
                measured_body_vy,
                dt,
            )

        # X-roller inverse kinematics with zero commanded yaw rate.
        wheel_speeds = [
            command_body_vx - command_body_vy,
            command_body_vx + command_body_vy,
            command_body_vx + command_body_vy,
            command_body_vx - command_body_vy,
        ]
        max_wheel_speed = max(abs(speed) for speed in wheel_speeds)
        if max_wheel_speed > self.config.max_speed_mps:
            scale = self.config.max_speed_mps / max_wheel_speed
            wheel_speeds = [speed * scale for speed in wheel_speeds]
            command_body_vx *= scale
            command_body_vy *= scale

        (
            front_left_speed_mps,
            front_right_speed_mps,
            rear_left_speed_mps,
            rear_right_speed_mps,
        ) = wheel_speeds
        left_speed_mps = (front_left_speed_mps + rear_left_speed_mps) / 2.0
        right_speed_mps = (front_right_speed_mps + rear_right_speed_mps) / 2.0
        speed_cmd_mps = sqrt(
            command_body_vx * command_body_vx
            + command_body_vy * command_body_vy
        )
        self.state.speed_cmd_mps = speed_cmd_mps
        self.state.angle_cmd = 0.0
        self.state.heading_error = 0.0
        self.state.left_speed_mps = left_speed_mps
        self.state.right_speed_mps = right_speed_mps

        return {
            "front_left": self._wheel_command_from_speed(front_left_speed_mps),
            "front_right": self._wheel_command_from_speed(front_right_speed_mps),
            "rear_left": self._wheel_command_from_speed(rear_left_speed_mps),
            "rear_right": self._wheel_command_from_speed(rear_right_speed_mps),
            "speed_mps": speed_cmd_mps,
            "steer_rad": 0.0,
            "target_heading": target_heading,
            "left_speed_mps": left_speed_mps,
            "right_speed_mps": right_speed_mps,
            "target_speed": target_speed,
            "measured_speed": measured_speed,
            "heading_error": 0.0,
        }

    @staticmethod
    def stop_command() -> Dict[str, float]:
        return {
            "front_left": 0.0,
            "front_right": 0.0,
            "rear_left": 0.0,
            "rear_right": 0.0,
            "speed_mps": 0.0,
            "steer_rad": 0.0,
            "target_heading": 0.0,
            "left_speed_mps": 0.0,
            "right_speed_mps": 0.0,
        }


VehicleControllerType = Union[
    VehicleController,
    ProjectedVehicleController,
    MecanumVehicleController,
]


def build_vehicle_controller(
    config: GroundVehicleControlConfig,
) -> VehicleControllerType:
    """Build the globally selected ground-vehicle chassis controller."""

    if config.controller_type == "differential":
        return VehicleController(config)
    if config.controller_type == "projected":
        return ProjectedVehicleController(config)
    if config.controller_type == "mecanum":
        return MecanumVehicleController(config)
    raise ValueError(
        "unsupported ground vehicle controller_type: %s" % config.controller_type
    )
