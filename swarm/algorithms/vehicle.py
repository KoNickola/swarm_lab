"""Existing ground-vehicle velocity and heading control algorithm."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, pi, sqrt
from typing import Dict, Tuple

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
