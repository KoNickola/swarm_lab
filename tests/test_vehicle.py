"""Tests for selectable differential and mecanum chassis controllers."""

from __future__ import annotations

from dataclasses import asdict, replace
from inspect import signature
from math import pi
from typing import Dict
import unittest

from configs.bearing_two import CONFIG
from swarm.algorithms.vehicle import (
    MecanumVehicleController,
    VehicleController,
    build_vehicle_controller,
)
from swarm.domain.config import (
    GROUND_VEHICLE_CONTROLLER_TYPE,
    GroundVehicleControlConfig,
    PIDConfig,
)
from swarm.domain.models import VehicleState, VelocityCommand
from swarm.infrastructure.ground_vehicle import GroundVehicleActuator


OUTPUT_KEYS = {
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
    "speed_mps",
    "steer_rad",
    "target_heading",
    "left_speed_mps",
    "right_speed_mps",
    "target_speed",
    "measured_speed",
    "heading_error",
}


def _mecanum_config() -> GroundVehicleControlConfig:
    return GroundVehicleControlConfig(
        max_speed_mps=1.0,
        wheel_command_min=-100.0,
        wheel_command_max=100.0,
        controller_type="mecanum",
        mecanum_speed_pid=PIDConfig(1.0, 0.0, 0.0, 1.0, None),
    )


def _step(
    controller: MecanumVehicleController,
    target_vx: float,
    target_vy: float,
    *,
    target_vz: float = 0.0,
    yaw: float = 0.0,
) -> Dict[str, float]:
    return controller.step(
        target_vx=target_vx,
        target_vy=target_vy,
        target_vz=target_vz,
        yaw=yaw,
        measured_vx=0.0,
        measured_vy=0.0,
        measured_vz=0.0,
        dt=0.1,
    )


class MecanumVehicleControllerTest(unittest.TestCase):
    def test_forward_velocity_drives_all_wheels_equally(self) -> None:
        result = _step(MecanumVehicleController(_mecanum_config()), 0.4, 0.0)

        self.assertEqual(set(result), OUTPUT_KEYS)
        for key in ("front_left", "front_right", "rear_left", "rear_right"):
            self.assertAlmostEqual(result[key], 40.0)

    def test_positive_body_lateral_velocity_uses_x_roller_pattern(self) -> None:
        result = _step(MecanumVehicleController(_mecanum_config()), 0.0, 0.4)

        self.assertAlmostEqual(result["front_left"], -40.0)
        self.assertAlmostEqual(result["front_right"], 40.0)
        self.assertAlmostEqual(result["rear_left"], 40.0)
        self.assertAlmostEqual(result["rear_right"], -40.0)

    def test_world_velocity_is_rotated_into_body_frame(self) -> None:
        result = _step(
            MecanumVehicleController(_mecanum_config()),
            0.4,
            0.0,
            yaw=pi / 2.0,
        )

        self.assertAlmostEqual(result["front_left"], 40.0)
        self.assertAlmostEqual(result["front_right"], -40.0)
        self.assertAlmostEqual(result["rear_left"], -40.0)
        self.assertAlmostEqual(result["rear_right"], 40.0)

    def test_wheel_saturation_preserves_inverse_kinematics_ratio(self) -> None:
        result = _step(MecanumVehicleController(_mecanum_config()), 0.8, 0.4)

        self.assertAlmostEqual(result["front_left"], 100.0 / 3.0)
        self.assertAlmostEqual(result["front_right"], 100.0)
        self.assertAlmostEqual(result["rear_left"], 100.0)
        self.assertAlmostEqual(result["rear_right"], 100.0 / 3.0)

    def test_axis_pid_uses_measured_world_velocity(self) -> None:
        controller = MecanumVehicleController(_mecanum_config())
        result = controller.step(
            target_vx=0.4,
            target_vy=0.0,
            target_vz=0.0,
            yaw=0.0,
            measured_vx=0.1,
            measured_vy=0.0,
            measured_vz=0.0,
            dt=0.1,
        )

        for key in ("front_left", "front_right", "rear_left", "rear_right"):
            self.assertAlmostEqual(result[key], 30.0)

    def test_negative_speed_uses_configured_reverse_command_range(self) -> None:
        config = replace(_mecanum_config(), wheel_command_min=-80.0)
        result = _step(MecanumVehicleController(config), -0.5, 0.0)

        for key in ("front_left", "front_right", "rear_left", "rear_right"):
            self.assertAlmostEqual(result[key], -40.0)

    def test_vertical_target_component_does_not_become_yaw_rate(self) -> None:
        without_vertical = _step(
            MecanumVehicleController(_mecanum_config()),
            0.2,
            -0.1,
            target_vz=0.0,
        )
        with_vertical = _step(
            MecanumVehicleController(_mecanum_config()),
            0.2,
            -0.1,
            target_vz=2.0,
        )

        self.assertEqual(without_vertical, with_vertical)

    def test_stationary_target_stops_wheels_and_resets_axis_pids(self) -> None:
        controller = MecanumVehicleController(_mecanum_config())
        _step(controller, 0.2, 0.1)

        result = _step(controller, 0.0, 0.0)

        for key in ("front_left", "front_right", "rear_left", "rear_right"):
            self.assertEqual(result[key], 0.0)
        self.assertFalse(controller.longitudinal_pid.initialized)
        self.assertFalse(controller.lateral_pid.initialized)

    def test_stop_command_matches_existing_controller_interface(self) -> None:
        self.assertEqual(
            MecanumVehicleController.stop_command(),
            VehicleController.stop_command(),
        )


class VehicleControllerSelectionTest(unittest.TestCase):
    def test_default_mecanum_pid_is_separate_from_existing_speed_pid(self) -> None:
        config = GroundVehicleControlConfig()
        differential = VehicleController(config)
        mecanum = MecanumVehicleController(config)

        self.assertEqual(vars(config.speed_pid), {
            "kp": 1.2,
            "ki": 0.0,
            "kd": 0.5,
            "integral_limit": 1.0,
            "output_limit": 0.3,
        })
        self.assertEqual(vars(config.mecanum_speed_pid), {
            "kp": 1.2,
            "ki": 0.0,
            "kd": 0.0,
            "integral_limit": 1.0,
            "output_limit": 0.3,
        })
        self.assertEqual(differential.speed_pid.kd, 0.5)
        self.assertEqual(mecanum.longitudinal_pid.kd, 0.0)
        self.assertEqual(mecanum.lateral_pid.kd, 0.0)
        serialized = asdict(config)
        self.assertEqual(serialized["speed_pid"]["kd"], 0.5)
        self.assertEqual(serialized["mecanum_speed_pid"]["kd"], 0.0)

    def test_mecanum_step_signature_matches_existing_controller(self) -> None:
        self.assertEqual(
            signature(MecanumVehicleController.step),
            signature(VehicleController.step),
        )

    def test_global_default_keeps_existing_differential_controller(self) -> None:
        self.assertEqual(GROUND_VEHICLE_CONTROLLER_TYPE, "differential")
        controller = build_vehicle_controller(GroundVehicleControlConfig())

        self.assertIsInstance(controller, VehicleController)

    def test_mecanum_controller_can_be_selected(self) -> None:
        controller = build_vehicle_controller(_mecanum_config())

        self.assertIsInstance(controller, MecanumVehicleController)

    def test_invalid_controller_type_is_rejected_by_configuration(self) -> None:
        invalid_control = replace(
            CONFIG.ground_vehicle_control,
            controller_type="unsupported",
        )
        invalid_config = replace(
            CONFIG,
            ground_vehicle_control=invalid_control,
        )

        with self.assertRaisesRegex(ValueError, "controller_type"):
            invalid_config.validate()


class _RecordingDriver:
    def __init__(self) -> None:
        self.commands = []

    def send(self, command) -> bool:
        self.commands.append(dict(command))
        return True


class MecanumActuatorIntegrationTest(unittest.TestCase):
    def test_execute_and_stop_use_selected_mecanum_controller(self) -> None:
        driver = _RecordingDriver()
        actuator = GroundVehicleActuator(
            vehicle_id="car1",
            controller=MecanumVehicleController(_mecanum_config()),
            driver=driver,
        )

        result = actuator.execute(
            VelocityCommand("car1", vx=0.2, valid=True),
            VehicleState("car1", valid=True),
            dt=0.1,
        )
        stopped = actuator.stop()

        self.assertTrue(result.sent)
        self.assertAlmostEqual(result.values["front_left"], 20.0)
        self.assertTrue(stopped)
        self.assertEqual(driver.commands[-1], MecanumVehicleController.stop_command())


if __name__ == "__main__":
    unittest.main()
