"""Tests for the continuous body-projection differential controller."""

from __future__ import annotations

from dataclasses import replace
from inspect import signature
from math import atan2, pi, sqrt
from typing import Dict
import unittest

from configs.bearing_two import CONFIG
from swarm.algorithms.vehicle import (
    ProjectedVehicleController,
    VehicleController,
    build_vehicle_controller,
)
from swarm.domain.config import GroundVehicleControlConfig, PIDConfig


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


def _projected_config() -> GroundVehicleControlConfig:
    return GroundVehicleControlConfig(
        track_width_m=0.2,
        max_speed_mps=1.0,
        wheel_command_min=-100.0,
        wheel_command_max=100.0,
        controller_type="projected",
        projected_speed_pid=PIDConfig(1.0, 0.0, 0.0, 1.0, None),
        projected_lookahead_m=0.1,
        projected_max_yaw_rate_rps=10.0,
    )


def _step(
    controller: ProjectedVehicleController,
    target_vx: float,
    target_vy: float,
    *,
    target_vz: float = 0.0,
    yaw: float = 0.0,
    measured_vx: float = 0.0,
    measured_vy: float = 0.0,
) -> Dict[str, float]:
    return controller.step(
        target_vx=target_vx,
        target_vy=target_vy,
        target_vz=target_vz,
        yaw=yaw,
        measured_vx=measured_vx,
        measured_vy=measured_vy,
        measured_vz=0.0,
        dt=0.1,
    )


class ProjectedVehicleControllerTest(unittest.TestCase):
    def test_forward_velocity_drives_all_wheels_equally(self) -> None:
        result = _step(ProjectedVehicleController(_projected_config()), 0.4, 0.0)

        self.assertEqual(set(result), OUTPUT_KEYS)
        for key in ("front_left", "front_right", "rear_left", "rear_right"):
            self.assertAlmostEqual(result[key], 40.0)

    def test_velocity_behind_vehicle_stops_translation_and_turns(self) -> None:
        config = replace(_projected_config(), wheel_command_min=-80.0)
        result = _step(ProjectedVehicleController(config), -0.5, 0.0)

        self.assertAlmostEqual(result["speed_mps"], 0.0)
        self.assertAlmostEqual(result["target_speed"], 0.0)
        self.assertAlmostEqual(result["steer_rad"], 10.0)
        self.assertAlmostEqual(result["front_left"], -80.0)
        self.assertAlmostEqual(result["front_right"], 100.0)
        self.assertAlmostEqual(result["rear_left"], -80.0)
        self.assertAlmostEqual(result["rear_right"], 100.0)

    def test_lateral_velocity_generates_continuous_yaw_rate(self) -> None:
        controller = ProjectedVehicleController(_projected_config())

        positive = _step(controller, 0.0, 0.001)
        negative = _step(controller, 0.0, -0.001)

        self.assertAlmostEqual(positive["steer_rad"], pi / 200.0)
        self.assertAlmostEqual(negative["steer_rad"], -pi / 200.0)
        self.assertAlmostEqual(positive["front_left"], -pi / 20.0)
        self.assertAlmostEqual(positive["front_right"], pi / 20.0)
        self.assertAlmostEqual(negative["front_left"], pi / 20.0)
        self.assertAlmostEqual(negative["front_right"], -pi / 20.0)

    def test_crossing_target_does_not_reverse_translation(self) -> None:
        controller = ProjectedVehicleController(_projected_config())

        before = _step(controller, 0.001, 0.0)
        after = _step(controller, -0.001, 0.0)

        self.assertAlmostEqual(before["speed_mps"], 0.001)
        self.assertAlmostEqual(after["speed_mps"], 0.0)
        self.assertAlmostEqual(after["steer_rad"], pi / 100.0)
        self.assertGreaterEqual(before["speed_mps"], 0.0)
        self.assertGreaterEqual(after["speed_mps"], 0.0)

    def test_world_velocity_is_projected_into_body_frame(self) -> None:
        result = _step(
            ProjectedVehicleController(_projected_config()),
            0.2,
            0.0,
            yaw=pi / 2.0,
        )

        self.assertAlmostEqual(result["speed_mps"], 0.0, places=12)
        self.assertAlmostEqual(result["steer_rad"], -pi)
        self.assertAlmostEqual(result["front_left"], 10.0 * pi)
        self.assertAlmostEqual(result["front_right"], -10.0 * pi)

    def test_forward_only_pid_uses_projected_measured_velocity(self) -> None:
        controller = ProjectedVehicleController(_projected_config())

        forward = _step(controller, 0.4, 0.0, measured_vx=0.1)
        controller.reset()
        reverse = _step(controller, -0.4, 0.0, measured_vx=-0.1)

        self.assertAlmostEqual(forward["speed_mps"], 0.3)
        self.assertAlmostEqual(reverse["speed_mps"], 0.0)
        self.assertAlmostEqual(forward["measured_speed"], 0.1)
        self.assertAlmostEqual(reverse["measured_speed"], -0.1)

    def test_negative_pid_output_is_clamped_to_zero(self) -> None:
        result = _step(
            ProjectedVehicleController(_projected_config()),
            0.1,
            0.0,
            measured_vx=0.2,
        )

        self.assertAlmostEqual(result["speed_mps"], 0.0)
        self.assertAlmostEqual(result["left_speed_mps"], 0.0)
        self.assertAlmostEqual(result["right_speed_mps"], 0.0)

    def test_wheel_saturation_preserves_differential_ratio(self) -> None:
        result = _step(
            ProjectedVehicleController(_projected_config()),
            1.0,
            0.1,
        )
        yaw_rate = sqrt(1.01) * atan2(0.1, 1.0) / 0.1
        expected_left = 100.0 * (1.0 - 0.1 * yaw_rate) / (
            1.0 + 0.1 * yaw_rate
        )

        self.assertAlmostEqual(result["front_left"], expected_left)
        self.assertAlmostEqual(result["front_right"], 100.0)
        self.assertAlmostEqual(result["rear_left"], result["front_left"])
        self.assertAlmostEqual(result["rear_right"], result["front_right"])

    def test_yaw_rate_is_limited_before_wheel_saturation(self) -> None:
        config = replace(
            _projected_config(),
            projected_max_yaw_rate_rps=0.5,
        )
        result = _step(ProjectedVehicleController(config), 0.0, 1.0)

        self.assertAlmostEqual(result["steer_rad"], 0.5)
        self.assertAlmostEqual(result["left_speed_mps"], -0.05)
        self.assertAlmostEqual(result["right_speed_mps"], 0.05)

    def test_stationary_target_stops_and_resets_longitudinal_pid(self) -> None:
        controller = ProjectedVehicleController(_projected_config())
        _step(controller, 0.2, 0.1)

        result = _step(controller, 0.0, 0.0)

        for key in ("front_left", "front_right", "rear_left", "rear_right"):
            self.assertEqual(result[key], 0.0)
        self.assertFalse(controller.longitudinal_pid.initialized)

    def test_vertical_target_component_does_not_affect_planar_control(self) -> None:
        without_vertical = _step(
            ProjectedVehicleController(_projected_config()),
            0.2,
            -0.1,
        )
        with_vertical = _step(
            ProjectedVehicleController(_projected_config()),
            0.2,
            -0.1,
            target_vz=2.0,
        )

        self.assertEqual(without_vertical, with_vertical)


class ProjectedVehicleSelectionTest(unittest.TestCase):
    def test_step_signature_matches_existing_differential_controller(self) -> None:
        self.assertEqual(
            signature(ProjectedVehicleController.step),
            signature(VehicleController.step),
        )

    def test_projected_controller_can_be_selected(self) -> None:
        controller = build_vehicle_controller(_projected_config())

        self.assertIsInstance(controller, ProjectedVehicleController)

    def test_stop_command_matches_existing_controller_interface(self) -> None:
        self.assertEqual(
            ProjectedVehicleController.stop_command(),
            VehicleController.stop_command(),
        )

    def test_invalid_projected_parameters_are_rejected(self) -> None:
        invalid_control = replace(
            CONFIG.ground_vehicle_control,
            controller_type="projected",
            projected_lookahead_m=0.0,
        )
        invalid_config = replace(
            CONFIG,
            ground_vehicle_control=invalid_control,
        )

        with self.assertRaisesRegex(ValueError, "projected_lookahead_m"):
            invalid_config.validate()


if __name__ == "__main__":
    unittest.main()
