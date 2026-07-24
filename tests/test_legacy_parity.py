"""Characterization tests comparing refactored algorithms with legacy code."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

from swarm.algorithms.bearing import (
    BearingOnlyFormationController,
    build_topology_from_matrices,
)
from swarm.algorithms.vehicle import VehicleController
from swarm.domain.models import MocapSnapshot, VehicleState
from swarm.infrastructure.udp_chassis import ChassisSocketDriver, SocketConfig
from configs.bearing_two import CONFIG


LEGACY_ROOT = Path(__file__).resolve().parents[2] / "bearing_swarm"


def _load_legacy_module(name: str, filename: str):
    specification = importlib.util.spec_from_file_location(name, LEGACY_ROOT / filename)
    if specification is None or specification.loader is None:
        raise ImportError(filename)
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


legacy_param = _load_legacy_module("param", "param.py")
legacy_controller = _load_legacy_module("controller", "controller.py")
legacy_config = _load_legacy_module("experiment_config", "experiment_config.py")
legacy_mocap = _load_legacy_module("mocap_bridge", "mocap_bridge.py")
legacy_formation = _load_legacy_module("legacy_swarm_planner", "swarm_planner.py")


class VehicleControllerParityTest(unittest.TestCase):
    def test_control_sequence_matches_legacy_output(self) -> None:
        old = legacy_controller.VehicleController()
        new = VehicleController(CONFIG.ground_vehicle_control)
        sequence = (
            (0.20, 0.00, 0.0, 0.00, 0.00, 0.00, 0.0, 0.020),
            (0.20, 0.10, 0.0, 0.15, 0.05, 0.01, 0.0, 0.021),
            (-0.10, 0.15, 0.0, 2.80, 0.08, 0.02, 0.0, 0.019),
            (0.00, 0.00, 0.0, 1.20, 0.01, 0.00, 0.0, 0.020),
            (0.12, -0.04, 0.0, -0.30, 0.03, -0.01, 0.0, 0.020),
        )
        for values in sequence:
            old_result = old.step(*values)
            new_result = new.step(*values)
            self.assertEqual(set(old_result), set(new_result))
            for key in old_result:
                self.assertAlmostEqual(old_result[key], new_result[key], places=12, msg=key)


class BearingFormationParityTest(unittest.TestCase):
    def test_two_vehicle_commands_match_legacy_output(self) -> None:
        old_topology = legacy_formation.build_topology_from_matrices(
            vehicle_ids=legacy_config.VEHICLE_IDS,
            adjacency_matrix=legacy_config.ADJACENCY_MATRIX.tolist(),
            bearing_matrix=legacy_config.BEARING_MATRIX,
            leader_mask=legacy_config.LEADER_MASK,
        )
        old = legacy_formation.BearingOnlyFormationController(
            legacy_formation.FormationConfig(topology=old_topology)
        )
        new_topology = build_topology_from_matrices(
            CONFIG.vehicle_ids,
            CONFIG.topology.adjacency_matrix,
            CONFIG.topology.bearing_matrix,
            CONFIG.topology.leader_mask,
        )
        new = BearingOnlyFormationController(
            new_topology,
            CONFIG.leader_target_velocities,
            CONFIG.formation_control,
            legacy_param.FORMATION_MAX_SPEED,
        )

        old_states = {
            "car1": legacy_mocap.VehicleMocapState(
                "car1", x=0.0, y=0.0, vx=0.01, valid=True, pose_ready=True, twist_ready=True
            ),
            "car2": legacy_mocap.VehicleMocapState(
                "car2", x=0.8, y=0.3, vy=0.02, valid=True, pose_ready=True, twist_ready=True
            ),
        }
        new_states = {
            "car1": VehicleState("car1", x=0.0, y=0.0, vx=0.01, valid=True, pose_ready=True, twist_ready=True),
            "car2": VehicleState("car2", x=0.8, y=0.3, vy=0.02, valid=True, pose_ready=True, twist_ready=True),
        }
        for dt in (0.02, 0.021, 0.019):
            old_result = old.step(legacy_mocap.MocapFrameSnapshot(1.0, old_states), dt)
            new_result = new.step(MocapSnapshot(1.0, new_states), dt)
            self.assertEqual(old_result.ready, new_result.ready)
            self.assertAlmostEqual(old_result.mean_error, new_result.mean_error, places=12)
            self.assertAlmostEqual(old_result.max_error, new_result.max_error, places=12)
            for vehicle_id in CONFIG.vehicle_ids:
                old_command = old_result.commands[vehicle_id]
                new_command = new_result.commands[vehicle_id]
                self.assertAlmostEqual(old_command.vx, new_command.vx, places=12)
                self.assertAlmostEqual(old_command.vy, new_command.vy, places=12)
                self.assertAlmostEqual(old_command.vz, new_command.vz, places=12)
                self.assertEqual(old_command.error_norm, new_command.error_vector)


class FormationCommandSpeedLimitTest(unittest.TestCase):
    def test_leader_command_uses_shared_speed_limit(self) -> None:
        topology = build_topology_from_matrices(
            CONFIG.vehicle_ids,
            CONFIG.topology.adjacency_matrix,
            CONFIG.topology.bearing_matrix,
            CONFIG.topology.leader_mask,
        )
        planner = BearingOnlyFormationController(
            topology,
            {"car1": (1.0, 0.0, 0.0)},
            CONFIG.formation_control,
            command_speed_limit_mps=0.2,
        )
        snapshot = MocapSnapshot(
            1.0,
            {
                "car1": VehicleState("car1", valid=True),
                "car2": VehicleState("car2", x=0.5, valid=True),
            },
        )

        result = planner.step(snapshot, 0.02)

        self.assertAlmostEqual(result.commands["car1"].vx, 0.2)
        self.assertAlmostEqual(result.commands["car1"].vy, 0.0)
        self.assertAlmostEqual(result.commands["car1"].vz, 0.0)


class ChassisPacketParityTest(unittest.TestCase):
    def test_packet_field_order_and_rounding_match_legacy_format(self) -> None:
        driver = ChassisSocketDriver("127.0.0.1", SocketConfig())
        payload = driver.encode(
            {
                "front_left": 10.6,
                "front_right": -9.6,
                "rear_left": 4.4,
                "rear_right": -3.4,
            }
        )
        self.assertEqual(payload, b"<11,-10,4,-3>")


if __name__ == "__main__":
    unittest.main()
