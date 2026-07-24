"""Configuration checks for the three-vehicle right-triangle formation."""

from __future__ import annotations

import unittest

from swarm.algorithms.bearing import BearingOnlyFormationController, build_topology_from_matrices
from swarm.domain.models import MocapSnapshot, VehicleState
from configs.bearing_three import CONFIG


class ThreeVehicleConfigurationTest(unittest.TestCase):
    def test_roles_addresses_and_complete_topology(self) -> None:
        CONFIG.validate()
        self.assertEqual(CONFIG.vehicle_ids, ("car1", "car2", "car3"))
        self.assertEqual(tuple(vehicle.role for vehicle in CONFIG.vehicles), ("leader", "follower", "follower"))
        self.assertEqual(tuple(vehicle.address for vehicle in CONFIG.vehicles), ("10.1.1.81", "10.1.1.82", "10.1.1.83"))
        self.assertEqual(
            CONFIG.topology.adjacency_matrix,
            ((0, 1, 1), (1, 0, 1), (1, 1, 0)),
        )

    def test_requested_world_frame_bearings(self) -> None:
        bearings = CONFIG.topology.bearing_matrix
        self.assertEqual(bearings[0][1], (1.0, 0.0, 0.0))
        self.assertEqual(bearings[0][2], (0.0, 1.0, 0.0))
        self.assertEqual(bearings[1][2], (-1.0, 1.0, 0.0))
        for source in range(3):
            for target in range(3):
                self.assertEqual(
                    bearings[source][target],
                    tuple(-value for value in bearings[target][source]),
                )

    def test_ideal_right_triangle_has_zero_bearing_error(self) -> None:
        topology = build_topology_from_matrices(
            CONFIG.vehicle_ids,
            CONFIG.topology.adjacency_matrix,
            CONFIG.topology.bearing_matrix,
            CONFIG.topology.leader_mask,
        )
        planner = BearingOnlyFormationController(
            topology,
            CONFIG.leader_target_velocities,
            CONFIG.formation_control,
            CONFIG.runtime.command_speed_limit_mps,
        )
        snapshot = MocapSnapshot(
            1.0,
            {
                "car1": VehicleState("car1", x=0.0, y=0.0, valid=True),
                "car2": VehicleState("car2", x=1.0, y=0.0, valid=True),
                "car3": VehicleState("car3", x=0.0, y=1.0, valid=True),
            },
        )

        result = planner.step(snapshot, 0.02)

        self.assertTrue(result.ready)
        self.assertAlmostEqual(result.mean_error, 0.0, places=12)
        self.assertAlmostEqual(result.max_error, 0.0, places=12)
        for command in result.commands.values():
            self.assertAlmostEqual(command.vx, 0.0, places=12)
            self.assertAlmostEqual(command.vy, 0.0, places=12)
            self.assertAlmostEqual(command.vz, 0.0, places=12)


if __name__ == "__main__":
    unittest.main()
