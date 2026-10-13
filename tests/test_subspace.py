"""Subspace control, constraint validation, and dimensionless edge RMS tests."""

from dataclasses import replace
import unittest

import numpy as np

from configs.subspace_two import CONFIG
from swarm.algorithms.subspace import SubspaceFormationController
from swarm.application.planner_factory import build_swarm_planner
from swarm.domain.config import (
    SubspaceControlConfig, SubspaceEdgeConfig, SubspaceTopologyConfig,
)
from swarm.domain.models import MocapSnapshot, VehicleState
from swarm.domain.subspace import (
    matrix_from_expected_points,
    projection_from_free_basis,
    matrix_from_free_direction,
)


def snapshot(points, time=0):
    return MocapSnapshot(time, {name: VehicleState(name, x=p[0], y=p[1],
        z=p[2] if len(p) == 3 else 0, valid=True) for name, p in points.items()})


def controller(edges, ids=("a", "b"), control=None, space="planar_xy", limit=100):
    return SubspaceFormationController(ids, SubspaceTopologyConfig(tuple(edges)),
        control or SubspaceControlConfig(), space, limit)


def edge(matrix=((1, 0), (0, 0)), bias=None, source="a", target="b"):
    return SubspaceEdgeConfig(source, target, matrix, bias)


class SubspaceTest(unittest.TestCase):
    def test_example_and_factory(self):
        CONFIG.validate()
        planner = build_swarm_planner(CONFIG)
        self.assertIsInstance(planner, SubspaceFormationController)
        result = planner.step(snapshot({"car3": (0, 0), "car6": (.5, 2)}), .02)
        self.assertTrue(result.ready)
        self.assertEqual(result.subspace_edge_rms, 0)
        self.assertTrue(result.commands["car6"].valid)

    def test_raw_feedback_bias_and_reverse_edge(self):
        points = snapshot({"a": (0, 0), "b": (2, 3)})
        first = controller([edge(((2, 0), (0, 0)), (.5, 0))]).step(points, .02)
        reverse = controller([edge(((2, 0), (0, 0)), (-.5, 0), "b", "a")]).step(points, .02)
        self.assertEqual(first.commands, reverse.commands)
        self.assertEqual(first.commands["a"].vx, 3)
        self.assertEqual(first.commands["b"].vx, -3)
        self.assertEqual(first.commands["a"].vy, 0)
        self.assertAlmostEqual(first.subspace_edge_rms, 1.5 / np.sqrt(13))
        self.assertEqual(first.subspace_valid_edges, 1)

    def test_neighbor_sum_and_edge_rms_not_node_average(self):
        result = controller([edge(), edge(source="a", target="c")], ("a", "b", "c")).step(
            snapshot({"a": (0, 0), "b": (2, 0), "c": (0, 3)}), .02)
        self.assertEqual(result.commands["a"].vx, 2)
        self.assertAlmostEqual(result.subspace_edge_rms, np.sqrt(.5))
        self.assertEqual(result.subspace_valid_edges, 2)

    def test_spectral_norm_and_weight_scaling(self):
        point = snapshot({"a": (0, 0), "b": (1, 0)})
        first = controller([edge(((2, 0), (0, 1)))]).step(point, .02)
        second = controller([edge(((20, 0), (0, 10)))]).step(point, .02)
        self.assertEqual(first.subspace_edge_rms, 1)
        self.assertEqual(second.subspace_edge_rms, 1)
        self.assertEqual(second.commands["a"].vx, first.commands["a"].vx * 10)

    def test_homogeneous_distance_invariance(self):
        planner = controller([edge()])
        errors = [planner.step(snapshot({"a": (0, 0), "b": (s, s)}), .02).subspace_edge_rms
                  for s in (1, 10)]
        self.assertAlmostEqual(errors[0], errors[1])

    def test_affine_error_not_clipped(self):
        result = controller([edge(bias=(10, 0))]).step(snapshot({"a": (0, 0), "b": (1, 0)}), .02)
        self.assertEqual(result.subspace_edge_rms, 9)

    def test_invalid_metric_does_not_remove_collision_feedback(self):
        result = controller([edge(bias=(1, 0))]).step(snapshot({"a": (0, 0), "b": (0, 0)}), .02)
        self.assertIsNone(result.subspace_edge_rms)
        self.assertEqual(result.subspace_samples[0].invalid_reason, "zero_relative_length")
        self.assertEqual(result.commands["a"].vx, -1)
        for distance in (1e-10, 1e-9):
            result = controller([edge()]).step(snapshot({"a": (0, 0), "b": (distance, 0)}), .02)
            self.assertIsNone(result.subspace_edge_rms)

    def test_zero_matrix_missing_state_and_empty_graph(self):
        point = snapshot({"a": (0, 0), "b": (1, 0)})
        result = controller([edge(((0, 0), (0, 0)))]).step(point, .02)
        self.assertEqual(result.subspace_samples[0].invalid_reason, "zero_matrix")
        self.assertIsNone(result.subspace_edge_rms)
        result = controller([edge()]).step(snapshot({"a": (0, 0)}), .02)
        self.assertFalse(result.ready)
        self.assertFalse(result.commands["b"].valid)
        self.assertEqual(result.commands["b"].vx, 0)
        self.assertEqual(result.subspace_samples[0].invalid_reason, "invalid_state")
        self.assertEqual(result.subspace_valid_edges, 0)
        self.assertIsNone(controller([]).step(point, .02).subspace_edge_rms)

    def test_invalid_edges_excluded_from_rms(self):
        result = controller([edge(), edge(source="a", target="c")], ("a", "b", "c")).step(
            snapshot({"a": (0, 0), "b": (1, 0)}), .02)
        self.assertEqual(result.subspace_edge_rms, 1)
        self.assertEqual(result.subspace_valid_edges, 1)
        self.assertEqual(len(result.subspace_samples), 2)

    def test_spatial_and_planar(self):
        point = snapshot({"a": (0, 0, 0), "b": (0, 0, 2)})
        result = controller([edge(np.eye(3))], space="spatial_xyz").step(point, .02)
        self.assertEqual(result.commands["a"].vz, 2)
        planar = controller([edge()]).step(point, .02)
        self.assertEqual(planar.commands["a"].vz, 0)

    def test_task_translation_limit_and_reset(self):
        control = SubspaceControlConfig(modes=((1,), (0,), (1,), (0,)), coefficients=lambda t: (1+t,))
        planner = controller([edge()], control=control, limit=100)
        point = snapshot({"a": (0, 0), "b": (0, 2)})
        self.assertEqual(planner.step(point, .1).commands["a"].vx, 1)
        self.assertAlmostEqual(planner.step(point, .1).commands["b"].vx, 1.1)
        planner.reset()
        self.assertEqual(planner.step(point, .1).commands["a"].vx, 1)
        limited = controller([edge()], control=control, limit=.3).step(
            snapshot({"a": (0, 0), "b": (1, 2)}), .1)
        self.assertAlmostEqual(limited.commands["a"].vx, .3)
        self.assertEqual(limited.commands["b"].vx, 0)

    def test_invalid_task_function_result(self):
        for output in ((1, 2), (float("nan"),)):
            control = SubspaceControlConfig(modes=((1,), (0,), (1,), (0,)), coefficients=lambda t: output)
            with self.assertRaises(ValueError):
                controller([edge()], control=control).step(snapshot({"a": (0, 0), "b": (0, 1)}), .02)

    def test_convergence_preserves_free_coordinates_and_centroid(self):
        planner = controller([edge(bias=(.5, 0))])
        positions = np.array([[0., 2.], [1., 5.]])
        for _ in range(400):
            result = planner.step(snapshot(dict(zip(("a", "b"), positions))), .02)
            positions += .02 * np.array([[result.commands[name].vx, result.commands[name].vy] for name in ("a", "b")])
        self.assertAlmostEqual(positions[1, 0] - positions[0, 0], .5, places=6)
        np.testing.assert_allclose(positions[:, 1], [2, 5])
        self.assertAlmostEqual(positions[:, 0].mean(), .5)

    def test_free_task_drives_motion_without_constraint_error(self):
        control = SubspaceControlConfig(modes=((0,), (0,), (0,), (1,)), coefficients=(.2,))
        planner = controller([edge(bias=(.5, 0))], control=control)
        result = planner.step(snapshot({"a": (0, 0), "b": (.5, 1)}), .02)
        self.assertEqual(result.subspace_edge_rms, 0)
        self.assertEqual(result.commands["a"].vy, 0)
        self.assertEqual(result.commands["b"].vy, .2)

    def test_projection_builder(self):
        matrix = projection_from_free_basis(((1,), (2,), (3,)))
        np.testing.assert_allclose(np.array(matrix) @ [1, 2, 3], 0, atol=1e-9)
        np.testing.assert_allclose(np.array(matrix) @ matrix, matrix, atol=1e-9)
        with self.assertRaisesRegex(ValueError, "independent"):
            projection_from_free_basis(((1, 2), (2, 4)))
        np.testing.assert_allclose(projection_from_free_basis(((), ())), np.eye(2))

    def test_expected_points_build_one_dimensional_free_subspace(self):
        planner = controller([
            SubspaceEdgeConfig(
                "a", "b", bias=(0.5, 0.0),
                expected_points=((0.0, 0.0), (0.0, 1.0)),
            )
        ])
        result = planner.step(snapshot({"a": (0, 0), "b": (0.5, 2)}), .02)
        self.assertEqual(result.subspace_edge_rms, 0)
        np.testing.assert_allclose(
            matrix_from_expected_points(((0, 0), (0, 1))),
            ((1, 0), (0, 0)),
        )

    def test_free_direction_builds_one_dimensional_subspace_directly(self):
        planner = controller([
            SubspaceEdgeConfig(
                "a", "b", bias=(0.5, 0.0), free_direction=(0.0, 1.0)
            )
        ])
        result = planner.step(snapshot({"a": (0, 0), "b": (0.5, 2)}), .02)
        self.assertEqual(result.subspace_edge_rms, 0)
        np.testing.assert_allclose(
            matrix_from_free_direction((0, 1)), ((1, 0), (0, 0))
        )

    def test_invalid_constraints(self):
        cases = [
            [edge(source="unknown")], [edge(target="a")],
            [edge(), edge(source="b", target="a")],
            [edge(((1, 1), (0, 1)))], [edge(((-1, 0), (0, 1)))],
            [edge(((float("nan"), 0), (0, 1)))], [edge(np.eye(3))],
            [edge(bias=(1, 2, 3))], [edge(bias=(float("inf"), 0))],
        ]
        for edges in cases:
            with self.subTest(edges=edges), self.assertRaises(ValueError):
                controller(edges)
        with self.assertRaisesRegex(ValueError, "feasible"):
            controller([edge(bias=(1, 0)), edge(bias=(1, 0), source="b", target="c"),
                        edge(bias=(1, 0), source="a", target="c")], ("a", "b", "c"))
        with self.assertRaisesRegex(ValueError, "nullspace"):
            controller([edge()], control=SubspaceControlConfig(modes=((1,), (0,), (0,), (0,)), coefficients=(1,)))
        for k in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                controller([edge()], control=SubspaceControlConfig(k=k))

    def test_control_loop_freezes_task_time_during_mocap_loss(self):
        from swarm.application.control_loop import ControlLoop
        from swarm.domain.models import ActuationResult

        times = []
        def coefficients(t):
            times.append(t)
            return (.1,)
        control = SubspaceControlConfig(modes=((1,), (0,), (1,), (0,)), coefficients=coefficients)
        planner = controller([edge()], control=control)
        class Clock:
            time = 0.0
            def monotonic(self):
                return self.time
            def sleep(self, duration):
                self.time += duration
        class Source:
            valid = True
            def get_snapshot(self):
                return snapshot({"a": (0, 0), "b": (0, 1)})
            def all_valid(self):
                return self.valid
        class Actuator:
            stops = 0
            def execute(self, command, state, dt):
                return ActuationResult(command.vehicle_id, "ground_vehicle", {}, True)
            def stop(self):
                self.stops += 1
        class Telemetry:
            def emit(self, frame):
                pass
        clock, source = Clock(), Source()
        actuators = {name: Actuator() for name in ("a", "b")}
        loop = ControlLoop(50, True, source, planner, actuators, Telemetry(),
                           lambda: False, clock=clock, control_space="planar_xy")
        self.assertTrue(loop.run_cycle().planner.ready)
        source.valid = False
        clock.time += 10
        self.assertIsNone(loop.run_cycle())
        self.assertEqual(actuators["a"].stops, 1)
        source.valid = True
        frame = loop.run_cycle()
        self.assertEqual(frame.dt, .02)
        self.assertEqual(times, [0, .02])

    def test_configuration_rejects_leader_reference(self):
        with self.assertRaisesRegex(ValueError, "empty"):
            replace(CONFIG, leader_target_velocities={"car3": (0, 0, 0)}).validate()


if __name__ == "__main__":
    unittest.main()
