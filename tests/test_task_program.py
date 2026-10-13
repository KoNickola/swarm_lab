"""Task allocation, scheduling, and legacy subspace integration regressions."""

from dataclasses import asdict, replace
import json
from math import sqrt
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock

import numpy as np

from configs.flyingwing import CONFIG
from swarm.algorithms.task_program import ModalTaskController
from swarm.application.planner_factory import build_swarm_planner
from swarm.application.control_loop import ControlLoop
from swarm.domain.models import ControlFrame, MocapSnapshot, VehicleState
from swarm.domain.tasks import (
    Delay, EdgeLengthGoal, Hold, NodePositionGoal, Parallel,
    ProjectedDistanceGoal, RelativePositionGoal, Release, Sequence,
    TaskFeedbackConfig, goal_measurement,
)
from swarm.infrastructure.telemetry.recorder import ExperimentRecorder
from task_programs.flyingwing_goals import PROGRAM


def snapshot(points, velocities=None):
    velocities = velocities or {}
    return MocapSnapshot(0.0, {
        name: VehicleState(name, x=p[0], y=p[1], z=p[2] if len(p) == 3 else 0,
                           vx=velocities.get(name, (0, 0))[0],
                           vy=velocities.get(name, (0, 0))[1], valid=True)
        for name, p in points.items()
    })


def engine(program, ids=("a",), modes=None, **options):
    modes = np.eye(2 * len(ids)) if modes is None else modes
    return ModalTaskController(ids, 2, modes, TaskFeedbackConfig(program, **options))


def goal(name="move", target=(1.0, 0.0), **options):
    return NodePositionGoal(id=name, node_id="a", target=target, settle_s=0.04, **options)


def step(controller, points=None, dt=0.02, velocities=None):
    points = {"a": (0, 0)} if points is None else points
    return controller.step(snapshot(points, velocities), dt,
                           np.zeros((len(controller.vehicle_ids), 2)))


class TaskProgramTest(unittest.TestCase):
    def test_goal_jacobians_match_finite_differences(self):
        points = np.array([[0.1, -0.2, .4], [1.2, .7, -.3]])
        goals = (
            NodePositionGoal(id="node", node_id="b", target=(0, 0), axes=(0, 2)),
            RelativePositionGoal(id="relative", source_id="a", target_id="b", target=(0, 0, 0)),
            EdgeLengthGoal(id="length", source_id="a", target_id="b", target=1),
            ProjectedDistanceGoal(id="projection", source_id="a", target_id="b",
                                  target=1, axis=(1, 2, -1)),
        )
        for item in goals:
            value, jacobian = goal_measurement(item, ("a", "b"), 3, points)
            numeric = np.zeros_like(jacobian)
            for axis in range(6):
                delta = np.zeros(6)
                delta[axis] = 1e-6
                plus, _ = goal_measurement(item, ("a", "b"), 3, points + delta.reshape(2, 3))
                minus, _ = goal_measurement(item, ("a", "b"), 3, points - delta.reshape(2, 3))
                numeric[:, axis] = (plus - minus) / 2e-6
            np.testing.assert_allclose(jacobian, numeric, atol=1e-9)
            self.assertEqual(len(value), jacobian.shape[0])

    def test_weighted_conflict_does_not_report_completion(self):
        controller = engine(Parallel((goal("right", weight=3), goal("left", target=(-1, 0)))))
        result = step(controller, dt=1)
        self.assertAlmostEqual(result.coefficients[0], .1)
        self.assertEqual(result.status, "running")
        self.assertTrue(all(not item.settled for item in result.goals))
        self.assertGreater(result.allocation_residual, 0)

    def test_unreachable_coordinate_is_reported_as_residual(self):
        controller = engine(goal(target=(0, 1)), modes=np.array([[1.0], [0.0]]))
        result = step(controller)
        self.assertEqual(result.coefficients, (0.0,))
        self.assertGreater(result.allocation_residual, 0)
        self.assertFalse(result.goals[0].settled)

    def test_formation_velocity_is_compensated(self):
        controller = engine(goal(target=(0, 0)))
        result = controller.step(snapshot({"a": (0, 0)}), 1, np.array([.1, -.05]))
        np.testing.assert_allclose(result.coefficients, [-.1, .05], atol=1e-10)

    def test_speed_and_acceleration_limits(self):
        controller = engine(goal())
        first = step(controller)
        self.assertAlmostEqual(first.coefficients[0], .016)
        for _ in range(50):
            result = step(controller)
        self.assertLessEqual(max(abs(x) for x in result.coefficients), .2)

    def test_sequence_waits_for_measured_position_and_speed(self):
        controller = engine(Sequence((goal("first"), goal("second", target=(2, 0)))))
        for _ in range(5):
            result = step(controller)
            self.assertEqual(result.active_nodes, ("first",))
        for _ in range(5):
            result = step(controller, {"a": (1, 0)}, velocities={"a": (.1, 0)})
            self.assertFalse(result.goals[0].settled)
        step(controller, {"a": (1, 0)})
        step(controller, {"a": (1, 0)})
        result = step(controller, {"a": (1, 0)})
        self.assertEqual(result.active_nodes, ("second",))

    def test_parallel_and_delayed_start(self):
        controller = engine(Parallel((goal("now"), Sequence((Delay(.04), goal("later"))))))
        result = step(controller)
        self.assertEqual([item.goal_id for item in result.goals], ["now"])
        step(controller)
        result = step(controller)
        self.assertEqual({item.goal_id for item in result.goals}, {"now", "later"})

    def test_completed_goal_hold_flag(self):
        for keep in (False, True):
            controller = engine(Sequence((goal(target=(0, 0), hold_after_completion=keep), Delay(5))))
            step(controller)
            step(controller)
            result = step(controller, {"a": (.2, 0)})
            if keep:
                self.assertLess(result.coefficients[0], 0)
                self.assertEqual(result.goals[0].state, "holding")
            else:
                self.assertEqual(result.coefficients, (0.0, 0.0))
                self.assertEqual(result.goals, ())

    def test_held_goal_replacement_and_release(self):
        controller = engine(Sequence((
            goal(target=(0, 0), hold_after_completion=True),
            goal(target=(1, 0), hold_after_completion=True),
            Release(("move",)), Delay(1),
        )))
        step(controller)
        step(controller)
        result = step(controller)
        self.assertEqual(len(result.goals), 1)
        self.assertGreater(result.coefficients[0], 0)
        step(controller, {"a": (1, 0)})
        step(controller, {"a": (1, 0)})
        result = step(controller, {"a": (1, 0)})
        self.assertEqual(result.goals, ())
        self.assertEqual(result.coefficients, (0.0, 0.0))

    def test_parallel_rechecks_held_goal_before_advancing(self):
        controller = engine(Sequence((Parallel((
            goal(target=(0, 0), hold_after_completion=True), Delay(.1),
        )), goal("next"))))
        step(controller)
        step(controller)
        for _ in range(10):
            result = step(controller, {"a": (.2, 0)})
            self.assertNotIn("next", result.active_nodes)
        step(controller)
        step(controller)
        self.assertIn("next", step(controller).active_nodes)

    def test_hold_waits_for_stop_and_restarts_after_disturbance(self):
        controller = engine(Sequence((Hold(.06), goal("next"))))
        for _ in range(10):
            self.assertEqual(step(controller, velocities={"a": (.1, 0)}).status, "running")
        step(controller)
        step(controller)
        disturbed = step(controller, {"a": (.2, 0)})
        self.assertLess(disturbed.coefficients[0], 0)
        step(controller)
        step(controller)
        self.assertNotIn("next", step(controller).active_nodes)
        self.assertIn("next", step(controller).active_nodes)

    def test_invalid_state_freezes_delay_and_clears_dwell(self):
        controller = engine(Sequence((Delay(.06), Hold(.06))))
        step(controller)
        invalid = controller.step(MocapSnapshot(0, {}), .5, np.zeros(2))
        self.assertEqual(invalid.status, "waiting")
        self.assertAlmostEqual(invalid.elapsed_s, .02)
        step(controller)
        step(controller)
        step(controller)
        step(controller)
        controller.pause()
        self.assertEqual(step(controller).status, "running")
        self.assertEqual(step(controller).status, "running")
        self.assertEqual(step(controller).status, "completed")

    def test_hold_does_not_count_vehicle_motion_outside_task_modes(self):
        controller = engine(Hold(.04), ids=("a", "b"),
                            modes=np.array([[1.0], [0.0], [1.0], [0.0]]))
        for _ in range(10):
            result = step(controller, {"a": (0, 0), "b": (1, 0)},
                          velocities={"a": (0, .1), "b": (0, -.1)})
            self.assertEqual(result.status, "running")

    def test_timeout_holds_current_modal_position_without_advancing(self):
        controller = engine(Sequence((goal(timeout_s=.04), goal("next"))))
        step(controller)
        result = step(controller)
        self.assertEqual(result.status, "failed")
        self.assertIn("move", result.reason)
        result = step(controller, {"a": (.1, 0)})
        self.assertLess(result.coefficients[0], 0)
        self.assertNotIn("next", result.active_nodes)

    def test_coincident_edge_waits_without_advancing(self):
        controller = engine(EdgeLengthGoal(id="edge", source_id="a", target_id="b", target=1),
                            ids=("a", "b"))
        result = step(controller, {"a": (0, 0), "b": (0, 0)})
        self.assertEqual(result.status, "waiting")
        self.assertIn("coincident", result.reason)
        self.assertEqual(result.elapsed_s, 0)

    def test_reset_and_instances_do_not_share_program_state(self):
        program = Sequence((goal(target=(0, 0)), Delay(.1)))
        first, second = engine(program), engine(program)
        step(first)
        step(first)
        first.reset()
        self.assertEqual(step(first), step(second))

    def test_control_loop_mocap_loss_resets_continuous_hold_time(self):
        config = replace(CONFIG, subspace_control=replace(
            CONFIG.subspace_control, coefficients=(), task_feedback=TaskFeedbackConfig(Hold(.06)),
        ))
        planner = build_swarm_planner(config)
        state = snapshot({name: (i, 1) for i, name in enumerate(CONFIG.vehicle_ids)})
        source, clock, telemetry = Mock(), Mock(), Mock()
        source.get_snapshot.return_value = state
        source.all_valid.return_value = True
        clock.monotonic.return_value = 0.0
        actuators = {name: Mock() for name in CONFIG.vehicle_ids}
        loop = ControlLoop(50, True, source, planner, actuators, telemetry,
                           lambda: False, clock=clock, control_space="planar_xy")
        loop.run_cycle()
        loop.run_cycle()
        source.all_valid.return_value = False
        self.assertIsNone(loop.run_cycle())
        for actuator in actuators.values():
            actuator.stop.assert_called_once()
        source.all_valid.return_value = True
        clock.monotonic.return_value = 10.0
        self.assertEqual(loop.run_cycle().planner.task_feedback.status, "running")
        self.assertEqual(loop.run_cycle().planner.task_feedback.status, "running")
        self.assertEqual(loop.run_cycle().planner.task_feedback.status, "completed")

    def test_node_axis_selection_does_not_drive_other_coordinates(self):
        controller = engine(NodePositionGoal(id="x", node_id="a", axes=(0,), target=(1,)))
        result = step(controller, dt=1)
        self.assertGreater(result.coefficients[0], 0)
        self.assertEqual(result.coefficients[1], 0)

    def test_length_goal_converges_without_moving_pair_midpoint(self):
        controller = engine(EdgeLengthGoal(id="edge", source_id="a", target_id="b", target=1),
                            ids=("a", "b"))
        points = np.array(((0., 0.), (.5, .3)))
        midpoint = points.mean(axis=0)
        velocities = np.zeros((2, 2))
        for _ in range(1000):
            result = step(controller, dict(zip(("a", "b"), points)),
                          velocities=dict(zip(("a", "b"), velocities)))
            velocities = np.array(result.coefficients).reshape(2, 2)
            points += .02 * velocities
            if result.status == "completed":
                break
        self.assertEqual(result.status, "completed")
        self.assertAlmostEqual(np.linalg.norm(points[1] - points[0]), 1, delta=.02)
        np.testing.assert_allclose(points.mean(axis=0), midpoint, atol=1e-12)

    def test_validation_rejects_invalid_programs(self):
        invalid = (
            Parallel((goal(), goal())), Parallel((Delay(1), Hold(1))), Sequence(()),
            replace(goal(), node_id="missing"), replace(goal(), axes=(0, 0)),
            replace(goal(), target=(float("nan"), 0)), Delay(-1),
            replace(goal(), weight=0),
            ProjectedDistanceGoal(id="edge", source_id="a", target_id="b", target=1, axis=(0, 0)),
        )
        for program in invalid:
            with self.subTest(program=program), self.assertRaises(ValueError):
                engine(program)

    def test_flyingwing_anchor_and_extension_use_expected_modes(self):
        program = Parallel((
            NodePositionGoal(id="anchor", node_id="car3", target=(.5, sqrt(3)/2),
                             hold_after_completion=True),
            ProjectedDistanceGoal(id="wing", source_id="car5", target_id="car6",
                                  axis=(0, 1), target=1.4),
        ))
        config = replace(CONFIG, subspace_control=replace(
            CONFIG.subspace_control, coefficients=(), task_feedback=TaskFeedbackConfig(program),
        ))
        config.validate()
        planner = build_swarm_planner(config)
        points = dict(zip(CONFIG.vehicle_ids, (
            (0, 0), (1, 0), (.5, sqrt(3)/2),
            (-1, -1), (-1, 0), (2, -1), (2, 0),
        )))
        result = planner.step(snapshot(points), 1)
        for name in ("car1", "car2", "car3", "car6", "car7", "car8"):
            self.assertAlmostEqual(result.commands[name].vx, 0)
            self.assertAlmostEqual(result.commands[name].vy, 0)
        self.assertAlmostEqual(result.commands["car5"].vy, -.2)
        self.assertAlmostEqual(result.task_feedback.coefficients[4], .2)

    def test_full_flyingwing_program_converges_in_kinematic_simulation(self):
        config = replace(CONFIG, subspace_control=replace(
            CONFIG.subspace_control, coefficients=(), task_feedback=TaskFeedbackConfig(PROGRAM),
        ))
        planner = build_swarm_planner(config)
        points = np.array(((0, 0), (.4, 0), (.2, sqrt(3)*.2),
                           (-.3, -.3), (-.3, 0), (.7, -.3), (.7, 0)))
        velocities = np.zeros((7, 2))
        for _ in range(3000):
            frame = snapshot(dict(zip(CONFIG.vehicle_ids, points)), dict(zip(CONFIG.vehicle_ids, velocities)))
            result = planner.step(frame, .02)
            self.assertNotEqual(result.task_feedback.status, "failed")
            velocities = np.array([(result.commands[name].vx, result.commands[name].vy)
                                   for name in CONFIG.vehicle_ids])
            points += .02 * velocities
            if result.task_feedback.status == "completed":
                break
        self.assertEqual(result.task_feedback.status, "completed")
        np.testing.assert_allclose(points[2], (0, 0), atol=.02)
        self.assertAlmostEqual(points[0, 0] - points[4, 0], .6, delta=.02)
        self.assertAlmostEqual(points[4, 1] - points[3, 1], .6, delta=.02)
        self.assertAlmostEqual(points[6, 1] - points[5, 1], .6, delta=.02)

    def test_time_input_is_mutually_exclusive_and_legacy_remains_valid(self):
        CONFIG.validate()
        invalid = replace(CONFIG, subspace_control=replace(
            CONFIG.subspace_control, task_feedback=TaskFeedbackConfig(PROGRAM),
        ))
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            invalid.validate()

    def test_metadata_distinguishes_program_nodes_and_tasks_are_recorded(self):
        encoded = json.loads(json.dumps(asdict(TaskFeedbackConfig(PROGRAM))))
        self.assertEqual(encoded["program"]["kind"], "sequence")
        self.assertEqual(encoded["program"]["children"][0]["kind"], "parallel")
        planner = build_swarm_planner(CONFIG)
        points = {name: (i, 1) for i, name in enumerate(CONFIG.vehicle_ids)}
        state = snapshot(points)
        result = planner.step(state, .02)
        result.task_feedback = step(engine(goal()))
        with TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            recorder = ExperimentRecorder(output, {}, 5, .1)
            recorder.emit(ControlFrame(0, .02, state, result, {}))
            recorder.close()
            record = json.loads((output / "tasks.jsonl").read_text())
            self.assertEqual(record["goals"][0]["goal_id"], "move")


if __name__ == "__main__":
    unittest.main()
