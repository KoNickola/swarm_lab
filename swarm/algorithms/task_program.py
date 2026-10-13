"""Closed-loop task scheduling and weighted allocation in configured modes."""

import numpy as np

from swarm.domain.tasks import (
    Delay, Goal, Hold, Parallel, Release, Sequence,
    TaskFeedbackResult, TaskGoalSample, goal_measurement, validate_task_feedback,
)


class _Node:
    def __init__(self, definition, path="root"):
        self.definition = definition
        self.path = path
        self.children = [
            _Node(child, "%s/%d" % (path, i))
            for i, child in enumerate(getattr(definition, "children", ()))
        ]
        self.index = 0
        self.started = False
        self.done = False
        self.elapsed = 0.0
        self.stable = 0.0
        self.hold_target = None

    def leaves(self):
        if self.done:
            return []
        if isinstance(self.definition, Sequence):
            return self.children[self.index].leaves()
        if isinstance(self.definition, Parallel):
            return [leaf for child in self.children for leaf in child.leaves()]
        return [self]

    def clear_stability(self):
        self.stable = 0.0
        for child in self.children:
            child.clear_stability()


def _limited(vector, limit):
    norm = float(np.linalg.norm(vector))
    return vector * min(1.0, limit / norm) if norm else vector


class ModalTaskController:
    """Execute a task tree; all active goals share one modal least-squares solve."""

    def __init__(self, vehicle_ids, dimension, modes, config):
        self.vehicle_ids = tuple(vehicle_ids)
        self.dimension = dimension
        self.modes = np.asarray(modes, dtype=float)
        validate_task_feedback(config, self.vehicle_ids, dimension, self.modes.shape[1])
        self.config = config
        self.mode_inverse = np.linalg.pinv(self.modes, rcond=1e-9)
        self.reset()

    def reset(self):
        self.root = _Node(self.config.program)
        self.held = {}
        self.previous = np.zeros(self.modes.shape[1])
        self.elapsed = 0.0
        self.failure = ""
        self.failure_target = None

    def pause(self):
        """Freeze program progress and discard continuous stability evidence."""
        self.root.clear_stability()
        self.previous[:] = 0.0

    def _result(self, status, active=(), samples=(), reason="", rank=0, residual=0.0):
        return TaskFeedbackResult(
            tuple(float(x) for x in self.previous), status,
            tuple(active), tuple(samples), self.elapsed, reason, rank, residual,
        )

    def _hold_equation(self, target, positions):
        error = target - self.mode_inverse @ positions.ravel()
        return error, self.mode_inverse, _limited(0.5 * error, self.config.mode_speed_limit), 1.0

    def _allocate(self, equations, formation_velocity, dt):
        if not equations:
            self.previous[:] = 0.0
            return 0, 0.0
        matrices, references = [], []
        for _, jacobian, reference, weight in equations:
            scale = np.sqrt(weight)
            matrices.append(scale * (jacobian @ self.modes))
            references.append(scale * (reference - jacobian @ formation_velocity))
        matrix, reference = np.vstack(matrices), np.concatenate(references)
        m = self.modes.shape[1]
        augmented = np.vstack((matrix, self.config.regularization * np.eye(m)))
        desired, *_ = np.linalg.lstsq(
            augmented, np.concatenate((reference, np.zeros(m))), rcond=None,
        )
        # A common factor preserves the allocation ratios across modes.
        peak = float(np.max(np.abs(desired)))
        if peak > self.config.mode_speed_limit:
            desired *= self.config.mode_speed_limit / peak
        change = desired - self.previous
        peak_change = float(np.max(np.abs(change)))
        limit = self.config.mode_acceleration_limit * dt
        if peak_change > limit:
            change *= limit / peak_change
        self.previous += change
        return (int(np.linalg.matrix_rank(matrix, tol=1e-9)),
                float(np.linalg.norm(matrix @ self.previous - reference)))

    def _advance(self, node, evaluations):
        if node.done:
            return
        if isinstance(node.definition, Sequence):
            child = node.children[node.index]
            self._advance(child, evaluations)
            if child.done:
                node.index += 1
                node.done = node.index == len(node.children)
        elif isinstance(node.definition, Parallel):
            for child in node.children:
                self._advance(child, evaluations)
            held_ready = all(
                evaluations[item.path][-1]
                for item in self.held.values()
                if item.path.startswith(node.path + "/")
            )
            node.done = all(child.done for child in node.children) and held_ready

    def step(self, snapshot, dt, formation_velocity):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("task dt must be positive and finite")
        positions, velocities = [], []
        for name in self.vehicle_ids:
            state = snapshot.states.get(name)
            if state is None or not state.valid:
                self.pause()
                return self._result("waiting", reason="invalid mocap: " + name)
            positions.append(state.position()[:self.dimension])
            velocities.append((state.vx, state.vy, state.vz)[:self.dimension])
        positions, velocities = np.asarray(positions), np.asarray(velocities).ravel()
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(velocities)):
            self.pause()
            return self._result("waiting", reason="nonfinite task state")
        formation_velocity = np.asarray(formation_velocity, dtype=float).ravel()
        if formation_velocity.shape != positions.ravel().shape or not np.all(np.isfinite(formation_velocity)):
            raise ValueError("formation velocity must match finite task coordinates")
        if self.failure:
            rank, residual = self._allocate(
                [self._hold_equation(self.failure_target, positions)], formation_velocity, dt,
            )
            return self._result("failed", reason=self.failure, rank=rank, residual=residual)

        leaves = self.root.leaves()
        for node in leaves:
            if node.started:
                continue
            node.started = True
            definition = node.definition
            if isinstance(definition, Goal):
                self.held.pop(definition.id, None)
            elif isinstance(definition, Release):
                for goal_id in definition.goal_ids:
                    self.held.pop(goal_id, None)
            elif isinstance(definition, Hold):
                node.hold_target = self.mode_inverse @ positions.ravel()
                for held in self.held.values():
                    held.stable = 0.0

        hold = next((node for node in leaves if isinstance(node.definition, Hold)), None)
        goals = ([] if hold else list(self.held.values()) +
                 [node for node in leaves if isinstance(node.definition, Goal)])
        evaluations, equations = {}, []
        # Evaluate all active goals before committing any sequence transitions.
        try:
            for node in goals:
                goal = node.definition
                value, jacobian = goal_measurement(
                    goal, self.vehicle_ids, self.dimension, positions,
                )
                error = np.atleast_1d(goal.target) - value
                rate = jacobian @ velocities
                within = (np.linalg.norm(error) <= goal.tolerance and
                          np.linalg.norm(rate) <= goal.speed_tolerance)
                evaluations[node.path] = (error, rate, within)
                equations.append((error, jacobian, _limited(goal.gain * error, goal.max_rate), goal.weight))
        except ValueError as error:
            self.pause()
            return self._result("waiting", reason=str(error))

        self.elapsed += dt
        for node in goals:
            error, rate, within = evaluations[node.path]
            node.stable = node.stable + dt if within else 0.0
            evaluations[node.path] = (error, rate, node.stable + 1e-12 >= node.definition.settle_s)

        for node in leaves:
            definition = node.definition
            node.elapsed += dt
            if isinstance(definition, Goal):
                node.done = evaluations[node.path][-1]
                if node.done and definition.hold_after_completion:
                    self.held[definition.id] = node
            elif isinstance(definition, Delay):
                node.done = node.elapsed + 1e-12 >= definition.seconds
            elif isinstance(definition, Release):
                node.done = True
            elif isinstance(definition, Hold):
                equation = self._hold_equation(node.hold_target, positions)
                equations = [equation]
                vehicle_speeds = np.linalg.norm(
                    velocities.reshape(len(self.vehicle_ids), self.dimension), axis=1,
                )
                stable = (np.max(np.abs(equation[0])) <= definition.tolerance and
                          np.max(vehicle_speeds) <= definition.speed_tolerance)
                node.stable = node.stable + dt if stable else 0.0
                node.done = node.stable + 1e-12 >= definition.seconds
            timeout = getattr(definition, "timeout_s", None)
            if timeout is not None and not node.done and node.elapsed >= timeout:
                self.failure = "task timeout: %s" % getattr(definition, "id", node.path)
                self.failure_target = self.mode_inverse @ positions.ravel()
                equations = [self._hold_equation(self.failure_target, positions)]
                break

        if not self.failure:
            self._advance(self.root, evaluations)
        samples = []
        for node in goals:
            error, rate, settled = evaluations[node.path]
            goal = node.definition
            state = ("holding" if node.done and goal.hold_after_completion else
                     "succeeded" if node.done else "running")
            samples.append(TaskGoalSample(
                goal.id, state, tuple(float(x) for x in error),
                tuple(float(x) for x in rate), settled, node.elapsed, node.stable,
            ))
        rank, residual = self._allocate(equations, formation_velocity, dt)
        active = [getattr(node.definition, "id", "%s:%s" % (node.path, type(node.definition).__name__))
                  for node in leaves]
        status = "failed" if self.failure else "completed" if self.root.done else "running"
        return self._result(status, active, samples, self.failure, rank, residual)
