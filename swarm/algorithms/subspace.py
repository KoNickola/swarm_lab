"""Fixed matrix-weight subspace feedback and normalized edge metrics."""

import numpy as np

from swarm.domain.models import PlannerResult, SubspaceEdgeSample, VelocityCommand
from swarm.domain.subspace import (
    TOLERANCE,
    evaluate_task_coefficients,
    prepare_subspace,
)


def _vector3(vector):
    return tuple(float(x) for x in vector) + (0.0,) * (3 - len(vector))


class SubspaceFormationController:
    """Sum raw edge feedback; normalize only the reported performance error."""

    def __init__(self, vehicle_ids, topology, control, control_space,
                 command_speed_limit_mps):
        self.vehicle_ids = tuple(vehicle_ids)
        self.dimension, self.edges, self.modes = prepare_subspace(
            self.vehicle_ids, topology, control, control_space)
        if not np.isfinite(command_speed_limit_mps) or command_speed_limit_mps <= 0:
            raise ValueError("command speed limit must be positive and finite")
        self.control = control
        self.command_speed_limit_mps = command_speed_limit_mps
        self._elapsed_s = 0.0
        self.task_controller = None
        if control.task_feedback is not None:
            from swarm.algorithms.task_program import ModalTaskController

            self.task_controller = ModalTaskController(
                self.vehicle_ids, self.dimension, self.modes, control.task_feedback,
            )

    def reset(self):
        self._elapsed_s = 0.0
        if self.task_controller is not None:
            self.task_controller.reset()

    def pause_tasks(self):
        """Discard continuous task dwell evidence when mocap is unavailable."""
        if self.task_controller is not None:
            self.task_controller.pause()

    def step(self, snapshot, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")
        d, n = self.dimension, len(self.vehicle_ids)
        positions, valid = [], []
        for name in self.vehicle_ids:
            state = snapshot.states.get(name)
            position = np.array(state.position()[:d]) if state is not None else np.zeros(d)
            valid.append(state is not None and state.valid and bool(np.all(np.isfinite(position))))
            positions.append(position)
        result = PlannerResult(snapshot.timestamp, ready=all(valid), subspace_samples=[])
        residuals = np.zeros((n, d))
        errors = []
        for edge in self.edges:
            i, j = edge.source, edge.target
            relative = residual = None
            error, reason = None, ""
            if not (valid[i] and valid[j]):
                reason = "invalid_state"
            else:
                relative = positions[j] - positions[i]
                residual = edge.matrix @ (relative - edge.bias)
                if not np.all(np.isfinite(residual)):
                    raise ValueError("subspace residual must be finite")
                residuals[i] += residual
                residuals[j] -= residual
                length = float(np.linalg.norm(relative))
                if edge.matrix_norm == 0:
                    reason = "zero_matrix"
                elif length <= TOLERANCE:
                    reason = "zero_relative_length"
                else:
                    normalized = (edge.matrix / edge.matrix_norm) @ ((relative - edge.bias) / length)
                    error = float(np.linalg.norm(normalized))
                    if not np.isfinite(error):
                        raise ValueError("subspace edge error must be finite")
                    errors.append(error)
            result.subspace_samples.append(SubspaceEdgeSample(
                self.vehicle_ids[i], self.vehicle_ids[j],
                None if relative is None else _vector3(relative), _vector3(edge.bias),
                None if residual is None else _vector3(residual),
                edge.matrix_norm, error, reason))
        if self.task_controller is None:
            coefficients = evaluate_task_coefficients(
                self.control.coefficients,
                self.modes.shape[1],
                self._elapsed_s,
            )
        else:
            result.task_feedback = self.task_controller.step(
                snapshot, dt, self.control.k * residuals,
            )
            coefficients = result.task_feedback.coefficients
        task = (self.modes @ coefficients).reshape(n, d)
        velocities = self.control.k * residuals + task
        if not np.all(np.isfinite(velocities)):
            raise ValueError("subspace velocity must be finite")
        for i, name in enumerate(self.vehicle_ids):
            velocity = velocities[i] if valid[i] else np.zeros(d)
            speed = float(np.linalg.norm(velocity))
            if speed > self.command_speed_limit_mps:
                velocity = velocity * (self.command_speed_limit_mps / speed)
            result.commands[name] = VelocityCommand(
                name, *_vector3(velocity), error_vector=_vector3(residuals[i]), valid=valid[i])
        magnitudes = [float(np.linalg.norm(residuals[i])) for i in range(n) if valid[i]]
        result.mean_error = sum(magnitudes) / len(magnitudes) if magnitudes else 0.0
        result.max_error = max(magnitudes, default=0.0)
        result.subspace_valid_edges = len(errors)
        # Scaling avoids overflow when affine normalized errors are large.
        if errors:
            maximum = max(errors)
            result.subspace_edge_rms = (maximum * float(np.sqrt(np.mean(
                (np.array(errors) / maximum) ** 2))) if maximum else 0.0)
        if any(valid):
            self._elapsed_s += dt
        return result
