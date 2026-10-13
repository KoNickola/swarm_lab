"""Immutable task goals, program nodes, and feedback diagnostics."""

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class Goal:
    id: str
    gain: float = 0.5
    weight: float = 1.0
    tolerance: float = 0.2
    speed_tolerance: float = 0.02
    settle_s: float = 1.0
    max_rate: float = 0.2
    timeout_s: Optional[float] = 60.0
    hold_after_completion: bool = False


@dataclass(frozen=True)
class NodePositionGoal(Goal):
    kind: str = field(default="node_position", init=False)
    node_id: str = ""
    target: Tuple[float, ...] = ()
    # Empty axes selects every coordinate in the configured control space.
    axes: Tuple[int, ...] = ()


@dataclass(frozen=True)
class RelativePositionGoal(Goal):
    kind: str = field(default="relative_position", init=False)
    source_id: str = ""
    target_id: str = ""
    target: Tuple[float, ...] = ()


@dataclass(frozen=True)
class EdgeLengthGoal(Goal):
    kind: str = field(default="edge_length", init=False)
    source_id: str = ""
    target_id: str = ""
    target: float = 0.0


@dataclass(frozen=True)
class ProjectedDistanceGoal(Goal):
    kind: str = field(default="projected_distance", init=False)
    source_id: str = ""
    target_id: str = ""
    target: float = 0.0
    axis: Tuple[float, ...] = ()


@dataclass(frozen=True)
class Sequence:
    children: Tuple[object, ...]
    kind: str = field(default="sequence", init=False)


@dataclass(frozen=True)
class Parallel:
    children: Tuple[object, ...]
    kind: str = field(default="parallel", init=False)


@dataclass(frozen=True)
class Delay:
    seconds: float
    kind: str = field(default="delay", init=False)


@dataclass(frozen=True)
class Hold:
    seconds: float
    kind: str = field(default="hold", init=False)
    tolerance: float = 0.02
    speed_tolerance: float = 0.02
    timeout_s: Optional[float] = 60.0


@dataclass(frozen=True)
class Release:
    goal_ids: Tuple[str, ...]
    kind: str = field(default="release", init=False)


@dataclass(frozen=True)
class TaskFeedbackConfig:
    program: object
    mode_speed_limit: float = 0.2
    mode_acceleration_limit: float = 0.8
    regularization: float = 1e-6


@dataclass(frozen=True)
class TaskGoalSample:
    goal_id: str
    state: str
    error: Tuple[float, ...]
    measured_rate: Tuple[float, ...]
    settled: bool
    elapsed_s: float
    stable_s: float


@dataclass(frozen=True)
class TaskFeedbackResult:
    coefficients: Tuple[float, ...]
    status: str
    active_nodes: Tuple[str, ...] = ()
    goals: Tuple[TaskGoalSample, ...] = ()
    elapsed_s: float = 0.0
    reason: str = ""
    allocation_rank: int = 0
    allocation_residual: float = 0.0


def goal_measurement(goal, vehicle_ids, dimension, positions):
    """Return the observed goal value and its position Jacobian."""

    n, d = len(vehicle_ids), dimension
    if isinstance(goal, NodePositionGoal):
        axes = goal.axes or tuple(range(d))
        index = vehicle_ids.index(goal.node_id)
        jacobian = np.zeros((len(axes), n * d))
        for row, axis in enumerate(axes):
            jacobian[row, index * d + axis] = 1.0
        return positions[index, list(axes)], jacobian
    i, j = vehicle_ids.index(goal.source_id), vehicle_ids.index(goal.target_id)
    relative = positions[j] - positions[i]
    if isinstance(goal, RelativePositionGoal):
        derivative, value = np.eye(d), relative
    elif isinstance(goal, ProjectedDistanceGoal):
        axis = np.asarray(goal.axis, dtype=float)
        derivative = (axis / np.linalg.norm(axis)).reshape(1, d)
        value = derivative @ relative
    elif isinstance(goal, EdgeLengthGoal):
        length = float(np.linalg.norm(relative))
        if length <= 1e-9:
            raise ValueError("%s: edge length derivative is undefined at coincident nodes" % goal.id)
        derivative, value = (relative / length).reshape(1, d), np.array([length])
    else:
        raise TypeError("unsupported goal type")
    jacobian = np.zeros((len(value), n * d))
    jacobian[:, i*d:(i+1)*d] = -derivative
    jacobian[:, j*d:(j+1)*d] = derivative
    return value, jacobian


def validate_task_feedback(config, vehicle_ids, dimension, mode_count):
    """Validate program structure and references without evaluating runtime state."""

    def positive(value, name):
        if not np.isfinite(value) or value <= 0:
            raise ValueError("%s must be positive and finite" % name)

    def vector(value, count, name):
        array = np.asarray(value, dtype=float)
        if array.shape != (count,) or not np.all(np.isfinite(array)):
            raise ValueError("%s must contain %d finite coordinates" % (name, count))
        return array

    def timeout(value):
        if value is not None:
            positive(value, "timeout_s")

    def visit(node, in_parallel=False):
        if isinstance(node, (Sequence, Parallel)):
            if not node.children:
                raise ValueError("task sequences and parallel groups must not be empty")
            ids = set()
            for child in node.children:
                child_ids = visit(child, in_parallel or isinstance(node, Parallel))
                if isinstance(node, Parallel) and ids.intersection(child_ids):
                    raise ValueError("parallel branches must use distinct goal IDs")
                ids.update(child_ids)
            return ids
        if isinstance(node, (Hold, Delay)):
            positive(node.seconds, "seconds")
            if isinstance(node, Hold):
                if in_parallel:
                    raise ValueError("Hold cannot run inside Parallel")
                positive(node.tolerance, "tolerance")
                positive(node.speed_tolerance, "speed_tolerance")
                timeout(node.timeout_s)
            return set()
        if isinstance(node, Release):
            if not node.goal_ids or any(not isinstance(x, str) or not x for x in node.goal_ids):
                raise ValueError("Release requires nonempty goal IDs")
            return set(node.goal_ids)
        if not isinstance(node, (NodePositionGoal, RelativePositionGoal,
                                 EdgeLengthGoal, ProjectedDistanceGoal)):
            raise ValueError("unsupported task node: %s" % type(node).__name__)
        if not isinstance(node.id, str) or not node.id:
            raise ValueError("goal id must be a nonempty string")
        for name in ("gain", "weight", "tolerance", "speed_tolerance", "settle_s", "max_rate"):
            positive(getattr(node, name), name)
        timeout(node.timeout_s)
        if not isinstance(node.hold_after_completion, bool):
            raise ValueError("hold_after_completion must be boolean")
        if isinstance(node, NodePositionGoal):
            if node.node_id not in vehicle_ids:
                raise ValueError("unknown task node: %s" % node.node_id)
            axes = node.axes or tuple(range(dimension))
            if (len(set(axes)) != len(axes) or
                    any(not isinstance(a, int) or a < 0 or a >= dimension for a in axes)):
                raise ValueError("goal axes must be distinct valid coordinate indices")
            vector(node.target, len(axes), "position target")
        else:
            if node.source_id not in vehicle_ids or node.target_id not in vehicle_ids:
                raise ValueError("goal references unknown vehicles")
            if node.source_id == node.target_id:
                raise ValueError("goal endpoints must be distinct")
            if isinstance(node, RelativePositionGoal):
                vector(node.target, dimension, "relative target")
            else:
                vector((node.target,), 1, "distance target")
                if isinstance(node, EdgeLengthGoal):
                    positive(node.target, "edge length target")
                else:
                    axis = vector(node.axis, dimension, "projection axis")
                    if np.linalg.norm(axis) <= 1e-9:
                        raise ValueError("projection axis must be nonzero")
        return {node.id}

    if not isinstance(config, TaskFeedbackConfig) or mode_count <= 0:
        raise ValueError("task feedback requires a TaskFeedbackConfig and nonempty modes")
    for name in ("mode_speed_limit", "mode_acceleration_limit", "regularization"):
        positive(getattr(config, name), name)
    visit(config.program)
