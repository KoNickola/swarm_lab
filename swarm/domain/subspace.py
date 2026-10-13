"""Validate fixed matrix-weight constraints and prepare numerical arrays."""

from dataclasses import dataclass

import numpy as np

from swarm.domain.config import SubspaceTopologyConfig

TOLERANCE = 1e-9


def finite_array(value, name):
    """Convert numeric input without silently accepting nonfinite entries."""
    try:
        array = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError):
        raise ValueError("%s must be a rectangular numeric array" % name) from None
    if not np.all(np.isfinite(array)):
        raise ValueError("%s must contain finite values" % name)
    return array


def projection_from_free_basis(basis):
    """Return I-QQ^T for independent basis columns (shape d by q)."""
    basis = finite_array(basis, "free basis")
    if basis.ndim != 2 or basis.shape[0] not in (2, 3):
        raise ValueError("free basis must have two or three rows")
    if np.linalg.matrix_rank(basis, tol=TOLERANCE) != basis.shape[1]:
        raise ValueError("free basis columns must be linearly independent")
    q, _ = np.linalg.qr(basis, mode="reduced")
    matrix = np.eye(basis.shape[0]) - q @ q.T
    return tuple(tuple(float(x) for x in row) for row in matrix)


def matrix_from_expected_points(points):
    """Build the constrained projection from two expected relative points."""

    if points is None or len(points) != 2:
        raise ValueError("expected_points must contain two points")
    first = finite_array(points[0], "expected source point")
    second = finite_array(points[1], "expected target point")
    if first.ndim != 1 or second.ndim != 1 or first.shape != second.shape:
        raise ValueError("expected points must be equal-length vectors")
    direction = second - first
    if direction.size not in (2, 3) or np.linalg.norm(direction) <= TOLERANCE:
        raise ValueError("expected points must define a nonzero 2D or 3D direction")
    return np.array(projection_from_free_basis(direction.reshape(-1, 1)))


def matrix_from_free_direction(direction):
    """Build the constrained projection from one free-subspace vector."""

    vector = finite_array(direction, "free direction")
    if vector.ndim != 1 or vector.size not in (2, 3):
        raise ValueError("free direction must be a 2D or 3D vector")
    if np.linalg.norm(vector) <= TOLERANCE:
        raise ValueError("free direction must be nonzero")
    return np.array(projection_from_free_basis(vector.reshape(-1, 1)))


def task_coefficients(value, count):
    coefficients = finite_array(value, "task coefficients")
    if coefficients.shape != (count,):
        raise ValueError("task coefficients must match mode count")
    return coefficients


def evaluate_task_coefficients(value, count, time_s):
    """Evaluate one vector function or one independent function per mode."""

    if callable(value):
        evaluated = value(time_s)
    else:
        if len(value) != count:
            raise ValueError("task coefficients must match mode count")
        evaluated = tuple(
            item(time_s) if callable(item) else item
            for item in value
        )
    coefficients = finite_array(evaluated, "task coefficients")
    if coefficients.shape != (count,):
        raise ValueError("task coefficients must match mode count")
    return coefficients


def prepare_modes(value, vehicle_count, dimension):
    """Normalize grouped modes or the legacy stacked matrix representation."""

    modes = finite_array(value, "task modes")
    if modes.ndim == 3:
        if modes.shape[1:] != (vehicle_count, dimension):
            raise ValueError(
                "grouped task modes must have shape "
                "(mode_count, vehicle_count, dimension)"
            )
        # Grouped input is easier to edit: modes[mode][vehicle][axis].
        return modes.transpose(1, 2, 0).reshape(vehicle_count * dimension, modes.shape[0])
    if modes.ndim == 2 and modes.shape[0] == vehicle_count * dimension:
        return modes
    raise ValueError(
        "task modes must be grouped by mode or have "
        "vehicle_count * dimension rows"
    )


@dataclass(frozen=True)
class PreparedEdge:
    source: int
    target: int
    matrix: np.ndarray
    bias: np.ndarray
    matrix_norm: float


def prepare_subspace(vehicle_ids, topology, control, control_space):
    """Check edge feasibility and task modes before resources are connected."""
    if not isinstance(topology, SubspaceTopologyConfig):
        raise ValueError("subspace requires SubspaceTopologyConfig")
    if control_space not in ("planar_xy", "spatial_xyz"):
        raise ValueError("unsupported control_space")
    d = 2 if control_space == "planar_xy" else 3
    n = len(vehicle_ids)
    if not n or len(set(vehicle_ids)) != n:
        raise ValueError("vehicle IDs must be nonempty and unique")
    if not np.isfinite(control.k) or control.k <= 0:
        raise ValueError("subspace k must be positive and finite")
    indices = {name: i for i, name in enumerate(vehicle_ids)}
    seen, edges, rows, offsets = set(), [], [], []
    for edge in topology.edges:
        if edge.source_id not in indices or edge.target_id not in indices:
            raise ValueError("subspace edge references unknown vehicle")
        i, j = indices[edge.source_id], indices[edge.target_id]
        if i == j:
            raise ValueError("subspace self edges are not allowed")
        pair = tuple(sorted((i, j)))
        if pair in seen:
            raise ValueError("duplicate undirected subspace edge")
        seen.add(pair)
        if edge.matrix is None:
            if edge.free_direction is not None:
                matrix = matrix_from_free_direction(edge.free_direction)
            elif edge.expected_points is not None:
                matrix = matrix_from_expected_points(edge.expected_points)
            else:
                raise ValueError(
                    "subspace edge requires matrix, free_direction, or expected_points"
                )
        else:
            matrix = finite_array(edge.matrix, "edge matrix")
        bias = np.zeros(d) if edge.bias is None else finite_array(edge.bias, "edge bias")
        if matrix.shape != (d, d) or bias.shape != (d,):
            raise ValueError("edge matrix/bias dimensions must match control space")
        if not np.allclose(matrix, matrix.T, atol=TOLERANCE, rtol=0):
            raise ValueError("edge matrix must be symmetric")
        if np.min(np.linalg.eigvalsh(matrix)) < -TOLERANCE:
            raise ValueError("edge matrix must be positive semidefinite")
        edges.append(PreparedEdge(i, j, matrix, bias, float(np.linalg.norm(matrix, 2))))
        row = np.zeros((d, n * d))
        row[:, i*d:(i+1)*d] = -matrix
        row[:, j*d:(j+1)*d] = matrix
        rows.append(row)
        offsets.append(matrix @ bias)
    constraints = np.vstack(rows) if rows else np.zeros((0, n*d))
    rhs = np.concatenate(offsets) if offsets else np.zeros(0)
    if rows:
        feasible, *_ = np.linalg.lstsq(constraints, rhs, rcond=TOLERANCE)
        if not np.allclose(constraints @ feasible, rhs, atol=TOLERANCE, rtol=0):
            raise ValueError("subspace biases are not jointly feasible")
    modes = (np.zeros((n*d, 0)) if control.modes is None
             else prepare_modes(control.modes, n, d))
    # For symmetric PSD weights, edgewise nullspace equals null(L_A).
    if not np.allclose(constraints @ modes, 0, atol=TOLERANCE, rtol=0):
        raise ValueError("task modes must lie in the constraint nullspace")
    if control.task_feedback is not None:
        from swarm.domain.tasks import validate_task_feedback

        if callable(control.coefficients) or len(control.coefficients):
            raise ValueError("task_feedback and time coefficients are mutually exclusive")
        validate_task_feedback(control.task_feedback, vehicle_ids, d, modes.shape[1])
    elif callable(control.coefficients):
        if modes.shape[1] == 0:
            raise ValueError("task coefficient function requires modes")
    else:
        if len(control.coefficients) != modes.shape[1]:
            raise ValueError("task coefficients must match mode count")
        for coefficient in control.coefficients:
            if not callable(coefficient):
                task_coefficients((coefficient,), 1)
    return d, tuple(edges), modes
