"""Bearing performance metrics derived without changing formation control."""

from __future__ import annotations

from math import atan2, sqrt
from typing import Optional

from swarm.domain.models import BearingSample, Vector3


def _norm(vector: Vector3) -> float:
    return sqrt(sum(component * component for component in vector))


def bearing_angle_error_rad(sample: BearingSample) -> Optional[float]:
    """Return the unsigned 3D angle between desired and measured edge bearings."""

    desired_norm = _norm(sample.desired)
    actual_norm = _norm(sample.actual)
    norm_product = desired_norm * actual_norm
    if norm_product < 1e-12:
        return None

    desired_x, desired_y, desired_z = sample.desired
    actual_x, actual_y, actual_z = sample.actual
    dot = (
        desired_x * actual_x
        + desired_y * actual_y
        + desired_z * actual_z
    ) / norm_product
    cross_x = desired_y * actual_z - desired_z * actual_y
    cross_y = desired_z * actual_x - desired_x * actual_z
    cross_z = desired_x * actual_y - desired_y * actual_x
    cross_norm = sqrt(
        cross_x * cross_x + cross_y * cross_y + cross_z * cross_z
    ) / norm_product
    return atan2(cross_norm, max(-1.0, min(1.0, dot)))
