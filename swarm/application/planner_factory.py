"""Construct swarm planners selected by experiment configuration."""

from __future__ import annotations

from typing import Callable, Dict, Tuple

from swarm.algorithms.bearing import (
    BearingOnlyFormationController,
    build_topology_from_matrices,
)
from swarm.application.interfaces import SwarmPlanner
from swarm.domain.config import ExperimentConfig


PlannerBuilder = Callable[[ExperimentConfig], SwarmPlanner]


def _build_bearing_planner(config: ExperimentConfig) -> SwarmPlanner:
    topology = build_topology_from_matrices(
        vehicle_ids=config.vehicle_ids,
        adjacency_matrix=config.topology.adjacency_matrix,
        bearing_matrix=config.topology.bearing_matrix,
        leader_mask=config.topology.leader_mask,
    )
    return BearingOnlyFormationController(
        topology=topology,
        leader_target_velocities=config.leader_target_velocities,
        control=config.formation_control,
        command_speed_limit_mps=config.runtime.command_speed_limit_mps,
    )


# Each selectable planner has one builder registered under its config value.
_PLANNER_BUILDERS: Dict[str, PlannerBuilder] = {
    "bearing": _build_bearing_planner,
}


def available_algorithms() -> Tuple[str, ...]:
    """Return the algorithm names accepted by :func:`build_swarm_planner`."""

    return tuple(sorted(_PLANNER_BUILDERS))


def build_swarm_planner(config: ExperimentConfig) -> SwarmPlanner:
    """Build the planner selected by ``config.algorithm``."""

    try:
        builder = _PLANNER_BUILDERS[config.algorithm]
    except KeyError:
        supported = ", ".join(available_algorithms())
        raise ValueError(
            "unsupported swarm algorithm %r; available algorithms: %s"
            % (config.algorithm, supported)
        ) from None
    return builder(config)
