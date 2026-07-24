"""Planar artificial-potential-field velocity filtering."""

from __future__ import annotations

from dataclasses import replace
from math import sqrt
from typing import Sequence, Tuple

from swarm.domain.config import ObstacleAvoidanceConfig, StaticObstacleConfig
from swarm.domain.models import MocapSnapshot, PlannerResult, VehicleState


class ArtificialPotentialField:
    """Add circular-obstacle repulsion to formation velocity commands."""

    def __init__(
        self,
        config: ObstacleAvoidanceConfig,
        command_speed_limit_mps: float,
        static_obstacles: Sequence[StaticObstacleConfig] = (),
    ) -> None:
        if command_speed_limit_mps <= 0.0:
            raise ValueError("command_speed_limit_mps must be positive")
        self.config = config
        self.command_speed_limit_mps = command_speed_limit_mps
        self.static_obstacles = tuple(static_obstacles)

    def _circle_repulsion(
        self,
        state: VehicleState,
        obstacle_x: float,
        obstacle_y: float,
        obstacle_radius: float,
    ) -> Tuple[float, float]:
        dx = state.x - obstacle_x
        dy = state.y - obstacle_y
        distance_squared = dx * dx + dy * dy
        combined_radius = self.config.vehicle_radius_m + obstacle_radius
        activation_radius = combined_radius + self.config.influence_distance_m
        if distance_squared >= activation_radius * activation_radius:
            return 0.0, 0.0

        center_distance = sqrt(distance_squared)
        if center_distance < 1e-12:
            unit_x, unit_y = 1.0, 0.0
        else:
            unit_x = dx / center_distance
            unit_y = dy / center_distance

        clearance = max(
            center_distance - combined_radius,
            self.config.min_distance_epsilon_m,
        )
        inverse_clearance = 1.0 / clearance
        inverse_influence = 1.0 / self.config.influence_distance_m
        magnitude = (
            self.config.repulsive_gain
            * (inverse_clearance - inverse_influence)
            * inverse_clearance
            * inverse_clearance
        )
        return magnitude * unit_x, magnitude * unit_y

    def _limit_planar_speed(self, vx: float, vy: float) -> Tuple[float, float]:
        speed_squared = vx * vx + vy * vy
        max_speed = self.command_speed_limit_mps
        if speed_squared <= max_speed * max_speed:
            return vx, vy
        scale = max_speed / sqrt(speed_squared)
        return vx * scale, vy * scale

    def apply(self, snapshot: MocapSnapshot, nominal: PlannerResult) -> PlannerResult:
        """Return velocity-filtered commands without changing the nominal result."""

        if not self.config.enabled:
            return nominal

        filtered_commands = None
        for vehicle_id, command in nominal.commands.items():
            state = snapshot.states.get(vehicle_id)
            if state is None or not state.valid or not command.valid:
                continue

            repulsive_x = 0.0
            repulsive_y = 0.0
            for obstacle in self.static_obstacles:
                contribution_x, contribution_y = self._circle_repulsion(
                    state,
                    obstacle.x,
                    obstacle.y,
                    obstacle.radius_m,
                )
                repulsive_x += contribution_x
                repulsive_y += contribution_y

            if self.config.avoid_other_vehicles:
                for other_id, other_state in snapshot.states.items():
                    if other_id == vehicle_id or not other_state.valid:
                        continue
                    contribution_x, contribution_y = self._circle_repulsion(
                        state,
                        other_state.x,
                        other_state.y,
                        self.config.vehicle_radius_m,
                    )
                    repulsive_x += contribution_x
                    repulsive_y += contribution_y

            filtered_vx, filtered_vy = self._limit_planar_speed(
                command.vx + repulsive_x,
                command.vy + repulsive_y,
            )
            if filtered_vx == command.vx and filtered_vy == command.vy:
                continue
            if filtered_commands is None:
                filtered_commands = dict(nominal.commands)
            filtered_commands[vehicle_id] = replace(
                command,
                vx=filtered_vx,
                vy=filtered_vy,
            )

        if filtered_commands is None:
            return nominal
        return replace(nominal, commands=filtered_commands)
