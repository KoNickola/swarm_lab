"""Ground-vehicle actuator combining control and UDP transport."""

from __future__ import annotations

from swarm.algorithms.vehicle import VehicleControllerType
from swarm.domain.models import ActuationResult, VehicleState, VelocityCommand
from swarm.infrastructure.udp_chassis import ChassisSocketDriver


class GroundVehicleActuator:
    """Translate a planner velocity into wheels and deliver it to one chassis."""

    platform = "ground_vehicle"

    def __init__(
        self,
        vehicle_id: str,
        controller: VehicleControllerType,
        driver: ChassisSocketDriver,
    ) -> None:
        self.vehicle_id = vehicle_id
        self.controller = controller
        self.driver = driver

    def execute(
        self,
        command: VelocityCommand,
        state: VehicleState,
        dt: float,
    ) -> ActuationResult:
        values = self.controller.step(
            target_vx=command.vx,
            target_vy=command.vy,
            target_vz=command.vz,
            yaw=state.yaw,
            measured_vx=state.vx,
            measured_vy=state.vy,
            measured_vz=state.vz,
            dt=dt,
        )
        sent = self.driver.send(values)
        return ActuationResult(self.vehicle_id, self.platform, values, sent)

    def stop(self) -> bool:
        return self.driver.send(self.controller.stop_command())
