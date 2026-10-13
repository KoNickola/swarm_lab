"""Build production adapters from a validated experiment configuration."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Mapping

from swarm.algorithms.obstacle_avoidance import ArtificialPotentialField
from swarm.algorithms.vehicle import build_vehicle_controller
from swarm.application.control_loop import ControlLoop
from swarm.application.interfaces import TelemetrySink
from swarm.application.planner_factory import build_swarm_planner
from swarm.domain.config import ExperimentConfig
from swarm.infrastructure.ground_vehicle import GroundVehicleActuator
from swarm.infrastructure.ros_mocap import RosMocapSource, rospy
from swarm.infrastructure.telemetry.composite import CompositeTelemetrySink
from swarm.infrastructure.telemetry.console import ConsoleTelemetrySink
from swarm.infrastructure.telemetry.live_plot import LiveTrajectoryPlotSink
from swarm.infrastructure.telemetry.recorder import ExperimentRecorder
from swarm.infrastructure.udp_chassis import ChassisSocketDriver, SocketConfig


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _run_directory(root: str, experiment_name: str) -> Path:
    root_path = Path(root)
    if not root_path.is_absolute():
        root_path = _project_root() / root_path
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return root_path / (experiment_name + "_" + timestamp)


def _metadata_value(value: Any) -> Any:
    """Convert callable configuration values into stable JSON metadata."""

    if callable(value):
        module = getattr(value, "__module__", "")
        qualified_name = getattr(value, "__qualname__", getattr(value, "__name__", "callable"))
        return "%s.%s" % (module, qualified_name) if module else qualified_name
    if isinstance(value, Mapping):
        return {str(key): _metadata_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_metadata_value(item) for item in value]
    return value


def _build_telemetry(config: ExperimentConfig) -> CompositeTelemetrySink:
    telemetry = config.runtime.telemetry
    sinks: List[TelemetrySink] = []
    if telemetry.console_enabled:
        sinks.append(ConsoleTelemetrySink(telemetry.console_hz))
    if telemetry.record_enabled:
        metadata: Dict[str, Any] = {
            "schema_version": 2,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "experiment": _metadata_value(asdict(config)),
        }
        recorder = ExperimentRecorder(
            run_directory=_run_directory(telemetry.record_root, config.name),
            metadata=metadata,
            record_hz=telemetry.record_hz,
            flush_interval_s=telemetry.record_flush_interval_s,
        )
        print("recording | %s" % recorder.run_directory)
        sinks.append(recorder)
    if telemetry.live_plot_enabled:
        sinks.append(
            LiveTrajectoryPlotSink(
                vehicle_ids=config.vehicle_ids,
                plot_hz=telemetry.live_plot_hz,
                queue_size=telemetry.live_plot_queue_size,
                max_points=telemetry.live_plot_max_points,
            )
        )
    return CompositeTelemetrySink(sinks)


def build_control_loop(config: ExperimentConfig) -> ControlLoop:
    """Construct the selected swarm planner and ground-vehicle runtime."""

    config.validate()
    planner = build_swarm_planner(config)
    if rospy is None:
        raise ImportError("rospy is required to run the experiment")
    rospy.init_node("swarm_experiment", anonymous=True)

    velocity_filter = ArtificialPotentialField(
        config=config.obstacle_avoidance,
        command_speed_limit_mps=config.runtime.command_speed_limit_mps,
        static_obstacles=config.static_obstacles,
    )
    state_source = RosMocapSource(config.vehicle_ids, config.runtime.mocap_prefix)
    socket_config = SocketConfig(
        port=config.runtime.udp_port,
        timeout_s=config.runtime.udp_timeout_s,
    )
    actuators = {}
    for vehicle in config.vehicles:
        if vehicle.platform != "ground_vehicle":
            raise ValueError("no actuator factory registered for platform: %s" % vehicle.platform)
        actuators[vehicle.vehicle_id] = GroundVehicleActuator(
            vehicle_id=vehicle.vehicle_id,
            controller=build_vehicle_controller(config.ground_vehicle_control),
            driver=ChassisSocketDriver(vehicle.address, socket_config),
        )
    return ControlLoop(
        control_hz=config.runtime.control_hz,
        stop_on_loss_of_mocap=config.runtime.stop_on_loss_of_mocap,
        state_source=state_source,
        planner=planner,
        actuators=actuators,
        telemetry=_build_telemetry(config),
        shutdown_requested=rospy.is_shutdown,
        velocity_filter=velocity_filter,
        control_space=config.control_space,
        duration_s=config.runtime.duration_s,
    )
