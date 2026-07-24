"""ROS1 adapter for VRPN pose and twist topics."""

from __future__ import annotations

from dataclasses import replace
from math import atan2
from threading import RLock
from typing import Dict, Sequence

from swarm.domain.models import MocapSnapshot, VehicleState

try:
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
except ImportError:  # pragma: no cover - permits tests outside ROS
    rospy = None  # type: ignore
    PoseStamped = None  # type: ignore
    TwistStamped = None  # type: ignore


class RosMocapSource:
    """Merge the latest VRPN pose and twist for every configured vehicle."""

    def __init__(self, vehicle_ids: Sequence[str], topic_prefix: str, queue_size: int = 10) -> None:
        if rospy is None:
            raise ImportError("rospy is required to use RosMocapSource")
        if not vehicle_ids:
            raise ValueError("vehicle_ids must not be empty")
        self.vehicle_ids = tuple(vehicle_ids)
        self.topic_prefix = topic_prefix.rstrip("/")
        self.queue_size = queue_size
        self._lock = RLock()
        self._subscribers: Dict[str, object] = {}
        self._states = {vehicle_id: VehicleState(vehicle_id) for vehicle_id in self.vehicle_ids}
        self._latest_timestamp = 0.0

    def start(self) -> None:
        with self._lock:
            if self._subscribers:
                return
            for vehicle_id in self.vehicle_ids:
                self._subscribers[vehicle_id + "_pose"] = rospy.Subscriber(
                    "%s/%s/pose" % (self.topic_prefix, vehicle_id),
                    PoseStamped,
                    callback=lambda message, vid=vehicle_id: self._handle_pose(vid, message),
                    queue_size=self.queue_size,
                )
                self._subscribers[vehicle_id + "_twist"] = rospy.Subscriber(
                    "%s/%s/twist" % (self.topic_prefix, vehicle_id),
                    TwistStamped,
                    callback=lambda message, vid=vehicle_id: self._handle_twist(vid, message),
                    queue_size=self.queue_size,
                )

    def stop(self) -> None:
        with self._lock:
            for subscriber in self._subscribers.values():
                try:
                    subscriber.unregister()
                except Exception:
                    pass
            self._subscribers.clear()

    @staticmethod
    def _stamp_to_sec(stamp: object) -> float:
        if hasattr(stamp, "to_sec"):
            return float(stamp.to_sec())
        seconds = getattr(stamp, "secs", None)
        nanoseconds = getattr(stamp, "nsecs", None)
        if seconds is not None and nanoseconds is not None:
            return float(seconds) + float(nanoseconds) * 1e-9
        return float(stamp)

    @staticmethod
    def _yaw_from_quaternion(orientation: object) -> float:
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return atan2(siny_cosp, cosy_cosp)

    def _handle_pose(self, vehicle_id: str, message: object) -> None:
        position = message.pose.position
        with self._lock:
            state = self._states[vehicle_id]
            state.x = float(position.x)
            state.y = float(position.y)
            state.z = float(position.z)
            state.yaw = self._yaw_from_quaternion(message.pose.orientation)
            state.frame_id = str(getattr(message.header, "frame_id", "world"))
            state.timestamp = self._stamp_to_sec(message.header.stamp)
            state.pose_ready = True
            state.valid = state.pose_ready and state.twist_ready
            self._latest_timestamp = max(self._latest_timestamp, state.timestamp)

    def _handle_twist(self, vehicle_id: str, message: object) -> None:
        linear = message.twist.linear
        angular = message.twist.angular
        with self._lock:
            state = self._states[vehicle_id]
            state.vx = float(linear.x)
            state.vy = float(linear.y)
            state.vz = float(linear.z)
            state.wx = float(angular.x)
            state.wy = float(angular.y)
            state.wz = float(angular.z)
            state.frame_id = str(getattr(message.header, "frame_id", "world"))
            state.timestamp = self._stamp_to_sec(message.header.stamp)
            state.twist_ready = True
            state.valid = state.pose_ready and state.twist_ready
            self._latest_timestamp = max(self._latest_timestamp, state.timestamp)

    def get_snapshot(self) -> MocapSnapshot:
        """Copy state objects so callbacks cannot mutate an active control frame."""

        with self._lock:
            states = {vehicle_id: replace(state) for vehicle_id, state in self._states.items()}
            return MocapSnapshot(self._latest_timestamp, states)

    def all_valid(self) -> bool:
        with self._lock:
            return all(state.valid for state in self._states.values())
