"""Fault-isolated fan-out to multiple telemetry consumers."""

from __future__ import annotations

from typing import List

from swarm.application.interfaces import TelemetrySink
from swarm.domain.models import ControlFrame


class CompositeTelemetrySink:
    """Deliver frames to each sink without letting telemetry stop control."""

    def __init__(self, sinks: List[TelemetrySink]) -> None:
        self._sinks = list(sinks)
        self._failed = set()

    def emit(self, frame: ControlFrame) -> None:
        for sink in self._sinks:
            if id(sink) in self._failed:
                continue
            try:
                sink.emit(frame)
            except Exception as error:
                self._failed.add(id(sink))
                print("telemetry disabled | %s: %s" % (type(sink).__name__, error))

    def close(self) -> None:
        for sink in reversed(self._sinks):
            try:
                sink.close()
            except Exception as error:
                print("telemetry close failed | %s: %s" % (type(sink).__name__, error))
