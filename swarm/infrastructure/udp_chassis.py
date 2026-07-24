"""UDP transport for four-wheel ground-vehicle commands."""

from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class SocketConfig:
    port: int = 12345
    timeout_s: float = 0.2
    encoding: str = "utf-8"


class ChassisSocketDriver:
    """Send one command using the original chassis datagram format."""

    def __init__(self, address: str, config: SocketConfig) -> None:
        self.address = address
        self.config = config

    def encode(self, command: Mapping[str, float]) -> bytes:
        """Encode wheel values with the original rounding and field order."""

        return (
            "<%d,%d,%d,%d>"
            % (
                int(round(command["front_left"])),
                int(round(command["front_right"])),
                int(round(command["rear_left"])),
                int(round(command["rear_right"])),
            )
        ).encode(self.config.encoding)

    def send(self, command: Mapping[str, float]) -> bool:
        payload = self.encode(command)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(self.config.timeout_s)
                sock.sendto(payload, (self.address, self.config.port))
            return True
        except OSError:
            return False
