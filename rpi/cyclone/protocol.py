"""Текстовый протокол Pi ⇄ ESP32:  $ТЕЛО*CS\\n  (CS — XOR байтов тела, 2 hex-цифры).

Зеркало firmware/lib/core/protocol.cpp. Описание команд — в docs/PROTOCOL.md.
"""
from __future__ import annotations

from dataclasses import dataclass


def checksum(body: str) -> int:
    c = 0
    for b in body.encode("ascii"):
        c ^= b
    return c


def frame(body: str) -> bytes:
    return f"${body}*{checksum(body):02X}\n".encode("ascii")


def parse(line: str, allow_no_checksum: bool = False) -> list[str] | None:
    """Вернуть поля кадра или None, если строка не кадр / неверная контрольная сумма."""
    line = line.strip()
    if not line.startswith("$"):
        return None
    body = line[1:]
    star = body.rfind("*")
    if star >= 0:
        cs = body[star + 1:]
        body = body[:star]
        try:
            if len(cs) != 2 or int(cs, 16) != checksum(body):
                return None
        except ValueError:
            return None
    elif not allow_no_checksum:
        return None
    return body.split(",")


@dataclass
class Telemetry:
    t_ms: int = 0
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    v_left: float = 0.0
    v_right: float = 0.0
    state: str = "?"
    range_left: float = 999.0
    range_center: float = 999.0
    range_right: float = 999.0
    vbat: float = 0.0
    blade_down: bool = False
    flags: int = 0

    # биты flags
    PAUSED, BUMPER, LOWBAT, CRITBAT, GYRO_FAIL, LINK_LOST = 1, 2, 4, 8, 16, 32

    @classmethod
    def from_fields(cls, f: list[str]) -> "Telemetry":
        # TEL,t,x,y,hdg,vL,vR,STATE,rL,rC,rR,vbat,blade,flags
        return cls(int(f[1]), float(f[2]), float(f[3]), float(f[4]), float(f[5]), float(f[6]), f[7],
                   float(f[8]), float(f[9]), float(f[10]), float(f[11]), f[12] == "1", int(f[13]))

    @property
    def paused(self) -> bool:
        return bool(self.flags & self.PAUSED)

    def as_dict(self) -> dict:
        d = self.__dict__.copy()
        d["paused"] = self.paused
        return d
