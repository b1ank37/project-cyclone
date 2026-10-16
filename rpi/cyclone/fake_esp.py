"""Упрощённая «ESP32» на Python — для отладки ПО Pi без железа и без компилятора C++.

Говорит тем же протоколом. Физики почти нет: робот едет с заданной скоростью.
Для проверки сценариев можно заранее задать исходы команд:
    fake = FakeEsp(script={"DRV": ["BLOCKED", "OK"]})   # первая поездка упрётся в препятствие
Точнее (с регуляторами, проскальзыванием, датчиками) — симулятор firmware/sim/cyclone_sim.
"""
from __future__ import annotations

import math
import queue
import threading
import time
from typing import Iterator

from .protocol import frame, parse


class FakeEsp:
    def __init__(self, speed: float = 10.0, script: dict[str, list[str]] | None = None, vbat: float = 15.2):
        self.speed = speed                      # ускорение времени
        self.script = {k: list(v) for k, v in (script or {}).items()}
        self.vbat = vbat
        self.x = self.y = self.th = 0.0         # th — градусы
        self.blade = False
        self.state = "IDLE"
        self.commands: list[list[str]] = []     # все принятые команды (для тестов)
        self._out: "queue.Queue[str | None]" = queue.Queue()
        self._lock = threading.Lock()
        self._job: dict | None = None
        self._last_seq = -1
        self._t = 0.0                            # «время ESP32», с
        self._last_rx = 0.0
        self._stop = threading.Event()
        self._emit("EVT,BOOT,fake-esp")
        threading.Thread(target=self._loop, daemon=True).start()

    # ------------- Transport
    def write(self, data: bytes) -> None:
        for line in data.decode("ascii").splitlines():
            self._on_line(line)

    def lines(self) -> Iterator[str]:
        while True:
            item = self._out.get()
            if item is None:
                return
            yield item

    def close(self) -> None:
        self._stop.set()
        self._out.put(None)

    # ------------- логика
    def _emit(self, body: str) -> None:
        self._out.put(frame(body).decode().rstrip("\n"))

    def _on_line(self, line: str) -> None:
        f = parse(line, allow_no_checksum=True)
        if not f:
            return
        with self._lock:
            self._last_rx = self._t
            if f[0] == "HB":
                return
            seq = int(f[1])
            if seq == self._last_seq:
                self._emit(f"ACK,{seq}")
                return
            self.commands.append(f)
            cmd, args = f[0], f[2:]
            busy = self._job is not None
            if cmd in ("DRV", "TRN", "CAL") and busy:
                self._emit(f"NAK,{seq},BUSY")
                return
            if cmd in ("DRV", "TRN") and self.vbat < 12.8:
                self._emit(f"NAK,{seq},LOWBAT")
                return
            self._last_seq = seq
            self._emit(f"ACK,{seq}")
            if cmd == "PNG":
                self._emit(f"DONE,{seq},OK,fake-esp")
            elif cmd == "STP":
                if self._job:
                    self._emit(f"DONE,{self._job['seq']},ABORT,STOP")
                    self._job = None
                self.state = "IDLE"
                self._emit(f"DONE,{seq},OK,STOPPED")
            elif cmd == "BLD":
                self.blade = args[0] == "1"
                self._emit(f"DONE,{seq},OK,{'DOWN' if self.blade else 'UP'}")
            elif cmd in ("RST", "SET"):
                if cmd == "RST":
                    self.x = self.y = self.th = 0.0
                self._emit(f"DONE,{seq},OK,")
            elif cmd == "CAL":
                self._job = {"seq": seq, "kind": "CAL", "left": 0.3}
                self.state = "CALIB"
            elif cmd == "DRV":
                dist = float(args[0])
                sp = float(args[1]) if len(args) > 1 else 0.25
                outcome = self._next_outcome("DRV")
                self._job = {"seq": seq, "kind": "DRV", "left": abs(dist), "dir": math.copysign(1, dist),
                             "speed": sp, "outcome": outcome, "total": abs(dist)}
                self.state = "DRIVE"
            elif cmd == "TRN":
                ang = float(args[0])
                if len(args) >= 3 and args[2] == "1":   # абсолютный курс
                    ang = (ang - self.th + 180) % 360 - 180
                self._job = {"seq": seq, "kind": "TRN", "left": abs(ang), "dir": math.copysign(1, ang),
                             "speed": 45.0, "outcome": self._next_outcome("TRN"), "total": abs(ang)}
                self.state = "TURN"
            elif cmd == "VEL":
                pass
            else:
                self._emit(f"NAK,{seq},UNKNOWN")

    def _next_outcome(self, cmd: str) -> str:
        lst = self.script.get(cmd)
        return lst.pop(0) if lst else "OK"

    def _loop(self) -> None:
        dt = 0.01
        tel = 0.0
        while not self._stop.is_set():
            time.sleep(dt / self.speed)
            with self._lock:
                self._t += dt
                j = self._job
                if j:
                    if j["kind"] in ("DRV", "TRN") and self._t - self._last_rx > 1.0:
                        self._emit("EVT,LINK_LOST")
                        self._finish("LINK", "NO_HEARTBEAT")
                    elif j["kind"] == "CAL":
                        j["left"] -= dt
                        if j["left"] <= 0:
                            self._emit("EVT,CAL,OK,0.0,0.0")
                            self._finish("OK", "GYRO")
                    else:
                        step = j["speed"] * dt
                        j["left"] -= step
                        if j["kind"] == "DRV":
                            r = math.radians(self.th)
                            self.x += j["dir"] * step * math.cos(r)
                            self.y += j["dir"] * step * math.sin(r)
                        else:
                            self.th = (self.th + j["dir"] * step + 180) % 360 - 180
                        bad = j["outcome"] != "OK"
                        if bad and j["left"] <= j["total"] / 2:
                            if j["outcome"] == "BLOCKED":
                                self._emit("EVT,OBST,30")
                            elif j["outcome"] == "STALL":
                                self._emit("EVT,STALL,BOTH")
                            self._finish(j["outcome"], "SCRIPTED")
                        elif j["left"] <= 0:
                            self._finish("OK", "")
                if self._t - tel >= 0.1:
                    tel = self._t
                    self._emit(f"TEL,{int(self._t * 1000)},{self.x:.3f},{self.y:.3f},{self.th:.1f},0,0,{self.state},"
                               f"999,999,999,{self.vbat:.2f},{1 if self.blade else 0},0")

    def _finish(self, result: str, info: str) -> None:
        if self._job:
            self._emit(f"DONE,{self._job['seq']},{result},{info}")
        self._job = None
        self.state = "IDLE"
