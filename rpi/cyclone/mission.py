"""Маршрут уборки и его выполнение.

Маршрут — JSON-файл со списком шагов (пример — routes/demo.json):
    {"name": "Дорожка у корпуса 1", "speed": 0.25, "steps": [
        {"blade": "down"},
        {"drive": 8.0},                       # метров вперёд (минус — назад)
        {"turn": 90},                         # градусов, плюс — налево (против часовой)
        {"marker": 3, "stop_at": 0.8},        # подъехать к метке ArUco №3 и встать в 0,8 м от неё
        {"wait": 2},                          # пауза, с
        {"blade": "up"}
    ]}
ESP32 сама удерживает курс, тормозит перед препятствием и ждёт, пока оно уйдёт.
Здесь — решения уровнем выше: повторить, сдать назад, сообщить человеку, вернуться.
"""
from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from .link import EspLink, LinkTimeout, NakError

log = logging.getLogger("cyclone.mission")


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


@dataclass
class Step:
    kind: str           # drive | turn | blade | marker | wait
    value: float = 0.0
    extra: dict = field(default_factory=dict)

    def describe(self) -> str:
        return {
            "drive": f"проехать {self.value:.2f} м",
            "turn": f"повернуть на {self.value:+.0f}°",
            "blade": "опустить отвал" if self.value else "поднять отвал",
            "marker": f"подъехать к метке №{int(self.value)}",
            "wait": f"подождать {self.value:.0f} с",
        }.get(self.kind, self.kind)


@dataclass
class Route:
    name: str
    speed: float
    steps: list[Step]


def load_route(src: str | Path | dict) -> Route:
    data = src if isinstance(src, dict) else json.loads(Path(src).read_text(encoding="utf-8"))
    steps: list[Step] = []
    for i, raw in enumerate(data["steps"], 1):
        if "drive" in raw:
            steps.append(Step("drive", float(raw["drive"]), {"speed": raw.get("speed")}))
        elif "turn" in raw:
            steps.append(Step("turn", float(raw["turn"])))
        elif "blade" in raw:
            if raw["blade"] not in ("up", "down"):
                raise ValueError(f"шаг {i}: blade должен быть up или down")
            steps.append(Step("blade", 1.0 if raw["blade"] == "down" else 0.0))
        elif "marker" in raw:
            steps.append(Step("marker", float(raw["marker"]), {"stop_at": float(raw.get("stop_at", 0.8))}))
        elif "wait" in raw:
            steps.append(Step("wait", float(raw["wait"])))
        else:
            raise ValueError(f"шаг {i}: неизвестный тип {raw}")
    speed = float(data.get("speed", 0.25))
    if not 0.02 <= speed <= 0.6:
        raise ValueError("speed должна быть в пределах 0.02–0.6 м/с")
    return Route(data.get("name", "маршрут"), speed, steps)


@dataclass
class Policy:
    blocked_retries: int = 2        # сколько раз пробовать снова после «препятствие не ушло»
    blocked_pause_s: float = 5.0
    stall_retries: int = 1          # застряли: поднять отвал, сдать назад, попробовать ещё раз
    stall_backoff_m: float = 0.3
    marker_scan_deg: float = 15.0
    marker_speed: float = 0.15
    time_scale: float = 1.0         # для симулятора с ускорением времени


@dataclass
class Report:
    route: str
    status: str = "running"         # running | done | stopped | failed | paused
    message: str = ""
    steps_done: int = 0
    distance_m: float = 0.0
    started: float = field(default_factory=time.time)
    finished: float = 0.0
    log: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return self.__dict__.copy()


class MissionStopped(Exception):
    pass


class MissionFailed(Exception):
    pass


class MissionRunner:
    def __init__(self, link: EspLink, notify: Callable[[str], None] = print,
                 detector=None, camera=None, policy: Policy | None = None):
        self.link = link
        self.notify = notify
        self.detector = detector
        self.camera = camera
        self.policy = policy or Policy()
        self.report: Report | None = None
        self.path: list[tuple[str, float]] = []   # фактически выполненные перемещения (для возврата)
        self.goal_heading = 0.0   # курс, который «должен» быть по маршруту (ошибки поворотов не копятся)
        self._stop = False

    # ------------------------------------------------ управление
    def request_stop(self) -> None:
        self._stop = True
        self.link.stop_motion()

    def run(self, route: Route) -> Report:
        self._stop = False
        self.path = []
        rep = self.report = Report(route.name)
        self._say(f"Старт маршрута «{route.name}»: {len(route.steps)} шагов")
        try:
            self._cmd("CAL", timeout=10)    # робот стоит — уточняем смещение гироскопа
            self.goal_heading = self.link.telemetry.heading
            for i, st in enumerate(route.steps, 1):
                self._check_stop()
                self._log(f"шаг {i}/{len(route.steps)}: {st.describe()}")
                self._do(st, route)
                rep.steps_done = i
            rep.status, rep.message = "done", "маршрут пройден"
        except MissionStopped:
            rep.status, rep.message = "stopped", "остановлено оператором"
        except MissionFailed as e:
            rep.status, rep.message = "failed", str(e)
        except (LinkTimeout, NakError) as e:
            rep.status, rep.message = "failed", f"связь с ESP32: {e}"
            self.link.stop_motion()
        rep.finished = time.time()
        self._say(f"Маршрут «{route.name}»: {rep.message} ({rep.steps_done}/{len(route.steps)} шагов, "
                  f"{rep.distance_m:.1f} м)")
        return rep

    def return_home(self) -> Report:
        """Вернуться к месту старта тем же путём (поднимаем отвал, разворот, шаги в обратном порядке)."""
        back: list[Step] = [Step("blade", 0.0), Step("turn", 180.0)]
        for kind, v in reversed(self.path):
            back.append(Step(kind, v if kind == "drive" else -v))
        back.append(Step("turn", 180.0))
        return self.run(Route("возврат на базу", 0.25, back))

    # ------------------------------------------------ шаги
    def _do(self, st: Step, route: Route) -> None:
        if st.kind == "drive":
            self._drive(st.value, st.extra.get("speed") or route.speed)
        elif st.kind == "turn":
            self._turn(st.value)
        elif st.kind == "blade":
            self._cmd("BLD", int(st.value), timeout=5)
        elif st.kind == "wait":
            end = time.monotonic() + st.value / self.policy.time_scale
            while time.monotonic() < end:
                self._check_stop()
                time.sleep(0.05)
        elif st.kind == "marker":
            self._marker(int(st.value), st.extra["stop_at"])

    def _drive(self, dist: float, speed: float) -> None:
        remaining = dist
        blocked = stalls = 0
        while abs(remaining) > 0.02:
            x0, y0 = self.link.telemetry.x, self.link.telemetry.y
            timeout = abs(remaining) / max(speed, 0.05) * 3 + 40
            res, info = self._cmd("DRV", remaining, speed, round(self.goal_heading, 1),
                                  timeout=timeout / self.policy.time_scale)
            moved = math.hypot(self.link.telemetry.x - x0, self.link.telemetry.y - y0)
            moved = min(moved, abs(remaining))
            if res == "OK":
                moved = abs(remaining)
            self._record("drive", math.copysign(moved, remaining))
            remaining = math.copysign(abs(remaining) - moved, remaining)
            if res == "OK":
                return
            if res == "BLOCKED":
                blocked += 1
                if blocked > self.policy.blocked_retries:
                    raise MissionFailed(f"дорога перекрыта: препятствие не уходит (осталось {abs(remaining):.1f} м)")
                self._say(f"Препятствие на пути, жду и пробую снова ({blocked}/{self.policy.blocked_retries})")
                self._sleep(self.policy.blocked_pause_s)
            elif res == "STALL":
                stalls += 1
                if stalls > self.policy.stall_retries:
                    raise MissionFailed("застрял: колёса не проворачивают (глубокий/плотный снег?)")
                self._say("Застрял — поднимаю отвал, сдаю назад и пробую ещё раз")
                self._cmd("BLD", 0, timeout=5)
                back = self.policy.stall_backoff_m
                r2, _ = self._cmd("DRV", -back, 0.12, timeout=30 / self.policy.time_scale)
                if r2 == "OK":
                    self._record("drive", -back)
                    remaining += math.copysign(back, remaining)
                self._cmd("BLD", 1, timeout=5)
            elif res == "BUMPER":
                self._cmd("DRV", -0.2, 0.1, timeout=20 / self.policy.time_scale)
                self._record("drive", -0.2)
                raise MissionFailed("сработал бампер — касание препятствия, нужен осмотр")
            elif res == "LOWBAT":
                raise MissionFailed("батарея разряжена")
            elif res == "ABORT":
                raise MissionStopped()
            else:
                raise MissionFailed(f"поездка прервана: {res} {info}")

    def _turn(self, angle: float) -> None:
        self.goal_heading = wrap180(self.goal_heading + angle)
        # абсолютный курс (5-е поле = 1): ESP32 доворачивает с учётом прошлых недоворотов
        res, info = self._cmd("TRN", round(self.goal_heading, 1), 45, 1,
                              timeout=(abs(angle) / 20 + 15) / self.policy.time_scale)
        if res == "ABORT":
            raise MissionStopped()
        if res != "OK":
            raise MissionFailed(f"поворот не удался: {res} {info}")
        self._record("turn", angle)

    def _marker(self, marker_id: int, stop_at: float) -> None:
        if self.detector is None or self.camera is None:
            self._say(f"Метка №{marker_id}: камера не подключена — шаг пропущен")
            return
        m = self._look(marker_id)
        scan = self.policy.marker_scan_deg
        for ang in (scan, -2 * scan):
            if m:
                break
            self._turn(ang)
            m = self._look(marker_id)
        if not m:
            self._turn(scan)  # вернуть исходный курс
            self._say(f"Метка №{marker_id} не найдена — еду дальше по одометрии")
            return
        if abs(m.bearing_deg) > 2:
            self._turn(m.bearing_deg)
            m = self._look(marker_id) or m
        gap = m.distance_m - stop_at
        self._log(f"метка №{marker_id}: {m.distance_m:.2f} м, азимут {m.bearing_deg:+.1f}°")
        if gap > 0.05:
            self._drive(gap, self.policy.marker_speed)

    def _look(self, marker_id: int, tries: int = 5):
        for _ in range(tries):
            frame = self.camera.read()
            if frame is None:
                continue
            for m in self.detector.detect(frame):
                if m.id == marker_id:
                    return m
        return None

    # ------------------------------------------------ служебное
    def _cmd(self, cmd: str, *args, timeout: float) -> tuple[str, str]:
        self._check_stop()
        res, info = self.link.run(cmd, *args, timeout=timeout)
        if res == "ABORT" or self._stop:
            raise MissionStopped()
        if cmd in ("CAL",) and res != "OK":
            self._say("Калибровка гироскопа не удалась (робот двигался?) — продолжаю со старой")
        return res, info

    def _record(self, kind: str, v: float) -> None:
        if abs(v) < 1e-3:
            return
        self.path.append((kind, v))
        if kind == "drive" and self.report:
            self.report.distance_m += abs(v)

    def _check_stop(self) -> None:
        if self._stop:
            raise MissionStopped()

    def _sleep(self, s: float) -> None:
        end = time.monotonic() + s / self.policy.time_scale
        while time.monotonic() < end:
            self._check_stop()
            time.sleep(0.05)

    def _log(self, msg: str) -> None:
        log.info(msg)
        if self.report:
            self.report.log.append(msg)

    def _say(self, msg: str) -> None:
        self._log(msg)
        try:
            self.notify(msg)
        except Exception:
            log.exception("уведомление не отправлено")
