"""Связь с ESP32: отправка команд с повторами, ожидание ACK/DONE, heartbeat, телеметрия, события."""
from __future__ import annotations

import logging
import queue
import random
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Iterator, Protocol

from .protocol import Telemetry, frame, parse

log = logging.getLogger("cyclone.link")


class Transport(Protocol):
    def write(self, data: bytes) -> None: ...
    def lines(self) -> Iterator[str]: ...   # блокирующий итератор строк; заканчивается при закрытии
    def close(self) -> None: ...


class SerialTransport:
    """Реальный UART: /dev/serial0 на Raspberry Pi или USB-порт ESP32 на ноутбуке."""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # pyserial

        self._s = serial.Serial(port, baud, timeout=0.2)
        self._closed = False

    def write(self, data: bytes) -> None:
        self._s.write(data)

    def lines(self) -> Iterator[str]:
        buf = b""
        while not self._closed:
            chunk = self._s.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                yield line.decode("ascii", "replace")

    def close(self) -> None:
        self._closed = True
        self._s.close()


class ProcessTransport:
    """Симулятор firmware/sim/cyclone_sim как дочерний процесс (stdin/stdout)."""

    def __init__(self, cmd: list[str]):
        self._p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)

    def write(self, data: bytes) -> None:
        try:
            self._p.stdin.write(data)
            self._p.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass

    def lines(self) -> Iterator[str]:
        for raw in iter(self._p.stdout.readline, b""):
            yield raw.decode("ascii", "replace").rstrip("\r\n")

    def close(self) -> None:
        try:
            self._p.stdin.close()
        except Exception:
            pass
        try:
            self._p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self._p.kill()


class NakError(RuntimeError):
    def __init__(self, cmd: str, reason: str):
        super().__init__(f"{cmd}: отказ ESP32 ({reason})")
        self.reason = reason


class LinkTimeout(RuntimeError):
    pass


@dataclass
class _Pending:
    cmd: str
    ack: threading.Event = field(default_factory=threading.Event)
    done: threading.Event = field(default_factory=threading.Event)
    nak: str | None = None
    result: str = ""
    info: str = ""


@dataclass
class Event:
    name: str
    args: list[str]
    t: float


class EspLink:
    def __init__(self, transport: Transport, heartbeat_s: float = 0.2):
        self.t = transport
        self.heartbeat_s = heartbeat_s
        self.telemetry = Telemetry()
        self.telemetry_time = 0.0
        self.truth: tuple[float, float, float] | None = None   # только из симулятора (#TRUE)
        self.events: "queue.Queue[Event]" = queue.Queue()
        self.event_log: list[Event] = []
        self._listeners: list[Callable[[Event], None]] = []
        self._pending: dict[int, _Pending] = {}
        self._lock = threading.Lock()
        self._wlock = threading.Lock()
        self._seq = random.randint(1000, 60000)   # после перезапуска Pi не совпадёт с прошлым seq
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # ---------------- жизненный цикл
    def start(self) -> "EspLink":
        for fn in (self._reader, self._heartbeat):
            th = threading.Thread(target=fn, daemon=True)
            th.start()
            self._threads.append(th)
        return self

    def close(self) -> None:
        self._stop.set()
        self.t.close()

    def on_event(self, cb: Callable[[Event], None]) -> None:
        self._listeners.append(cb)

    # ---------------- команды
    def _write(self, body: str) -> None:
        with self._wlock:
            self.t.write(frame(body))

    def send(self, cmd: str, *args, ack_timeout: float = 0.5, retries: int = 3) -> int:
        """Отправить команду и дождаться ACK. Возвращает seq. NakError — если ESP32 отказала."""
        with self._lock:
            self._seq = self._seq + 1 if self._seq < 2_000_000_000 else 1
            seq = self._seq
            p = _Pending(cmd)
            self._pending[seq] = p
        body = ",".join([cmd, str(seq)] + [self._fmt(a) for a in args])
        for attempt in range(retries):
            self._write(body)
            if p.ack.wait(ack_timeout):
                if p.nak:
                    with self._lock:
                        self._pending.pop(seq, None)
                    raise NakError(cmd, p.nak)
                return seq
            log.warning("нет ACK на %s (попытка %d)", body, attempt + 1)
        with self._lock:
            self._pending.pop(seq, None)
        raise LinkTimeout(f"ESP32 не отвечает на {cmd}")

    def wait_done(self, seq: int, timeout: float) -> tuple[str, str]:
        with self._lock:
            p = self._pending.get(seq)
        if p is None:
            raise KeyError(seq)
        ok = p.done.wait(timeout)
        with self._lock:
            self._pending.pop(seq, None)
        if not ok:
            raise LinkTimeout(f"нет DONE для {p.cmd} за {timeout:.0f} с")
        return p.result, p.info

    def run(self, cmd: str, *args, timeout: float = 60.0) -> tuple[str, str]:
        """Команда целиком: отправить, дождаться DONE. Возвращает (результат, пояснение)."""
        seq = self.send(cmd, *args)
        return self.wait_done(seq, timeout)

    def stop_motion(self) -> None:
        try:
            self.run("STP", timeout=2)
        except (NakError, LinkTimeout) as e:
            log.error("STP не прошёл: %s", e)

    @staticmethod
    def _fmt(a) -> str:
        if isinstance(a, bool):
            return "1" if a else "0"
        if isinstance(a, float):
            return f"{a:.3f}"
        return str(a)

    # ---------------- потоки
    def _heartbeat(self) -> None:
        while not self._stop.wait(self.heartbeat_s):
            try:
                self._write("HB")
            except Exception:  # порт закрыт
                return

    def _reader(self) -> None:
        for line in self.t.lines():
            if self._stop.is_set():
                return
            if line.startswith("#TRUE,"):
                try:
                    _, _, x, y, th = line.split(",")
                    self.truth = (float(x), float(y), float(th))
                except ValueError:
                    pass
                continue
            f = parse(line)
            if not f:
                continue
            try:
                self._dispatch(f)
            except (ValueError, IndexError) as e:
                log.warning("битый кадр %r: %s", line, e)

    def _dispatch(self, f: list[str]) -> None:
        kind = f[0]
        if kind == "TEL":
            self.telemetry = Telemetry.from_fields(f)
            self.telemetry_time = time.monotonic()
            return
        if kind == "EVT":
            ev = Event(f[1], f[2:], time.monotonic())
            self.event_log.append(ev)
            self.events.put(ev)
            for cb in self._listeners:
                try:
                    cb(ev)
                except Exception:
                    log.exception("обработчик события")
            return
        if kind in ("ACK", "NAK", "DONE"):
            if f[1] == "-":
                log.warning("ESP32: %s", ",".join(f))
                return
            seq = int(f[1])
            with self._lock:
                p = self._pending.get(seq)
            if p is None:
                return
            if kind == "ACK":
                p.ack.set()
            elif kind == "NAK":
                p.nak = f[2] if len(f) > 2 else "?"
                p.ack.set()
            else:
                p.result = f[2] if len(f) > 2 else ""
                p.info = f[3] if len(f) > 3 else ""
                p.ack.set()   # DONE подразумевает, что команда принята
                p.done.set()

    # ---------------- удобства
    def wait_ready(self, timeout: float = 10.0) -> str:
        """Дождаться, пока ESP32 ответит на PNG и закончит калибровку гироскопа."""
        t_end = time.monotonic() + timeout
        last: Exception | None = None
        while time.monotonic() < t_end:
            try:
                _, info = self.run("PNG", timeout=1.0)
                # ждём первую телеметрию и окончания калибровки гироскопа
                while (self.telemetry_time == 0 or self.telemetry.state == "CALIB") and time.monotonic() < t_end:
                    time.sleep(0.05)
                return info
            except (NakError, LinkTimeout) as e:
                last = e
                time.sleep(0.2)
        raise LinkTimeout(f"ESP32 не готова: {last}")
