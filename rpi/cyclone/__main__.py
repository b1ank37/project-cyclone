"""Запуск ПО Raspberry Pi.

    python3 -m cyclone serve --port /dev/serial0              # веб-панель :8080, ждёт команд
    python3 -m cyclone serve --sim ../firmware/sim/cyclone_sim  # то же, но с симулятором ESP32
    python3 -m cyclone serve --fake                             # Python-заглушка ESP32 (без компилятора)
    python3 -m cyclone run routes/demo.json --sim ../firmware/sim/cyclone_sim --sim-speed 5
    python3 -m cyclone cmd DRV 1.0 0.2 --port /dev/ttyUSB0      # одна команда (отладка)
    python3 -m cyclone markers 0 1 2 3 --out markers/           # PNG меток для печати
Дополнительно: --camera 0|picamera --marker-size 0.15  --telegram-token T --telegram-chat ID
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from pathlib import Path

from .link import EspLink, ProcessTransport, SerialTransport
from .mission import MissionRunner, Policy, load_route
from .notify import LogNotifier, MultiNotifier

HERE = Path(__file__).resolve().parent.parent


def make_link(a) -> tuple[EspLink, float]:
    if a.fake:
        from .fake_esp import FakeEsp

        return EspLink(FakeEsp(speed=a.sim_speed), heartbeat_s=0.2 / a.sim_speed).start(), a.sim_speed
    if a.sim:
        cmd = [a.sim, "--speed", str(a.sim_speed)] + (a.sim_args.split() if a.sim_args else [])
        return EspLink(ProcessTransport(cmd), heartbeat_s=0.2 / a.sim_speed).start(), a.sim_speed
    if a.port:
        return EspLink(SerialTransport(a.port, a.baud)).start(), 1.0
    sys.exit("укажите --port, --sim или --fake")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="cyclone", description="ПО Raspberry Pi робота-уборщика «Циклон»")
    p.add_argument("action", choices=["serve", "run", "cmd", "markers"])
    p.add_argument("args", nargs="*")
    p.add_argument("--port", help="/dev/serial0 (UART Pi) или /dev/ttyUSB0 (ESP32 по USB)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--sim", help="путь к firmware/sim/cyclone_sim")
    p.add_argument("--sim-args", default="", help="доп. параметры симулятора, напр. '--obstacle 3,0,0.2,5,12'")
    p.add_argument("--fake", action="store_true", help="Python-заглушка вместо ESP32")
    p.add_argument("--sim-speed", type=float, default=1.0)
    p.add_argument("--routes", default=str(HERE / "routes"))
    p.add_argument("--web", type=int, default=8080)
    p.add_argument("--camera", help="0 / 1 / picamera / путь к видео")
    p.add_argument("--marker-size", type=float, default=0.15)
    p.add_argument("--hfov", type=float, default=62.2)
    p.add_argument("--telegram-token", default=os.environ.get("CYCLONE_TG_TOKEN"))
    p.add_argument("--telegram-chat", type=int, action="append", default=[])
    p.add_argument("--out", default="markers")
    p.add_argument("-v", "--verbose", action="store_true")
    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, format="%(asctime)s %(name)s: %(message)s")

    if a.action == "markers":
        from .markers import generate

        ids = [int(x) for x in a.args] or list(range(4))
        for path in generate(ids, a.out):
            print(path)
        return 0

    link, scale = make_link(a)
    print("ESP32:", link.wait_ready(15))

    if a.action == "cmd":
        if not a.args:
            sys.exit("пример: cmd DRV 1.0 0.2")
        res = link.run(a.args[0].upper(), *a.args[1:], timeout=120)
        print("результат:", *res)
        link.close()
        return 0

    detector = camera = None
    if a.camera is not None:
        from .camera import Camera
        from .markers import MarkerDetector

        camera, detector = Camera(a.camera), MarkerDetector(a.marker_size, a.hfov)

    notifiers = [LogNotifier()]
    runner = MissionRunner(link, notify=lambda t: notifier.send(t), detector=detector, camera=camera,
                           policy=Policy(time_scale=scale))
    notifier = MultiNotifier(*notifiers)

    if a.action == "run":
        if not a.args:
            sys.exit("укажите файл маршрута")
        rep = runner.run(load_route(a.args[0]))
        if link.truth:
            print(f"истинное положение (симулятор): x={link.truth[0]:.2f} y={link.truth[1]:.2f} курс={link.truth[2]:.0f}°")
        link.close()
        return 0 if rep.status == "done" else 1

    # serve
    from .app import Controller, make_server

    ctrl = Controller(link, runner, a.routes)
    if a.telegram_token and a.telegram_chat:
        from .notify import TelegramBot

        bot = TelegramBot(a.telegram_token, a.telegram_chat, ctrl.on_command)
        notifier.items.append(bot)
        bot.start()
    srv = make_server(ctrl, port=a.web)
    print(f"веб-панель: http://<адрес Pi>:{a.web}  (Ctrl+C — выход)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.stop_motion()
        link.close()
        time.sleep(0.2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
