"""Тесты ПО Raspberry Pi:  cd rpi && python3 -m unittest -v
Интеграционные тесты с настоящим ядром прошивки запускаются, если собран
firmware/sim/cyclone_sim (make -C firmware/test sim), иначе пропускаются.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import threading
import time
import unittest
import urllib.request
from pathlib import Path

from cyclone.fake_esp import FakeEsp
from cyclone.link import EspLink, NakError, ProcessTransport
from cyclone.mission import MissionRunner, Policy, load_route
from cyclone.notify import TelegramBot
from cyclone.protocol import Telemetry, checksum, frame, parse

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT / "firmware" / "sim" / "cyclone_sim"
ROUTES = ROOT / "rpi" / "routes"
SPEED = 10.0


def fake_link(**kw) -> tuple[EspLink, FakeEsp]:
    esp = FakeEsp(speed=SPEED, **kw)
    link = EspLink(esp, heartbeat_s=0.2 / SPEED).start()
    link.wait_ready(5)
    return link, esp


class Notes(list):
    def __call__(self, text):
        self.append(text)


class ProtocolTest(unittest.TestCase):
    def test_roundtrip_and_c_compat(self):
        self.assertEqual(frame("PNG,1"), f"$PNG,1*{0x50 ^ 0x4E ^ 0x47 ^ 0x2C ^ 0x31:02X}\n".encode())
        self.assertEqual(parse(frame("DRV,5,1.5,0.2").decode()), ["DRV", "5", "1.5", "0.2"])
        self.assertIsNone(parse("$PNG,1*00"))
        self.assertIsNone(parse("$PNG,1"))
        self.assertEqual(parse("$PNG,1", allow_no_checksum=True), ["PNG", "1"])
        self.assertIsNone(parse("#TRUE,1,2,3"))
        self.assertEqual(checksum("HB"), 0x0A)

    def test_telemetry(self):
        f = parse(frame("TEL,1200,1.000,-0.500,45.0,0.2,0.21,DRIVE,120,80,999,15.10,1,1").decode())
        t = Telemetry.from_fields(f)
        self.assertEqual((t.t_ms, t.state, t.blade_down, t.paused), (1200, "DRIVE", True, True))
        self.assertAlmostEqual(t.y, -0.5)


class LinkTest(unittest.TestCase):
    def test_commands(self):
        link, esp = fake_link()
        self.assertEqual(link.run("BLD", 1, timeout=2), ("OK", "DOWN"))
        seq = link.send("DRV", 1.0, 0.25)
        with self.assertRaises(NakError) as e:
            link.send("DRV", 1.0, 0.25)
        self.assertEqual(e.exception.reason, "BUSY")
        self.assertEqual(link.wait_done(seq, 10)[0], "OK")
        self.assertAlmostEqual(esp.x, 1.0, delta=0.05)
        link.close()


class MissionTest(unittest.TestCase):
    def runner(self, link, notes):
        return MissionRunner(link, notify=notes, policy=Policy(time_scale=SPEED, blocked_pause_s=0.5))

    def test_demo_route(self):
        link, esp = fake_link()
        notes = Notes()
        rep = self.runner(link, notes).run(load_route(ROUTES / "demo.json"))
        self.assertEqual(rep.status, "done", rep.message)
        self.assertAlmostEqual(rep.distance_m, 12.4, delta=0.05)
        self.assertAlmostEqual(esp.x, 0.0, delta=0.1)
        self.assertAlmostEqual(esp.y, 0.4, delta=0.1)
        self.assertEqual(esp.commands[1][0], "CAL")   # [0] — PNG из wait_ready
        link.close()

    def test_blocked_then_ok(self):
        link, esp = fake_link(script={"DRV": ["BLOCKED", "OK"]})
        notes = Notes()
        rep = self.runner(link, notes).run(load_route({"steps": [{"drive": 4.0}]}))
        self.assertEqual(rep.status, "done", rep.message)
        self.assertTrue(any("Препятствие" in n for n in notes))
        drv = [c for c in esp.commands if c[0] == "DRV"]
        self.assertEqual(len(drv), 2)
        self.assertAlmostEqual(float(drv[1][2]), 2.0, delta=0.1)   # доехали остаток
        self.assertAlmostEqual(esp.x, 4.0, delta=0.1)
        link.close()

    def test_blocked_forever_fails(self):
        link, esp = fake_link(script={"DRV": ["BLOCKED"] * 5})
        rep = self.runner(link, Notes()).run(load_route({"steps": [{"drive": 4.0}]}))
        self.assertEqual(rep.status, "failed")
        self.assertIn("перекрыта", rep.message)
        link.close()

    def test_stall_recovery_then_fail(self):
        link, esp = fake_link(script={"DRV": ["STALL", "OK", "STALL"]})
        rep = self.runner(link, Notes()).run(load_route({"steps": [{"blade": "down"}, {"drive": 4.0}]}))
        self.assertEqual(rep.status, "failed")
        self.assertIn("застрял", rep.message)
        names = [(c[0], c[2] if len(c) > 2 else "") for c in esp.commands]
        i = names.index(("DRV", "-0.300"))
        self.assertEqual(names[i - 1], ("BLD", "0"))   # перед откатом подняли отвал
        self.assertEqual(names[i + 1], ("BLD", "1"))
        link.close()

    def test_stop(self):
        link, esp = fake_link()
        r = self.runner(link, Notes())
        out = {}
        th = threading.Thread(target=lambda: out.setdefault("rep", r.run(load_route({"steps": [{"drive": 20.0}]}))))
        th.start()
        time.sleep(0.5)
        r.request_stop()
        th.join(5)
        self.assertEqual(out["rep"].status, "stopped")
        self.assertLess(esp.x, 19)
        link.close()

    def test_return_home(self):
        link, esp = fake_link()
        r = self.runner(link, Notes())
        r.run(load_route({"steps": [{"drive": 2.0}, {"turn": 90}, {"drive": 1.0}]}))
        self.assertAlmostEqual(esp.x, 2.0, delta=0.1)
        self.assertAlmostEqual(esp.y, 1.0, delta=0.1)
        rep = r.return_home()
        self.assertEqual(rep.status, "done", rep.message)
        self.assertAlmostEqual(esp.x, 0.0, delta=0.15)
        self.assertAlmostEqual(esp.y, 0.0, delta=0.15)
        self.assertAlmostEqual(abs(esp.th), 0.0, delta=1.0)
        link.close()

    def test_route_validation(self):
        with self.assertRaises(ValueError):
            load_route({"steps": [{"fly": 1}]})
        with self.assertRaises(ValueError):
            load_route({"speed": 2.0, "steps": []})
        r = load_route(ROUTES / "korpus1.json")
        self.assertEqual([s.kind for s in r.steps].count("marker"), 2)

    def test_marker_step_without_camera_is_skipped(self):
        link, esp = fake_link()
        notes = Notes()
        rep = self.runner(link, notes).run(load_route({"steps": [{"marker": 1}]}))
        self.assertEqual(rep.status, "done")
        self.assertTrue(any("камера не подключена" in n for n in notes))
        link.close()


class BotTest(unittest.TestCase):
    def test_access_and_commands(self):
        calls = []
        bot = TelegramBot("TOKEN", [42], lambda c, a: calls.append((c, a)) or f"ok {c}")
        upd = lambda chat, text: {"message": {"chat": {"id": chat}, "text": text}}
        self.assertEqual(bot.handle_update(upd(7, "/stop")), "нет доступа")
        self.assertEqual(bot.handle_update(upd(42, "/start_clean demo")), "ok start")
        self.assertEqual(calls, [("start", ["demo"])])
        self.assertIn("/stop", bot.handle_update(upd(42, "/dance")))
        self.assertIsNone(bot.handle_update(upd(42, "привет")))


class WebTest(unittest.TestCase):
    def test_panel(self):
        from cyclone.app import Controller, make_server

        link, esp = fake_link()
        runner = MissionRunner(link, notify=Notes(), policy=Policy(time_scale=SPEED))
        ctrl = Controller(link, runner, ROUTES)
        srv = make_server(ctrl, "127.0.0.1", 0)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        base = f"http://127.0.0.1:{srv.server_address[1]}"
        self.assertIn("Циклон", urllib.request.urlopen(base + "/").read().decode())
        st = json.loads(urllib.request.urlopen(base + "/api/status").read())
        self.assertIn("demo", st["routes"])
        req = urllib.request.Request(base + "/api/start", data=b'{"route":"demo"}', method="POST",
                                     headers={"Content-Type": "application/json"})
        self.assertIn("запускаю", json.loads(urllib.request.urlopen(req).read())["reply"])
        busy = urllib.request.Request(base + "/api/start", data=b"{}", method="POST")
        self.assertIn("занят", json.loads(urllib.request.urlopen(busy).read())["reply"])
        t_end = time.time() + 20
        while ctrl.busy() and time.time() < t_end:
            time.sleep(0.1)
        st = json.loads(urllib.request.urlopen(base + "/api/status").read())
        self.assertEqual(st["mission"]["status"], "done")
        srv.shutdown()
        link.close()


class MarkerTest(unittest.TestCase):
    def setUp(self):
        try:
            import cv2  # noqa: F401
            from cyclone.markers import MarkerDetector
        except Exception as e:  # pragma: no cover
            self.skipTest(f"нет OpenCV: {e}")
        self.det = MarkerDetector(0.15, 62.2)

    def render(self, marker_id, dist, lateral, w=1280, h=720):
        """Синтетический кадр: метка 15 см на расстоянии dist, смещена вбок на lateral (плюс — вправо)."""
        import cv2
        import numpy as np

        f = (w / 2) / math.tan(math.radians(62.2 / 2))
        side = int(round(f * 0.15 / dist))
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        m = cv2.aruco.generateImageMarker(d, marker_id, side)
        img = np.full((h, w), 200, np.uint8)
        cx = int(w / 2 + f * lateral / dist)
        y0, x0 = h // 2 - side // 2, cx - side // 2
        border = max(4, side // 8)
        img[y0 - border:y0 + side + border, x0 - border:x0 + side + border] = 255
        img[y0:y0 + side, x0:x0 + side] = m
        return img

    def test_distance_and_bearing(self):
        for dist, lat in ((1.0, 0.0), (2.0, 0.3), (3.0, -0.4)):
            ms = self.det.detect(self.render(5, dist, lat))
            self.assertEqual(len(ms), 1)
            m = ms[0]
            self.assertEqual(m.id, 5)
            self.assertAlmostEqual(m.distance_m, dist, delta=dist * 0.08)
            expect = -math.degrees(math.atan(lat / dist))
            self.assertAlmostEqual(m.bearing_deg, expect, delta=1.5)

    def test_generate(self):
        from cyclone.markers import generate

        with tempfile.TemporaryDirectory() as d:
            paths = generate([0, 7], d, px=300)
            self.assertEqual(len(paths), 2)
            self.assertTrue(all(os.path.getsize(p) > 0 for p in paths))


@unittest.skipUnless(SIM.exists(), "симулятор не собран: make -C firmware/test sim")
class FirmwareSimTest(unittest.TestCase):
    """Сквозной тест: ПО Pi ⇄ протокол ⇄ НАСТОЯЩЕЕ ядро прошивки ⇄ модель робота."""

    SPEED = 5.0

    def link(self, *extra):
        link = EspLink(ProcessTransport([str(SIM), "--speed", str(self.SPEED), "--true", *extra]),
                      heartbeat_s=0.2 / self.SPEED).start()
        link.wait_ready(10)
        return link

    def runner(self, link, notes):
        return MissionRunner(link, notify=notes, policy=Policy(time_scale=self.SPEED, blocked_pause_s=1.0))

    def test_demo_route_true_position(self):
        link = self.link()
        rep = self.runner(link, Notes()).run(load_route(ROUTES / "demo.json"))
        self.assertEqual(rep.status, "done", rep.message)
        time.sleep(0.3)
        x, y, th = link.truth
        self.assertAlmostEqual(x, 0.0, delta=0.35)
        self.assertAlmostEqual(y, 0.4, delta=0.15)
        self.assertAlmostEqual(abs(th), 180.0, delta=2.0)
        link.close()

    def test_person_crosses_path(self):
        # человек на дорожке в 2 м впереди: уходит через 6 с после того, как робот подъехал
        link = self.link("--person", "2.0,0,6")
        notes = Notes()
        rep = self.runner(link, notes).run(load_route({"steps": [{"drive": 3.0}]}))
        self.assertEqual(rep.status, "done", rep.message)
        names = [e.name for e in link.event_log]
        self.assertIn("OBST", names)
        self.assertIn("CLEAR", names)
        time.sleep(0.3)
        self.assertAlmostEqual(link.truth[0], 3.0, delta=0.1)
        link.close()

    def test_stuck_in_snow(self):
        link = self.link("--stuck-x", "1.0")
        rep = self.runner(link, Notes()).run(load_route({"steps": [{"blade": "down"}, {"drive": 3.0}]}))
        self.assertEqual(rep.status, "failed")
        self.assertIn("застрял", rep.message)
        self.assertIn("STALL", [e.name for e in link.event_log])
        link.close()


if __name__ == "__main__":
    unittest.main()
