"""Контроллер (общий для веб-панели и бота) и веб-панель на стандартной библиотеке.

Панель: http://<адрес Pi>:8080 — статус, кнопки «Старт», «Стоп», «Домой», отвал.
Работает в локальной сети (Wi-Fi кампуса или точка доступа с телефона), интернет не нужен.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .link import EspLink, LinkTimeout, NakError
from .mission import MissionRunner, load_route

log = logging.getLogger("cyclone.app")


class Controller:
    def __init__(self, link: EspLink, runner: MissionRunner, routes_dir: str | Path):
        self.link = link
        self.runner = runner
        self.routes_dir = Path(routes_dir)
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self.messages: list[tuple[float, str]] = []
        link.on_event(lambda ev: self._msg(f"событие: {ev.name} {' '.join(ev.args)}".strip()))

    def _msg(self, text: str) -> None:
        self.messages.append((time.time(), text))
        del self.messages[:-50]

    def routes(self) -> list[str]:
        return sorted(p.stem for p in self.routes_dir.glob("*.json"))

    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _spawn(self, fn, *args) -> str | None:
        with self._lock:
            if self.busy():
                return "робот занят — сначала «Стоп»"
            self._thread = threading.Thread(target=fn, args=args, daemon=True)
            self._thread.start()
        return None

    # --- команды (возвращают короткий ответ для человека)
    def start(self, name: str | None = None) -> str:
        names = self.routes()
        if not names:
            return "нет маршрутов в папке routes"
        name = name or names[0]
        if name not in names:
            return f"нет маршрута {name}; есть: {', '.join(names)}"
        route = load_route(self.routes_dir / f"{name}.json")
        err = self._spawn(self.runner.run, route)
        return err or f"запускаю «{route.name}»"

    def stop(self) -> str:
        if self.busy():
            self.runner.request_stop()
        else:
            self.link.stop_motion()
        return "стоп"

    def home(self) -> str:
        if not self.runner.path:
            return "путь пуст — робот на месте старта"
        return self._spawn(self.runner.return_home) or "возвращаюсь на базу"

    def blade(self, down: bool) -> str:
        if self.busy():
            return "во время маршрута отвалом управляет маршрут"
        try:
            self.link.run("BLD", 1 if down else 0, timeout=3)
            return "отвал опущен" if down else "отвал поднят"
        except (NakError, LinkTimeout) as e:
            return f"ошибка: {e}"

    def status(self) -> dict:
        t = self.link.telemetry
        rep = self.runner.report
        return {
            "telemetry": t.as_dict(),
            "telemetry_age_s": round(time.monotonic() - self.link.telemetry_time, 1) if self.link.telemetry_time else None,
            "busy": self.busy(),
            "mission": rep.as_dict() if rep else None,
            "routes": self.routes(),
            "messages": self.messages[-15:],
        }

    def status_text(self) -> str:
        t = self.link.telemetry
        rep = self.runner.report
        m = f"маршрут: {rep.route} — {rep.status} ({rep.steps_done} шагов, {rep.distance_m:.1f} м)" if rep else "маршрутов не было"
        return (f"состояние {t.state}, x={t.x:.1f} м, y={t.y:.1f} м, курс {t.heading:.0f}°, "
                f"АКБ {t.vbat:.1f} В, отвал {'внизу' if t.blade_down else 'поднят'}\n{m}")

    def on_command(self, cmd: str, args: list[str]) -> str:
        """Вход для бота: имена команд — см. notify.COMMANDS."""
        if cmd == "start":
            return self.start(args[0] if args else None)
        if cmd == "stop":
            return self.stop()
        if cmd == "home":
            return self.home()
        if cmd == "status":
            return self.status_text()
        if cmd in ("blade_up", "blade_down"):
            return self.blade(cmd == "blade_down")
        return "неизвестная команда"


PAGE = """<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Циклон</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#f4f6f7;color:#1c2833;max-width:640px}
h1{font-size:20px;margin:0 0 12px} .card{background:#fff;border-radius:10px;padding:12px;margin-bottom:12px;box-shadow:0 1px 3px #0002}
button{font-size:16px;padding:12px 14px;margin:4px 4px 4px 0;border:0;border-radius:8px;background:#1f618d;color:#fff}
button.stop{background:#c0392b;font-weight:700;width:100%;font-size:20px} select{font-size:16px;padding:8px}
table{width:100%;font-size:14px} td:first-child{color:#5d6d7e} #log{font-size:13px;white-space:pre-wrap}
</style></head><body>
<h1>Робот-уборщик «Циклон»</h1>
<div class="card"><button class="stop" onclick="cmd('stop')">СТОП</button></div>
<div class="card"><select id="route"></select> <button onclick="cmd('start',{route:val('route')})">Старт</button>
<button onclick="cmd('home')">Домой</button><br><button onclick="cmd('blade',{down:1})">Отвал вниз</button>
<button onclick="cmd('blade',{down:0})">Отвал вверх</button></div>
<div class="card"><table id="st"></table></div><div class="card" id="log"></div>
<script>
const val=id=>document.getElementById(id).value;
async function cmd(c,body={}){const r=await fetch('/api/'+c,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
 const j=await r.json();document.getElementById('log').textContent=j.reply+'\\n'+document.getElementById('log').textContent;}
async function poll(){try{const s=await (await fetch('/api/status')).json();const t=s.telemetry;
 const sel=document.getElementById('route');if(sel.options.length!==s.routes.length){sel.innerHTML=s.routes.map(r=>`<option>${r}</option>`).join('');}
 const m=s.mission;const rows=[['Состояние',t.state+(t.paused?' (препятствие)':'')],['Положение',`x ${t.x.toFixed(2)} м, y ${t.y.toFixed(2)} м, курс ${t.heading.toFixed(0)}°`],
 ['Скорость бортов',`${t.v_left.toFixed(2)} / ${t.v_right.toFixed(2)} м/с`],['Дальномеры',`${t.range_left} / ${t.range_center} / ${t.range_right} см`],
 ['АКБ',t.vbat.toFixed(2)+' В'],['Отвал',t.blade_down?'опущен':'поднят'],['Маршрут',m?`${m.route}: ${m.status}, шагов ${m.steps_done}, ${m.distance_m.toFixed(1)} м`:'—'],
 ['Связь',s.telemetry_age_s===null?'нет данных':(s.telemetry_age_s<1?'есть':'нет '+s.telemetry_age_s+' с')]];
 document.getElementById('st').innerHTML=rows.map(r=>`<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');}catch(e){}}
setInterval(poll,500);poll();
</script></body></html>"""


def make_server(ctrl: Controller, host: str = "0.0.0.0", port: int = 8080) -> ThreadingHTTPServer:
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            data = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/":
                data = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif self.path == "/api/status":
                self._json(ctrl.status())
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._json({"error": "bad json"}, 400)
            name = self.path.removeprefix("/api/")
            if name == "start":
                reply = ctrl.start(body.get("route"))
            elif name == "stop":
                reply = ctrl.stop()
            elif name == "home":
                reply = ctrl.home()
            elif name == "blade":
                reply = ctrl.blade(bool(body.get("down")))
            else:
                return self._json({"error": "unknown"}, 404)
            self._json({"reply": reply})

    return ThreadingHTTPServer((host, port), H)
