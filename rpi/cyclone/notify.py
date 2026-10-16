"""Уведомления и команды из мессенджера.

Telegram в России с 2026 г. работает с перебоями (замедление), поэтому основной
способ управления — веб-панель (cyclone/web.py), а бот — дополнительный.
Чтобы подключить другой мессенджер (например MAX), достаточно реализовать тот же
интерфейс: send(text) и вызывать on_command(name) при входящих командах.
Работает только на стандартной библиотеке (urllib), без сторонних пакетов.
"""
from __future__ import annotations

import json
import logging
import threading
import urllib.parse
import urllib.request
from typing import Callable

log = logging.getLogger("cyclone.notify")


class Notifier:
    def send(self, text: str) -> None:
        raise NotImplementedError


class LogNotifier(Notifier):
    def send(self, text: str) -> None:
        log.info("уведомление: %s", text)
        print(f"[робот] {text}", flush=True)


class MultiNotifier(Notifier):
    def __init__(self, *items: Notifier):
        self.items = list(items)

    def send(self, text: str) -> None:
        for n in self.items:
            try:
                n.send(text)
            except Exception as e:  # один канал не должен ронять другие
                log.warning("канал %s: %s", type(n).__name__, e)


COMMANDS = {
    "/start_clean": "start", "/stop": "stop", "/status": "status", "/home": "home",
    "/blade_up": "blade_up", "/blade_down": "blade_down",
}


class TelegramBot(Notifier):
    """Бот: /start_clean [маршрут], /stop, /status, /home, /blade_up, /blade_down.

    Отвечает только чатам из allowed_chats (иначе кто угодно сможет управлять роботом).
    """

    API = "https://api.telegram.org/bot{token}/{method}"

    def __init__(self, token: str, allowed_chats: list[int], on_command: Callable[[str, list[str]], str]):
        self.token = token
        self.allowed = set(allowed_chats)
        self.on_command = on_command
        self._offset = 0
        self._stop = threading.Event()

    def _call(self, method: str, params: dict, timeout: float = 35) -> dict:
        data = urllib.parse.urlencode(params).encode()
        req = urllib.request.Request(self.API.format(token=self.token, method=method), data=data)
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())

    def send(self, text: str) -> None:
        for chat in self.allowed:
            self._call("sendMessage", {"chat_id": chat, "text": text}, timeout=10)

    def start(self) -> None:
        threading.Thread(target=self._poll, daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def handle_update(self, upd: dict) -> str | None:
        """Обработать одно обновление; вернуть текст ответа (для тестов — без сети)."""
        msg = upd.get("message") or {}
        chat = (msg.get("chat") or {}).get("id")
        text = (msg.get("text") or "").strip()
        if not text.startswith("/"):
            return None
        if chat not in self.allowed:
            return "нет доступа"
        parts = text.split()
        cmd = COMMANDS.get(parts[0].split("@")[0])
        if cmd is None:
            return "команды: " + ", ".join(COMMANDS)
        return self.on_command(cmd, parts[1:])

    def _poll(self) -> None:
        while not self._stop.is_set():
            try:
                res = self._call("getUpdates", {"offset": self._offset, "timeout": 30})
                for upd in res.get("result", []):
                    self._offset = upd["update_id"] + 1
                    reply = self.handle_update(upd)
                    chat = ((upd.get("message") or {}).get("chat") or {}).get("id")
                    if reply and chat is not None:
                        self._call("sendMessage", {"chat_id": chat, "text": reply}, timeout=10)
            except Exception as e:
                log.warning("Telegram недоступен: %s", e)
                self._stop.wait(10)
