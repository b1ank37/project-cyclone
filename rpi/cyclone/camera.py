"""Источник кадров: камера Raspberry Pi (picamera2), USB-камера или видеофайл (OpenCV).

Не проверялось на реальной камере — интерфейс простой: read() → кадр BGR или None.
"""
from __future__ import annotations


class Camera:
    def __init__(self, source: str = "0", width: int = 1280, height: int = 720):
        self._pc = None
        self._cap = None
        if source == "picamera":
            from picamera2 import Picamera2  # есть в Raspberry Pi OS

            self._pc = Picamera2()
            cfg = self._pc.create_video_configuration(main={"size": (width, height), "format": "RGB888"})
            self._pc.configure(cfg)
            self._pc.start()
        else:
            import cv2

            src = int(source) if source.isdigit() else source
            self._cap = cv2.VideoCapture(src)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if not self._cap.isOpened():
                raise RuntimeError(f"камера {source} не открылась")

    def read(self):
        if self._pc is not None:
            return self._pc.capture_array()   # формат RGB888 у picamera2 — это порядок BGR для OpenCV
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        if self._pc is not None:
            self._pc.stop()
        if self._cap is not None:
            self._cap.release()
