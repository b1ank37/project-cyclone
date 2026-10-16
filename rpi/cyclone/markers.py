"""Метки ArUco: поиск на кадре, оценка расстояния и азимута.

Метку печатают (например 15×15 см) и вешают на столбик/стену у точки поворота.
Расстояние оценивается по видимому размеру метки (модель камеры-обскуры):
    d ≈ f_px · размер_метки / сторона_в_пикселях,  f_px = (ширина_кадра/2) / tan(HFOV/2)
Точнее — после калибровки камеры (шахматная доска), но для разворота у метки хватает.
Азимут: плюс — метка слева (поворачивать против часовой, как TRN с плюсом).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Marker:
    id: int
    distance_m: float
    bearing_deg: float
    side_px: float
    center: tuple[float, float]


def _aruco():
    import cv2

    if not hasattr(cv2, "aruco"):
        raise RuntimeError("нужен OpenCV с модулем aruco: pip install opencv-contrib-python-headless")
    return cv2, cv2.aruco


class MarkerDetector:
    def __init__(self, marker_size_m: float = 0.15, hfov_deg: float = 62.2, dictionary: str = "DICT_4X4_50"):
        cv2, aruco = _aruco()
        self.size = marker_size_m
        self.hfov = hfov_deg   # Raspberry Pi Camera v2: 62,2°; Camera Module 3: 66°; веб-камеры ~60–78°
        self._dict = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        if hasattr(aruco, "ArucoDetector"):           # OpenCV ≥ 4.7
            self._det = aruco.ArucoDetector(self._dict, aruco.DetectorParameters())
            self._detect = lambda g: self._det.detectMarkers(g)
        else:                                          # старые OpenCV
            params = aruco.DetectorParameters_create()
            self._detect = lambda g: aruco.detectMarkers(g, self._dict, parameters=params)

    def detect(self, frame: np.ndarray) -> list[Marker]:
        cv2, _ = _aruco()
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return []
        h, w = gray.shape[:2]
        f_px = (w / 2) / math.tan(math.radians(self.hfov / 2))
        out = []
        for c, i in zip(corners, ids.flatten()):
            pts = c.reshape(4, 2)
            sides = [np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]
            # по вертикальным сторонам точнее: при взгляде сбоку горизонтальные сжимаются
            side = float(max(sides[1], sides[3], 1e-6))
            cx, cy = pts.mean(axis=0)
            dist = f_px * self.size / side
            bearing = -math.degrees(math.atan((cx - w / 2) / f_px))
            out.append(Marker(int(i), dist, bearing, side, (float(cx), float(cy))))
        return out


def generate(ids: list[int], out_dir: str, px: int = 600, dictionary: str = "DICT_4X4_50") -> list[str]:
    """Сохранить PNG меток для печати (с белым полем и подписью)."""
    import os

    cv2, aruco = _aruco()
    d = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    for i in ids:
        img = aruco.generateImageMarker(d, i, px) if hasattr(aruco, "generateImageMarker") else aruco.drawMarker(d, i, px)
        pad = px // 6
        canvas = np.full((px + 2 * pad + 40, px + 2 * pad), 255, np.uint8)
        canvas[pad:pad + px, pad:pad + px] = img
        cv2.putText(canvas, f"ID {i}  ({dictionary})", (pad, px + pad + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)
        p = os.path.join(out_dir, f"aruco_{i:02d}.png")
        cv2.imwrite(p, canvas)
        paths.append(p)
    return paths
