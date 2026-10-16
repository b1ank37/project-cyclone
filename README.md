# «Циклон» v2 — робот-уборщик дорожек кампуса РУТ (МИИТ)

Прототип: колёсная платформа 4×4 с отвалом 45 см под 30°, ESP32 (низкий уровень) + Raspberry Pi (маршрут, метки ArUco, веб-панель/бот).

```
calc/       расчёты (raschet.py → отчёт и графики; build_xlsx.py → смета и расчётный лист Excel)
docs/       схемы (diagrams.py, electrical.dot), протокол PROTOCOL.md, img/
firmware/   прошивка ESP32 (PlatformIO). Логика в lib/core — без Arduino, тестируется на ПК
  sim/      симулятор: настоящее ядро прошивки + модель робота, говорит протоколом через stdin/stdout
  test/     тесты ядра (make), заглушки Arduino для проверки синтаксиса main.cpp
rpi/        ПО Raspberry Pi на Python: связь, маршруты, метки, веб-панель, бот, тесты
```

## Быстрый старт без железа

```bash
make -C firmware/test            # тесты ядра прошивки (101 проверка)
make -C firmware/test sim        # собрать симулятор
cd rpi && pip install -r requirements.txt
python3 -m unittest -v           # тесты ПО Pi (18), включая прогон маршрута через симулятор
python3 -m cyclone serve --sim ../firmware/sim/cyclone_sim   # веб-панель http://localhost:8080
python3 -m cyclone run routes/demo.json --sim ../firmware/sim/cyclone_sim --sim-speed 5 \
        --sim-args "--person 3,0,6 --true"   # «прохожий» на дорожке: робот ждёт и едет дальше
python3 -m cyclone serve --fake  # без компилятора: Python-заглушка ESP32
```

## На железе

1. ESP32: открыть `firmware` в VS Code + PlatformIO, `Upload`. В мониторе порта набрать `$PNG,1` — ответ `$ACK,1…`, `$DONE,1,OK,…`.
   Проверить знаки: `$VEL,2,0.1,0` — оба борта вперёд? Иначе `motorLeftSign/motorRightSign` и `encLeftSign/encRightSign` в `lib/core/config.h`. `countsPerRev` — по вашему мотору (11 × редукция × 4).
2. Raspberry Pi: включить UART (`raspi-config` → Interface → Serial: login shell — **нет**, hardware — **да**), соединить GPIO14→GPIO16 ESP32, GPIO15←GPIO17, GND–GND.
3. `python3 -m cyclone serve --port /dev/serial0 [--camera picamera] [--telegram-token … --telegram-chat …]`.
4. Метки: `python3 -m cyclone markers 1 2 3 --out markers`, напечатать по 15 см.

## Что проверено, а что нет
- Ядро прошивки и ПО Pi — тесты на ПК и сквозной прогон через симулятор.
- `firmware/src/main.cpp` (работа с выводами ESP32), камера, Telegram — **не запускались на железе**: при первой сборке возможны мелкие правки.
- Числа в расчётах — оценки; коэффициенты трения и плотность снега проверить опытом с динамометром.
