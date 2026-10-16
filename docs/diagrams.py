"""Схемы для документа: электрическая (блочная), архитектура ПО, вид сверху."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle, Circle, Polygon
import os

plt.rcParams["font.family"] = "DejaVu Sans"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "img")
PWR, SIG, DATA, GND = "#C0392B", "#2C3E50", "#1F618D", "#7F8C8D"


def box(ax, x, y, w, h, t, fc="#FFFFFF", ec="#34495E", fs=7.5, bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.12,rounding_size=0.8",
                                fc=fc, ec=ec, lw=1.3))
    ax.text(x + w / 2, y + h / 2, t, ha="center", va="center", fontsize=fs, weight="bold" if bold else "normal")


def line(ax, pts, c=SIG, lw=1.4, ls="-", arrow=False, two=False):
    for i in range(len(pts) - 1):
        last = i == len(pts) - 2
        if arrow and last:
            ax.add_patch(FancyArrowPatch(pts[i], pts[i + 1], arrowstyle="<|-|>" if two else "-|>",
                                         mutation_scale=10, color=c, lw=lw, ls=ls))
        else:
            ax.plot([pts[i][0], pts[i + 1][0]], [pts[i][1], pts[i + 1][1]], color=c, lw=lw, ls=ls)


def label(ax, x, y, t, c=SIG, fs=6.3, ha="left"):
    ax.text(x, y, t, fontsize=fs, color=c, ha=ha, va="center",
            bbox=dict(fc="white", ec="none", pad=0.4, alpha=0.9))


def software():
    fig, ax = plt.subplots(figsize=(10, 5.6), dpi=170)
    ax.set_xlim(0, 100); ax.set_ylim(0, 56); ax.axis("off")
    ax.text(50, 54.5, "Архитектура ПО: кто за что отвечает", ha="center", fontsize=11, weight="bold")
    box(ax, 2, 6, 44, 44, "", fc="#FEF5E7", ec="#B9770E")
    ax.text(24, 47.5, "Raspberry Pi — Python (rpi/cyclone)", ha="center", fontsize=8.5, weight="bold", color="#B9770E")
    box(ax, 5, 36, 18, 8, "Веб-панель\n(в Wi-Fi кампуса /\nточке доступа)", fs=6.8)
    box(ax, 26, 36, 17, 8, "Бот (Telegram;\nMAX — через тот\nже интерфейс)", fs=6.8)
    box(ax, 5, 22, 38, 10, "MissionRunner: маршрут = список шагов\nDRIVE / TURN / BLADE / MARKER\nожидание препятствий, повторы, отчёт", fs=6.8, fc="#FCF3CF", bold=False)
    box(ax, 5, 9, 18, 9, "Метки ArUco\n(OpenCV): поправка\nкурса и дистанции", fs=6.8)
    box(ax, 26, 9, 17, 9, "EspLink: протокол\nтекстовые кадры\n$…*CS, повторы", fs=6.8)
    for x in (14, 34.5):
        line(ax, [(x, 36), (x, 32)], SIG, arrow=True)
    line(ax, [(14, 22), (14, 18)], SIG, arrow=True, two=True)
    line(ax, [(34.5, 22), (34.5, 18)], SIG, arrow=True, two=True)

    box(ax, 54, 6, 44, 44, "", fc="#E8F8F5", ec="#117A65")
    ax.text(76, 47.5, "ESP32 — C++ (firmware, PlatformIO)", ha="center", fontsize=8.5, weight="bold", color="#117A65")
    box(ax, 57, 36, 38, 8, "Robot (ядро, без Arduino — тестируется на ПК)\nразбор команд · состояние · телеметрия 10 Гц", fs=6.8, fc="#D1F2EB")
    box(ax, 57, 24, 18, 9, "Motion: профиль\nскорости, удержание\nкурса по гироскопу", fs=6.8)
    box(ax, 77, 24, 18, 9, "Safety: препятствие,\nбампер, застревание,\nАКБ, потеря связи", fs=6.8)
    box(ax, 57, 12, 18, 9, "ПИД скорости\nбортов по\nэнкодерам", fs=6.8)
    box(ax, 77, 12, 18, 9, "Odometry: x, y,\nкурс (гироскоп),\nпуть (энкодеры)", fs=6.8)
    box(ax, 57, 7, 38, 3.5, "HAL: ESP32 (main.cpp) или симулятор (sim/) — одинаковый интерфейс", fs=6.2, fc="#F4F6F7")
    line(ax, [(66, 36), (66, 33)], SIG, arrow=True)
    line(ax, [(86, 36), (86, 33)], SIG, arrow=True)
    line(ax, [(66, 24), (66, 21)], SIG, arrow=True)
    line(ax, [(86, 24), (86, 21)], SIG, arrow=True, two=True)
    line(ax, [(43, 13.5), (50, 13.5), (50, 40), (57, 40)], DATA, lw=2, arrow=True, two=True)
    label(ax, 50.5, 30, "UART\nкоманды ↓\nACK/DONE/\nTEL/EVT ↑", DATA, fs=6.2, ha="center")
    ax.text(50, 2.5, "Если Raspberry Pi или связь пропали — ESP32 останавливает робота (heartbeat 1 с). Ехать без Pi можно в ручном режиме с ноутбука по USB.",
            ha="center", fontsize=6.8, color="#5D6D7E", style="italic")
    fig.savefig(f"{OUT}/software.png", bbox_inches="tight", facecolor="white")
    plt.close(fig)


def layout():
    fig, ax = plt.subplots(figsize=(9, 5.4), dpi=170)
    ax.set_xlim(-10, 95); ax.set_ylim(-9, 58); ax.axis("off"); ax.set_aspect("equal")
    ax.text(42, 56, "Компоновка v2 — вид сверху (размеры в см, ориентировочно)", ha="center", fontsize=10, weight="bold")
    # корпус 35 x 30 (длина по x)
    ax.add_patch(Rectangle((10, 13), 35, 24, fc="#EAEDED", ec="#2C3E50", lw=1.6))
    # колёса D10 ширина 4
    for (x, y) in [(13, 9), (33, 9), (13, 37), (33, 37)]:
        ax.add_patch(Rectangle((x, y), 10, 4, fc="#566573", ec="#1C2833"))
    ax.text(27.5, 25, "АКБ + балласт\n(низко, между осями)", ha="center", va="center", fontsize=7)
    ax.add_patch(Rectangle((19, 20), 17, 10, fc="none", ec="#C0392B", ls="--"))
    ax.text(12, 34, "ESP32, драйверы", fontsize=6.5)
    ax.text(12, 15, "Raspberry Pi", fontsize=6.5)
    # отвал под углом 30°
    import math
    a = math.radians(30)
    cx, cy, L = 60, 25, 45
    # передний (ведущий) край сверху, задний — снизу: снег скользит назад-вбок и сходит с заднего края
    x1, y1 = cx + L / 2 * math.sin(a), cy + L / 2 * math.cos(a)
    x2, y2 = cx - L / 2 * math.sin(a), cy - L / 2 * math.cos(a)
    ax.plot([x1, x2], [y1, y2], color="#1F618D", lw=5, solid_capstyle="round")
    ax.text(x1 - 17, y1 + 1, "отвал 45 см\nпод 30°\n(захват 39 см)", fontsize=7, color="#1F618D")
    ax.plot([45, cx], [25, 25], color="#1F618D", lw=2)
    ax.text(50, 27, "рычаг +\nсерво", fontsize=6, color="#1F618D")
    ax.add_patch(FancyArrowPatch((x2 + 1, y2 + 2), (x2 - 4, y2 - 7), arrowstyle="-|>", mutation_scale=12, color="#85C1E9", lw=2))
    ax.text(x2 + 1, y2 - 5, "снег сходит с\nзаднего края вбок", fontsize=6.5, color="#2E86C1")
    # направление
    ax.add_patch(FancyArrowPatch((78, 30), (93, 30), arrowstyle="-|>", mutation_scale=14, color="black", lw=1.5))
    ax.text(85.5, 32, "движение", fontsize=7, ha="center")
    # датчики
    for (x, y, t) in [(44, 33, "УЗ-Л"), (44, 25, "УЗ-Ц"), (44, 17, "УЗ-П")]:
        ax.add_patch(Circle((x, y), 1.2, fc="#F5B041", ec="#7E5109"))
        ax.text(x - 6.5, y, t, fontsize=6)
    ax.text(66, 51, "УЗ-датчики и камера — на стойке\nвыше отвала (~20–30 см),\nиначе они «видят» сам отвал", fontsize=6.3, color="#7E5109")
    ax.add_patch(Rectangle((39, 27.5), 3, 4, fc="#AF7AC5", ec="#4A235A"))
    ax.text(36.5, 30, "кам.", fontsize=5.5, color="#4A235A", ha="right")
    # размеры
    ax.annotate("", (10, 2), (45, 2), arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.text(27.5, 0, "база 20 / длина 35", ha="center", fontsize=6.5)
    ax.annotate("", (5, 9), (5, 41), arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.text(3, 25, "колея 28\nширина 32", ha="right", va="center", fontsize=6.5)
    ax.text(42, -8, "База меньше колеи (20 < 28) — так легче поворачивать на месте при бортовом приводе.",
            ha="center", fontsize=6.8, style="italic", color="#5D6D7E")
    fig.savefig(f"{OUT}/layout.png", bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    software(); layout()  # electrical — см. electrical.dot (graphviz)
    print("ok")
