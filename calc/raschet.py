"""Расчёты для главы 3 эссе: кинематика, тяга и сопротивление, моторы, энергетика, отвал.

Все исходные данные собраны в PARAMS: меняйте их и перезапускайте
    python3 calc/raschet.py            # отчёт в консоль
    python3 calc/raschet.py --plots    # + графики в docs/img
Коэффициенты трения и плотность снега — ориентировочные диапазоны: их нужно
проверить опытом (см. «Опыт с динамометром» в документе).
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace

G = 9.81


@dataclass(frozen=True)
class Params:
    # --- робот ---
    mass_kg: float = 10.0           # полная масса с балластом (рекомендуем 8–12 кг)
    drive_share: float = 1.0        # доля веса на ведущих колёсах (4WD = 1.0; 2WD ≈ 0.5–0.6)
    n_motors: int = 4
    wheel_d_m: float = 0.100
    track_w_m: float = 0.28         # колея (между центрами колёс слева и справа)
    wheelbase_m: float = 0.20       # база (между осями)
    eta_drive: float = 0.90         # КПД передачи мотор → колесо (прямой вал)
    accel_mps2: float = 0.2         # разгон
    slope_deg: float = 2.0          # уклон дорожки
    # --- контакт со снегом/покрытием ---
    mu_traction: float = 0.30       # резина по снегу: 0.15 (рыхлый/накат) … 0.45 (влажный снег на асфальте)
    mu_lateral: float = 0.30        # боковое трение (для поворота на месте)
    c_roll: float = 0.08            # сопротивление качению малых колёс по снегу/слякоти
    # --- снег и отвал ---
    snow_h_m: float = 0.03
    snow_rho: float = 120.0         # свежий снег 50–200 кг/м³
    mu_pile: float = 0.5            # трение кома снега по снегу/покрытию
    blade_w_m: float = 0.45
    blade_angle_deg: float = 30.0   # угол отвала к поперечному направлению (0 = прямой)
    slide_factor: float = 1.5       # снег скользит по отвалу медленнее идеального
    safety: float = 1.5             # запас на неучтённое (уплотнение, неровности)
    # --- мотор (линейная модель DC-мотора) ---
    motor_name: str = "JGB37-520, 12 В, ~66–76 об/мин (1:90…1:131)"
    motor_n0_rpm: float = 70.0      # обороты холостого хода при 12 В
    motor_stall_kgcm: float = 5.0   # момент блокировки (консервативно; продавцы дают 5–8)
    motor_i0_a: float = 0.12
    motor_istall_a: float = 1.2
    motor_v: float = 12.0
    # --- скорость ---
    v_work_mps: float = 0.25
    # --- питание ---
    cells_s: int = 4
    cells_p: int = 1
    cell_ah: float = 3.4
    cell_v_nom: float = 3.6
    dod: float = 0.8                # используем 80 % ёмкости
    cold_factor: float = 0.7        # потеря ёмкости на морозе (−10 °C), ориентир
    p_esp32_w: float = 0.6
    p_rpi_w: float = 3.5            # Raspberry Pi 4 со средней загрузкой
    p_camera_w: float = 1.0
    p_sensors_w: float = 0.6
    eta_buck: float = 0.85
    motor_eta: float = 0.45         # КПД мотор-редуктора в рабочей точке
    # --- подъём отвала ---
    blade_mass_kg: float = 0.5
    blade_arm_m: float = 0.10       # плечо центра масс отвала относительно оси качания
    servo_kgcm: float = 9.4         # MG996R при 4,8 В (по даташиту), при 6 В ~11


def kgcm_to_nm(x: float) -> float:
    return x * G / 100.0


def nm_to_kgcm(x: float) -> float:
    return x * 100.0 / G


# ---------------- кинематика ----------------
def wheel_rpm(v_mps: float, d_m: float) -> float:
    return v_mps / (math.pi * d_m) * 60.0


def speed_from_rpm(rpm: float, d_m: float) -> float:
    return rpm / 60.0 * math.pi * d_m


# ---------------- силы ----------------
def traction_n(p: Params) -> float:
    """Максимальная сила тяги, которую колёса передадут без пробуксовки."""
    return p.mu_traction * p.mass_kg * G * p.drive_share * math.cos(math.radians(p.slope_deg))


def snow_per_m(p: Params) -> float:
    """Масса снега, которую отвал собирает на 1 м пути, кг/м."""
    a = math.radians(p.blade_angle_deg)
    return p.snow_rho * p.snow_h_m * p.blade_w_m * math.cos(a)


def pile_mass_angled(p: Params) -> float:
    """Снег, находящийся на наклонном отвале в установившемся режиме, кг.

    Снег, попав на отвал, скользит вдоль него со скоростью ≈ v·sin(α) и уходит
    с заднего края; в среднем проходит половину длины отвала. Отсюда
    m ≈ q · B / (2·sin α) · k_скольж.
    """
    a = math.radians(p.blade_angle_deg)
    if a < math.radians(5):
        return float("inf")
    return snow_per_m(p) * p.blade_w_m / (2 * math.sin(a)) * p.slide_factor


def pile_mass_straight(p: Params, dist_m: float) -> float:
    """Прямой отвал: снег копится, масса растёт с пройденным путём."""
    return p.snow_rho * p.snow_h_m * p.blade_w_m * dist_m


def plow_force_n(p: Params) -> float:
    return p.mu_pile * pile_mass_angled(p) * G


def resist_n(p: Params, with_plow: bool = True) -> dict:
    s = math.radians(p.slope_deg)
    parts = {
        "качение": p.c_roll * p.mass_kg * G * math.cos(s),
        "уклон": p.mass_kg * G * math.sin(s),
        "разгон": p.mass_kg * p.accel_mps2,
        "отвал": plow_force_n(p) if with_plow else 0.0,
    }
    parts["итого"] = sum(parts.values())
    parts["итого с запасом"] = parts["итого"] * p.safety
    return parts


def turn_force_per_side_n(p: Params) -> float:
    """Бортовой поворот на месте: сила на каждом борту, чтобы преодолеть боковое трение.

    Момент сопротивления M = μ_бок · m·g · L/2 (4 колеса, каждое на плече L/2);
    движущий момент F·W (бортa тянут в разные стороны) → F = μ m g L / (2W).
    """
    return p.mu_lateral * p.mass_kg * G * p.wheelbase_m / (2 * p.track_w_m)


# ---------------- мотор ----------------
def motor_point(p: Params, force_total_n: float) -> dict:
    """Рабочая точка одного мотора, если сила делится поровну между моторами."""
    r = p.wheel_d_m / 2
    tau = force_total_n * r / (p.n_motors * p.eta_drive)       # Н·м на валу мотора-редуктора
    tau_s = kgcm_to_nm(p.motor_stall_kgcm)
    frac = tau / tau_s
    rpm = p.motor_n0_rpm * max(0.0, 1 - frac)
    cur = p.motor_i0_a + (p.motor_istall_a - p.motor_i0_a) * min(frac, 1.0)
    return {
        "момент, кг·см": nm_to_kgcm(tau),
        "доля от момента блокировки": frac,
        "обороты, об/мин": rpm,
        "скорость, м/с": speed_from_rpm(rpm, p.wheel_d_m),
        "ток мотора, А": cur,
        "мощность мотора (электр.), Вт": cur * p.motor_v,
    }


def max_motor_force_n(p: Params, frac_of_stall: float = 0.6) -> float:
    """Сила на всех колёсах при работе мотора на заданной доле момента блокировки."""
    tau = kgcm_to_nm(p.motor_stall_kgcm) * frac_of_stall
    return tau * p.n_motors * p.eta_drive / (p.wheel_d_m / 2)


# ---------------- энергетика ----------------
def energy(p: Params, motor_force_n: float) -> dict:
    e_nom = p.cells_s * p.cells_p * p.cell_ah * p.cell_v_nom
    e_use = e_nom * p.dod
    e_cold = e_use * p.cold_factor
    p_mech = motor_force_n * p.v_work_mps
    p_motors = p_mech / p.motor_eta + p.n_motors * p.motor_i0_a * p.motor_v * 0.3  # +холостые потери (ШИМ ~30 %)
    p_logic = (p.p_esp32_w + p.p_rpi_w + p.p_camera_w + p.p_sensors_w) / p.eta_buck
    p_total = p_motors + p_logic
    return {
        "энергия АКБ номинальная, Вт·ч": e_nom,
        "доступно (80 %), Вт·ч": e_use,
        "доступно на морозе, Вт·ч": e_cold,
        "мощность моторов, Вт": p_motors,
        "мощность электроники, Вт": p_logic,
        "мощность всего, Вт": p_total,
        "время работы (тепло), ч": e_use / p_total,
        "время работы (мороз), ч": e_cold / p_total,
        "пиковый ток АКБ (все моторы в блокировке), А": p.n_motors * p.motor_istall_a + p_logic / (p.cells_s * 3.2),
    }


# ---------------- отвал ----------------
def blade(p: Params, outer_w_m: float = 0.32, margin_m: float = 0.025) -> dict:
    a = math.radians(p.blade_angle_deg)
    need_w = (outer_w_m + 2 * margin_m) / math.cos(a)
    servo_need = p.blade_mass_kg * p.blade_arm_m * 100  # кг·см
    return {
        "ширина захвата при угле α, м": p.blade_w_m * math.cos(a),
        "минимальная длина отвала, м": need_w,
        "высота отвала (≥ 3× глубины снега), м": max(0.12, 3 * p.snow_h_m),
        "момент для подъёма, кг·см": servo_need,
        "запас сервопривода, раз": p.servo_kgcm / servo_need,
    }


def scenario_table(p: Params) -> list[tuple]:
    """Сравнение: робот v1 (3 кг, 2WD) и варианты v2."""
    v1 = replace(p, mass_kg=3.0, drive_share=0.55, n_motors=2, blade_w_m=0.45, blade_angle_deg=30,
                 motor_name="JGA25-370 200 об/мин", motor_n0_rpm=200, motor_stall_kgcm=2.5 * 3)
    rows = []
    for name, q in [("v1: 3 кг, 2 ведущих", v1),
                    ("v2: 8 кг, 4 ведущих", replace(p, mass_kg=8.0)),
                    ("v2: 10 кг, 4 ведущих", replace(p, mass_kg=10.0)),
                    ("v2: 12 кг, 4 ведущих", replace(p, mass_kg=12.0))]:
        t = traction_n(q)
        r = resist_n(q)["итого с запасом"]
        rows.append((name, t, r, t - r))
    return rows


def depth_curve(p: Params, depths_cm, mass, share, mu=None):
    out = []
    for h in depths_cm:
        q = replace(p, snow_h_m=h / 100, mass_kg=mass, drive_share=share, mu_traction=mu or p.mu_traction)
        out.append((traction_n(q), resist_n(q)["итого с запасом"]))
    return out


def report(p: Params) -> str:
    L = []
    w = L.append
    w("=== 1. Кинематика ===")
    w(f"Колесо D = {p.wheel_d_m*1000:.0f} мм, окружность {math.pi*p.wheel_d_m:.3f} м")
    for v in (0.2, 0.25, 0.3, 0.6):
        w(f"  {v:.2f} м/с → {wheel_rpm(v, p.wheel_d_m):5.1f} об/мин колеса")
    w(f"  мотор {p.motor_n0_rpm:.0f} об/мин х.х. → {speed_from_rpm(p.motor_n0_rpm, p.wheel_d_m):.2f} м/с без нагрузки")
    w("")
    w("=== 2. Тяга и сопротивление (рабочий режим с отвалом) ===")
    w(f"Масса {p.mass_kg} кг, доля на ведущих {p.drive_share}, μ = {p.mu_traction}, уклон {p.slope_deg}°")
    t = traction_n(p)
    w(f"  Доступная тяга по сцеплению: {t:.1f} Н")
    w(f"  Снег h = {p.snow_h_m*100:.0f} см, ρ = {p.snow_rho:.0f} кг/м³ → собирается {snow_per_m(p):.2f} кг на 1 м пути")
    w(f"  Снег на наклонном отвале ({p.blade_angle_deg:.0f}°): ≈ {pile_mass_angled(p):.2f} кг")
    for k, v in resist_n(p).items():
        w(f"  {k:>18}: {v:6.1f} Н")
    w(f"  Запас по сцеплению: {t - resist_n(p)['итого с запасом']:.1f} Н")
    lim = t / (p.mu_pile * p.snow_rho * p.snow_h_m * p.blade_w_m * G)
    w(f"  Для сравнения: ПРЯМОЙ отвал упрётся в сцепление через ≈ {lim:.1f} м (снег копится)")
    w("")
    w("  Сравнение вариантов (Н): тяга / нужно с запасом / запас")
    for name, a, b, c in scenario_table(p):
        w(f"    {name:<22} {a:6.1f} / {b:6.1f} / {c:+6.1f}")
    w("")
    w("=== 3. Поворот на месте (бортовой) ===")
    for mu, surf in ((0.3, "снег"), (0.7, "сухой асфальт")):
        f = turn_force_per_side_n(replace(p, mu_lateral=mu))
        tau = f * p.wheel_d_m / 2 / (p.n_motors / 2) / p.eta_drive
        w(f"  {surf:<14}: {f:5.1f} Н на борт → {nm_to_kgcm(tau):.1f} кг·см на мотор "
          f"({nm_to_kgcm(tau)/p.motor_stall_kgcm*100:.0f} % от блокировки)")
    w("")
    w("=== 4. Мотор ===")
    w(f"  {p.motor_name}: блокировка {p.motor_stall_kgcm} кг·см, х.х. {p.motor_n0_rpm} об/мин")
    need = resist_n(p)["итого"]
    for label, f in (("рабочая точка (без запаса)", need), ("на пределе сцепления", t)):
        mp = motor_point(p, f)
        w(f"  {label}: " + ", ".join(f"{k} {v:.2f}" for k, v in mp.items()))
    w(f"  Сила от 4 моторов на 60 % блокировки: {max_motor_force_n(p):.1f} Н (сцепление {t:.1f} Н)")
    w("")
    w("=== 5. Энергетика ===")
    for k, v in energy(p, need).items():
        w(f"  {k}: {v:.1f}")
    w("")
    w("=== 6. Отвал ===")
    for k, v in blade(p).items():
        w(f"  {k}: {v:.2f}")
    return "\n".join(L)


def plots(p: Params, outdir: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = "DejaVu Sans"

    # --- график 1: тяга vs сопротивление по глубине снега ---
    depths = [i * 0.5 for i in range(1, 17)]
    fig, ax = plt.subplots(figsize=(8, 4.6), dpi=170)
    cases = [("v1: 3 кг, 2 ведущих", 3.0, 0.55, "#C0392B"),
             ("v2: 8 кг, 4 ведущих", 8.0, 1.0, "#2471A3"),
             ("v2: 10 кг, 4 ведущих", 10.0, 1.0, "#8E44AD"),
             ("v2: 12 кг, 4 ведущих", 12.0, 1.0, "#1E8449")]
    for name, m, sh, col in cases:
        d = depth_curve(p, depths, m, sh)
        ax.plot(depths, [x[0] for x in d], color=col, lw=2, label=f"{name}: тяга")
        ax.plot(depths, [x[1] for x in d], color=col, lw=2, ls="--", label=f"{name}: нужно")
    ax.set_xlabel("Глубина свежего снега, см")
    ax.set_ylabel("Сила, Н")
    ax.set_title(f"Тяга по сцеплению и сопротивление (μ = {p.mu_traction}, ρ = {p.snow_rho:.0f} кг/м³, отвал {p.blade_angle_deg:.0f}°, запас ×{p.safety})", fontsize=9)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=2, loc="upper left")
    ax.set_ylim(0, None)
    fig.tight_layout()
    fig.savefig(f"{outdir}/calc_traction.png")
    plt.close(fig)

    # --- график 2: характеристика мотора и рабочие точки ---
    fig, ax = plt.subplots(figsize=(8, 4.2), dpi=170)
    ts = p.motor_stall_kgcm
    for stall, n0, lab, col in ((ts, p.motor_n0_rpm, f"JGB37-520 ~{p.motor_n0_rpm:.0f} об/мин (блок. {ts} кг·см)", "#2471A3"),
                                (8.0, p.motor_n0_rpm, "то же, если блок. 8 кг·см (данные части продавцов)", "#85C1E9"),
                                (4.8, 107, "JGB37-520 107 об/мин (блок. 4,8 кг·см)", "#E67E22")):
        ax.plot([0, stall], [speed_from_rpm(n0, p.wheel_d_m), 0], color=col, lw=2, label=lab)
    need = resist_n(p)["итого"]
    for f, lab, mk in ((need, "работа с отвалом (3 см)", "o"), (traction_n(p), f"предел сцепления {p.mass_kg:.0f} кг", "s")):
        mp = motor_point(p, f)
        ax.plot(mp["момент, кг·см"], speed_from_rpm(p.motor_n0_rpm * max(0, 1 - mp["доля от момента блокировки"]), p.wheel_d_m),
                mk, color="black", ms=8, label=f"{lab}: {mp['момент, кг·см']:.1f} кг·см на мотор")
    ax.axhline(0.25, color="grey", ls=":", lw=1)
    ax.text(0.1, 0.26, "рабочая скорость 0,25 м/с", fontsize=7, color="grey")
    ax.set_xlabel("Момент на валу одного мотора, кг·см")
    ax.set_ylabel("Скорость робота, м/с (колесо D100)")
    ax.set_title("Характеристика мотора (линейная модель) и рабочие точки, 4 мотора", fontsize=9)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    ax.set_xlim(0, 9)
    ax.set_ylim(0, 0.65)
    fig.tight_layout()
    fig.savefig(f"{outdir}/calc_motor.png")
    plt.close(fig)


if __name__ == "__main__":
    P = Params()
    print(report(P))
    if "--plots" in sys.argv:
        import os
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "img")
        os.makedirs(out, exist_ok=True)
        plots(P, out)
        print("графики:", os.path.abspath(out))
