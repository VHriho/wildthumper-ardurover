#!/usr/bin/env python3
"""
lidar_to_mavlink.py - міст "лідар Gazebo -> ArduPilot" без ROS.

Підписується на 3D-скан Gazebo (/xl/lidar, gz.msgs.LaserScan), відкидає землю,
небо й корпус ровера, зводить скан у 72 сектори по 5 град і надсилає в ArduPilot
повідомлення MAVLink OBSTACLE_DISTANCE (10 Гц). ArduPilot приймає його як
датчик близькості PRX1_TYPE = 2 (MAVLink) і використовує для AC_Avoid / BendyRuler.

Запуск (venv-ardupilot активний, Gazebo і SITL уже працюють):
    python3 lidar_to_mavlink.py
    python3 lidar_to_mavlink.py --mav tcp:127.0.0.1:5762 --max-range 20

Потрібно: python3-gz-transport13, python3-gz-msgs10 (apt), pymavlink (є у venv).
"""
import argparse
import math
import threading
import time

from pymavlink import mavutil

N_SECT = 72          # 72 x 5 град = 360
INC = 5              # ширина сектора, град


def compute_sectors(scan, a):
    """LaserScan -> список з 72 відстаней (м або None), сектор 0 = вперед, кут за годинниковою.

    Фільтр землі з урахуванням рельєфу: для кожного горизонтального напрямку промені
    перебираються знизу вгору, а міст "веде" поверхню землі від точки під ровером.
    Точка, до якої від попередньої точки землі підйом не крутіший за --max-slope
    (+ допуск --ground-clear), вважається землею (схил, горб, канава).
    Різкий підйом (стіна, стовбур, куб) - перешкода, якщо вона вища за --ground-clear
    над місцевою землею і не вища за --max-height (гілки, дроти)."""
    best = [None] * N_SECT
    hc = scan.count
    vc = max(scan.vertical_count, 1)
    hs = scan.angle_step
    vs = scan.vertical_angle_step if vc > 1 else 0.0
    rmin = max(scan.range_min, a.min_range)
    rmax = scan.range_max
    ranges = scan.ranges
    x0, x1, y0, y1 = a.self_box
    tan_slope = math.tan(math.radians(a.max_slope))
    els = [scan.vertical_angle_min + v * vs for v in range(vc)]
    order = sorted(range(vc), key=lambda v: els[v])          # знизу вгору
    trig = [(math.cos(e), math.sin(e)) for e in els]
    tan_steep = math.tan(math.radians(a.steep_angle))
    for h in range(hc):
        az = scan.angle_min + h * hs
        ca, sa = math.cos(az), math.sin(az)
        pts = []                                            # (d, z) знизу вгору
        for v in order:
            r = ranges[v * hc + h]
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            ce, se = trig[v]
            pts.append((r * ce, a.lidar_height + r * se))
        n = len(pts)
        # крутий відрізок між сусідніми точками = вертикальна поверхня (стіна, стовбур, куб)
        steep_seg = [False] * max(n - 1, 0)
        for i in range(n - 1):
            dd = pts[i + 1][0] - pts[i][0]
            dz = pts[i + 1][1] - pts[i][1]
            steep_seg[i] = dz > 0.05 and (dd <= 0.0 or dz > dd * tan_steep)
        gd, gz = 0.0, 0.0                                   # земля під ровером
        for i, (d, z) in enumerate(pts):
            steep = (i > 0 and steep_seg[i - 1]) or (i < n - 1 and steep_seg[i])
            dd = d - gd
            # земля: не на крутій поверхні і підйом від попередньої землі не крутіший за max_slope
            if not steep and dd > 0.05 and z - gz <= dd * tan_slope + a.ground_clear:
                gd, gz = d, z
                continue
            hz = z - gz                                     # висота над місцевою землею
            if hz < a.ground_clear or hz > a.max_height or d > a.max_range:
                continue
            x, y = d * ca, d * sa
            if x0 <= x <= x1 and y0 <= y <= y1:
                continue                                    # власний корпус
            # Gazebo: кут проти годинникової (вліво +). MAVLink FRD: за годинниковою.
            brg = (-math.degrees(math.atan2(y, x))) % 360.0
            s = int((brg + INC / 2) // INC) % N_SECT
            if best[s] is None or d < best[s]:
                best[s] = d
    return best


def confirm(history):
    """Перешкода в секторі лише якщо вона є в усіх останніх сканах (прибирає поодинокі хибні)."""
    out = []
    for s in range(N_SECT):
        vals = [hst[s] for hst in history]
        out.append(None if any(v is None for v in vals) else max(vals))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/xl/lidar")
    ap.add_argument("--mav", default="tcp:127.0.0.1:5762",
                    help="з'єднання з SITL (5762 = SERIAL1, MAVLink)")
    ap.add_argument("--lidar-height", type=float, default=1.74, help="висота лідара над землею, м")
    ap.add_argument("--ground-clear", type=float, default=0.30,
                    help="допуск землі: нижче цієї висоти над місцевою землею - не перешкода, м")
    ap.add_argument("--max-height", type=float, default=2.20,
                    help="вище над місцевою землею - не перешкода для ровера (гілки, дроти), м")
    ap.add_argument("--max-slope", type=float, default=15.0,
                    help="найкрутіший схил, який вважається землею, град")
    ap.add_argument("--steep-angle", type=float, default=40.0,
                    help="відрізок між сусідніми променями крутіший за це - вертикальна поверхня, град")
    ap.add_argument("--confirm", type=int, default=2,
                    help="у скількох сканах поспіль має бути перешкода (1 = без фільтра)")
    ap.add_argument("--min-range", type=float, default=0.5)
    ap.add_argument("--max-range", type=float, default=20.0,
                    help="далі не беремо (на нерівному рельєфі далека земля дає хибні 'перешкоди')")
    ap.add_argument("--self-box", type=float, nargs=4, default=[-1.55, 0.70, -0.85, 0.85],
                    metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                    help="прямокутник корпусу відносно лідара (X вперед, Y вліво), м")
    ap.add_argument("--rate", type=float, default=10.0)
    a = ap.parse_args()

    # gz-transport (імпорт тут, щоб --help працював і без нього)
    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan

    lock = threading.Lock()
    state = {"scan": None, "n": 0}
    history = []
    last_n = -1

    def cb(msg):
        with lock:
            state["scan"] = msg
            state["n"] += 1

    node = Node()
    if not node.subscribe(LaserScan, a.topic, cb):
        raise SystemExit(f"Не вдалося підписатися на {a.topic}")
    print(f"Підписано на {a.topic}")

    print(f"З'єднання з ArduPilot: {a.mav} ...")
    mav = mavutil.mavlink_connection(a.mav, source_system=1, source_component=196,
                                     autoreconnect=True)
    if mav.wait_heartbeat(timeout=30) is None:
        raise SystemExit("ArduPilot не відповідає (SITL запущено?)")
    print(f"ArduPilot відповів (sysid {mav.target_system})")
    t_ap = time.time()          # час останнього HEARTBEAT від автопілота
    ap_lost = False

    min_cm = int(a.min_range * 100)
    max_cm = int(a.max_range * 100)
    period = 1.0 / a.rate
    t_hb = t_print = 0.0
    while True:
        t0 = time.time()
        # читаємо (і відкидаємо) вхідну телеметрію, стежимо за HEARTBEAT автопілота
        while True:
            m = mav.recv_match(blocking=False)
            if m is None:
                break
            if (m.get_type() == "HEARTBEAT" and m.get_srcComponent() == 1
                    and m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                t_ap = t0
                if ap_lost:
                    print("ArduPilot знову на зв'язку")
                    ap_lost = False
        if t0 - t_ap > 3.0 and not ap_lost:
            print("УВАГА: немає HEARTBEAT від ArduPilot > 3 с - дані лідара не доходять "
                  "(SITL перезапущено? міст перепідключається)")
            ap_lost = True
        if t0 - t_hb > 1.0:
            mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                   mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            t_hb = t0
        with lock:
            scan = state["scan"]
            n = state["n"]
        if scan is not None:
            if n != last_n:                     # новий скан -> оновити історію
                history.append(compute_sectors(scan, a))
                history = history[-max(1, a.confirm):]
                last_n = n
            best = confirm(history)
            dist = [max_cm + 1 if b is None else max(min_cm, min(int(b * 100), max_cm))
                    for b in best]
            mav.mav.obstacle_distance_send(
                int(t0 * 1e6), mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, dist,
                INC, min_cm, max_cm, 0.0, 0.0, mavutil.mavlink.MAV_FRAME_BODY_FRD)
            if t0 - t_print > 2.0:
                valid = [(b, i * INC) for i, b in enumerate(best) if b is not None]
                if valid:
                    b, ang = min(valid)
                    print(f"сканів: {state['n']:6d}  найближча перешкода: {b:5.1f} м, кут {ang:3d} град "
                          f"(0 = вперед, за годинниковою), секторів з перешкодами: {len(valid)}")
                else:
                    print(f"сканів: {state['n']:6d}  перешкод у межах {a.max_range:.0f} м немає")
                t_print = t0
        time.sleep(max(0.0, period - (time.time() - t0)))


if __name__ == "__main__":
    main()
