#!/usr/bin/env python3
"""
scale_wildthumper.py - масштабує модель WildThumper (SITL_Models) для Gazebo.

Що робить:
  * масштабує геометрію: позиції (<pose>), розміри collision (<size>, <radius>, <length>),
    масштаб 3D-мешів (<mesh><scale>)
  * задає нову загальну масу і перераховує інерцію кожної ланки (I ~ m * L^2)
  * масштабує підвіску (пружини, демпфери, ліміти моменту)
  * перераховує керування колесами в ArduPilotPlugin:
      - multiplier: щоб повний газ давав задану макс. швидкість
      - PID і обмеження моменту на колесі
  * створює нову модель (окрема папка), оригінал не змінюється

Приклад:
  python3 scale_wildthumper.py \
      --src ~/SITL_Models/Gazebo/models/wildthumper \
      --dst ~/my_gz_models/wildthumper_xl \
      --length 2.0 --mass 350 --vmax-kmh 45 --wheel-torque 250
"""
import argparse
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET

# Габарити оригінальної моделі (з model.sdf):
# осі коліс на x = +-0.15 м, радіус колеса 0.062 м -> довжина 0.30 + 2*0.062
ORIG_LENGTH = 0.30 + 2 * 0.062   # 0.424 м
ORIG_WHEEL_RADIUS = 0.062        # м
WHEEL_JOINT_SUFFIX = "_wheel_joint"


def fnum(x):
    """Компактний запис числа для SDF."""
    return f"{x:.6g}"


def scale_vec_text(text, factors):
    vals = text.split()
    out = []
    for i, v in enumerate(vals):
        f = factors[i] if i < len(factors) else 1.0
        out.append(fnum(float(v) * f))
    return " ".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", required=True, help="папка оригінальної моделі wildthumper")
    ap.add_argument("--dst", required=True, help="папка нової моделі (буде створена)")
    ap.add_argument("--name", default=None, help="ім'я нової моделі (за замовчуванням = назва папки dst)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--length", type=float, help="цільова габаритна довжина, м")
    g.add_argument("--scale", type=float, help="коефіцієнт масштабу (замість --length)")
    ap.add_argument("--mass", type=float, required=True, help="цільова загальна маса, кг")
    ap.add_argument("--vmax-kmh", type=float, required=True,
                    help="швидкість при повному газі, км/год")
    ap.add_argument("--wheel-torque", type=float, default=250.0,
                    help="макс. момент на одне колесо, Н*м (обмеження PID)")
    ap.add_argument("--wheel-damping", type=float, default=0.3,
                    help="демпфування в осі колеса (втрати в підшипниках/редукторі), Н*м*с/рад")
    ap.add_argument("--extra-visuals", default=None,
                    help="XML-файл з додатковими <visual> (лише графіка, без фізики)")
    ap.add_argument("--extra-sensors", default=None,
                    help="XML-файл з <sensor> (лідар, камери), кріпляться до --visual-link")
    ap.add_argument("--visual-link", default="base_link",
                    help="ланка, до якої кріпляться додаткові visual (за замовч. base_link)")
    ap.add_argument("--mu", type=float, default=None,
                    help="тертя шин mu (за замовчуванням як в оригіналі)")
    ap.add_argument("--mu2", type=float, default=None,
                    help="тертя шин mu2 (за замовчуванням як в оригіналі)")
    args = ap.parse_args()

    src = os.path.expanduser(args.src)
    dst = os.path.expanduser(args.dst)
    name = args.name or os.path.basename(os.path.normpath(dst))
    s = args.scale if args.scale else args.length / ORIG_LENGTH

    # зберігаємо коментарі з оригінального файлу
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(os.path.join(src, "model.sdf"), parser=parser)
    root = tree.getroot()
    model = root.find("model")

    # ---- маса: стара сума -> коефіцієнт ----
    masses = model.findall(".//inertial/mass")
    m_old = sum(float(m.text) for m in masses)
    m_ratio = args.mass / m_old
    # коефіцієнт моментів (сил * плече): M ~ m * g * L
    k_torque = m_ratio * s
    # характерний час динаміки ~ sqrt(L/g)
    k_time = math.sqrt(s)

    # ---- 1. позиції ----
    for pose in model.iter("pose"):
        if pose.text and pose.text.strip():
            pose.text = scale_vec_text(pose.text, [s, s, s, 1, 1, 1])

    # ---- 2. геометрія ----
    for geom in model.iter("geometry"):
        for box_size in geom.findall("./box/size"):
            box_size.text = scale_vec_text(box_size.text, [s, s, s])
        for tag in ("radius", "length"):
            for el in geom.findall(f"./cylinder/{tag}"):
                el.text = fnum(float(el.text) * s)
            for el in geom.findall(f"./sphere/{tag}"):
                el.text = fnum(float(el.text) * s)
        for sc in geom.findall("./mesh/scale"):
            sc.text = scale_vec_text(sc.text, [s, s, s])

    # ---- 3. маса й інерція ----
    for inertial in model.iter("inertial"):
        m = inertial.find("mass")
        if m is not None:
            m.text = fnum(float(m.text) * m_ratio)
        inertia = inertial.find("inertia")
        if inertia is not None:
            for comp in inertia:
                comp.text = fnum(float(comp.text) * m_ratio * s * s)

    # ---- 4. шарніри (підвіска і колеса) ----
    for joint in model.iter("joint"):
        axis = joint.find("axis")
        if axis is None:
            continue
        dyn = axis.find("dynamics")
        is_wheel = joint.get("name", "").endswith(WHEEL_JOINT_SUFFIX)
        if dyn is not None and is_wheel:
            # колесо: демпфування = лише втрати в осі; масштабувати його не можна,
            # бо разом із моментом росте й швидкість обертання -> "гальмо" на швидкості
            el = dyn.find("damping")
            if el is not None:
                el.text = fnum(args.wheel_damping)
        elif dyn is not None:
            for tag, k in (("spring_stiffness", k_torque),
                           ("damping", k_torque * k_time),
                           ("friction", k_torque)):
                el = dyn.find(tag)
                if el is not None:
                    el.text = fnum(float(el.text) * k)
        lim = axis.find("limit")
        if lim is not None:
            el = lim.find("effort")
            if el is not None:
                el.text = fnum(float(el.text) * k_torque)
            el = lim.find("velocity")
            if el is not None:
                el.text = fnum(float(el.text) / k_time)

    # ---- 5. тертя шин ----
    for fr in model.iter("ode"):
        mu = fr.find("mu")
        mu2 = fr.find("mu2")
        if mu is not None and args.mu is not None:
            mu.text = fnum(args.mu)
        if mu2 is not None and args.mu2 is not None:
            mu2.text = fnum(args.mu2)

    # ---- 6. ArduPilotPlugin: керування колесами ----
    r_new = ORIG_WHEEL_RADIUS * s
    v_max = args.vmax_kmh / 3.6
    w_max = v_max / r_new                 # рад/с при повному газі
    # PWM [1000..2000] -> [0..1] + offset(-0.5) -> [-0.5..0.5] * multiplier
    mult_abs = 2.0 * w_max
    n_controls = 0
    for plugin in model.iter("plugin"):
        if plugin.get("filename") != "ArduPilotPlugin":
            continue
        for ctrl in plugin.findall("control"):
            jn = ctrl.findtext("jointName", "")
            if not jn.endswith(WHEEL_JOINT_SUFFIX):
                continue
            n_controls += 1
            mul = ctrl.find("multiplier")
            sign = -1.0 if float(mul.text) < 0 else 1.0
            mul.text = fnum(sign * mult_abs)
            for tag in ("p_gain", "i_gain", "d_gain", "i_max", "i_min"):
                el = ctrl.find(tag)
                if el is not None:
                    el.text = fnum(float(el.text) * k_torque)
            cmax = ctrl.find("cmd_max")
            cmin = ctrl.find("cmd_min")
            if cmax is not None:
                cmax.text = fnum(args.wheel_torque)
            if cmin is not None:
                cmin.text = fnum(-args.wheel_torque)

    # ---- 7. додаткові візуальні деталі (після масштабування: задаються в метрах XL) ----
    n_vis = 0
    if args.extra_visuals:
        vparser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        vroot = ET.parse(os.path.expanduser(args.extra_visuals), parser=vparser).getroot()
        link = None
        for l in model.findall("link"):
            if l.get("name") == args.visual_link:
                link = l
        if link is None:
            raise SystemExit(f"Ланку '{args.visual_link}' не знайдено в моделі")
        for v in vroot.findall("visual"):
            if v.find("collision") is not None or v.find("inertial") is not None:
                raise SystemExit("У --extra-visuals дозволені лише елементи <visual>")
            link.append(v)
            n_vis += 1

    # ---- 7b. сенсори (лідар, камери) - у тій самій ланці, координати в метрах XL ----
    n_sens = 0
    if args.extra_sensors:
        sparser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        sroot = ET.parse(os.path.expanduser(args.extra_sensors), parser=sparser).getroot()
        link = None
        for l in model.findall("link"):
            if l.get("name") == args.visual_link:
                link = l
        if link is None:
            raise SystemExit(f"Ланку '{args.visual_link}' не знайдено в моделі")
        for sn in sroot.findall("sensor"):
            link.append(sn)
            n_sens += 1

    # ---- 8. ім'я моделі ----
    model.set("name", name)

    # ---- запис ----
    os.makedirs(dst, exist_ok=True)
    out_sdf = os.path.join(dst, "model.sdf")
    tree.write(out_sdf, encoding="unicode", xml_declaration=False)
    with open(out_sdf, "r+", encoding="utf-8") as fh:
        body = fh.read()
        fh.seek(0)
        fh.write("<?xml version='1.0'?>\n" + body)
        fh.truncate()

    cfg_src = os.path.join(src, "model.config")
    cfg_dst = os.path.join(dst, "model.config")
    if os.path.exists(cfg_src):
        cfg = open(cfg_src, encoding="utf-8").read()
        cfg = re.sub(r"<name>.*?</name>", f"<name>{name}</name>", cfg, count=1)
        cfg = re.sub(r"<description>.*?</description>",
                     f"<description>WildThumper scaled x{s:.3f}, {args.mass:g} kg, "
                     f"{args.vmax_kmh:g} km/h</description>", cfg, count=1, flags=re.S)
        open(cfg_dst, "w", encoding="utf-8").write(cfg)

    # ---- звіт ----
    print(f"Модель:            {name}  ->  {out_sdf}")
    print(f"Масштаб s:         {s:.3f}")
    print(f"Довжина / ширина:  {ORIG_LENGTH * s:.2f} м / {(0.24 + 0.062) * s:.2f} м")
    print(f"Колісна база:      {0.30 * s:.2f} м,  колія {0.24 * s:.2f} м")
    print(f"Колесо:            D = {2 * r_new:.3f} м, ширина {0.062 * s:.3f} м")
    print(f"Кліренс (оцінка):  {0.042 * s:.2f} м")
    print(f"Маса:              {m_old:.2f} кг -> {args.mass:g} кг (x{m_ratio:.1f})")
    print(f"Коеф. моментів:    x{k_torque:.1f}")
    print(f"Макс. швидкість:   {args.vmax_kmh:g} км/год = {v_max:.2f} м/с,"
          f" колесо {w_max:.1f} рад/с, multiplier = {mult_abs:.2f}")
    print(f"Момент на колесо:  <= {args.wheel_torque:g} Н*м"
          f" (сумарна тяга <= {6 * args.wheel_torque / r_new:.0f} Н)")
    loss = args.wheel_damping * w_max
    print(f"Втрати в осі коліс: {args.wheel_damping:g} Н*м*с/рад -> {loss:.0f} Н*м на макс. швидкості"
          f" ({100 * loss / args.wheel_torque:.0f}% від ліміту моменту)")
    print(f"Колеса в плагіні:  {n_controls} шт.")
    if args.extra_visuals:
        print(f"Додаткові visual:  {n_vis} шт. -> ланка '{args.visual_link}'")
    if args.extra_sensors:
        print(f"Сенсори:           {n_sens} шт. -> ланка '{args.visual_link}'")
    if n_controls != 6:
        print("УВАГА: очікувалося 6 коліс - перевірте jointName у плагіні")


if __name__ == "__main__":
    main()
