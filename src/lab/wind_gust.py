#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ПОРЫВЫ ВЕТРА: детерминированный профиль поверх постоянного WIND_SPD.

Плагин WindEffects (gz-sim-wind-effects-system) слушает runtime-топик
/world/<мир>/wind (gz.msgs.Wind) — вектор ветра меняется на лету, мир не
трогается (проверено сервисом /world/<мир>/wind_info: публикация принята).
Скрипт публикует СУММАРНЫЙ вектор: база (WIND_SPD/WIND_DIR_DEG из env
контейнера — ровно то, с чем плагин загрузился) + огибающая порыва
«1−cos фронт → плато → 1−cos спад». Расписание — в АБСОЛЮТНОМ sim-времени
(часы Gazebo /clock, t=0 = старт мира): два прогона с одним профилем получают
порывы в одни и те же sim-секунды → честный A/B против bag'а.

Запуск С ХОСТА (сам копирует себя в контейнер simulator и стартует там —
python-биндинги gz.transport13/gz.msgs10 есть только в нём):
    python3 src/lab/wind_gust.py "spd=12 at=60 rise=2 hold=5 fall=4 every=30"
    python3 src/lab/wind_gust.py --fg "spd=12 at=<t>"   # форграунд (отладка)
Спека можно не давать аргументом — возьмётся из env WIND_GUST (так его
запускает capture_scene.sh при заданном WIND_GUST).

Спека порыва (key=value через пробел; скорости м/с, времена sim-секунды):
    spd    — ПИК ветра в порыве (абсолютная величина, не добавка) — ОБЯЗАТЕЛЕН
    dir    — куда дует порыв, ° мировых осей (дефолт = базовый WIND_DIR_DEG);
             вектор порыва интерполируется от базового К ЦЕЛЕВОМУ (огибающей),
             так что порыв с другим dir крутит и направление
    at     — sim-время ПЕРВОГО порыва, с от старта Gazebo (дефолт 60; прогрев
             EKF ~44 с, взлёт после — дефолт бьёт в ранний полёт)
    rise   — фронт 1−cos, с (дефолт 2)
    hold   — плато, с (дефолт 5)
    fall   — спад 1−cos, с (дефолт 4)
    every  — период повторения, с (дефолт 0 = ОДИН порыв, скрипт сам выйдет;
             >0 — порыв каждые every секунд до убийства процесса)

Лог фаз — /root/output/wind_gust.log (хост: docker/sim/output/wind_gust.log),
freefly_lv.sh забирает его в архив прогона. Разбор отклика — по sim_t фаз
против истины /model/iris_cam/odometry в bag'е.

Смерть (SIGTERM/SIGINT, pkill -f wind_gust) — вежливая: публикует базу и
выходит; конечный профиль (every=0) возвращает базу и сам.

⚠️ Порыв проходит тот же квадратичный закон силы, что и база (WIND_FACTOR,
см. sim_up.sh): 5→10 м/с ≈ ×4 силы — это физично, factor не трогать.
⚠️ Гейт «физики висения» (vins_hover_v=3.0): сильный порыв на центральных
стиках честно разгоняет борт — гейт может уронить ярус (см. control.md).
"""

import math
import os
import re
import signal
import subprocess
import sys
import time

SIM = os.environ.get("SIM_CONTAINER", "p1317_simulator")
LOG_PATH = "/root/output/wind_gust.log"
RATE = 25.0          # Гц публикации внутри порыва (плагин просто хранит seed)

DEFAULTS = {"at": 60.0, "rise": 2.0, "hold": 5.0, "fall": 4.0, "every": 0.0}


def parse_spec(text):
    """key=value → dict; неизвестный ключ/мусор — падаем громко (тихий дефолт
    вместо опечатки уже терял свипы, см. урок белого списка в capture_scene)."""
    kv = dict(DEFAULTS)
    kv["dir"] = None
    for tok in text.split():
        if "=" not in tok:
            sys.exit("wind_gust: не key=value: %r (спека: %r)" % (tok, text))
        k, v = tok.split("=", 1)
        if k not in ("spd", "dir", "at", "rise", "hold", "fall", "every"):
            sys.exit("wind_gust: неизвестный ключ %r (спека: %r)" % (k, text))
        try:
            kv[k] = float(v)
        except ValueError:
            sys.exit("wind_gust: не число: %s=%r" % (k, v))
    if "spd" not in kv:
        sys.exit("wind_gust: обязателен spd=<пик м/с> (спека: %r)" % text)
    cyc = kv["rise"] + kv["hold"] + kv["fall"]
    if kv["every"] > 0 and kv["every"] < cyc:
        sys.exit("wind_gust: every=%g короче самого порыва (%g с)" % (kv["every"], cyc))
    return kv


# ── хост: копируем себя в контейнер simulator и запускаем там ────────────────
def host_main(spec, fg):
    me = os.path.abspath(__file__)
    subprocess.run(["docker", "cp", me, SIM + ":/tmp/wind_gust.py"], check=True)
    cmd = (["docker", "exec"] + ([] if fg else ["-d"])
           + [SIM, "python3", "/tmp/wind_gust.py", "--in-container", spec])
    if fg:
        os.execvp("docker", cmd)          # Ctrl+C доедет до скрипта → база
    subprocess.run(cmd, check=True)
    print("wind_gust: запущен в %s фоном (%s); лог docker/sim/output/wind_gust.log;"
          % (SIM, spec))
    print("wind_gust: стоп: docker exec %s pkill -f wind_gust" % SIM)


# ── контейнер: собственно публикатор ─────────────────────────────────────────
def in_container_main(spec):
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.wind_pb2 import Wind
    from gz.transport13 import Node

    g = parse_spec(spec)
    base_spd = float(os.environ.get("WIND_SPD", "0") or "0")
    base_dir = float(os.environ.get("WIND_DIR_DEG", "98") or "98")
    gdir = base_dir if g["dir"] is None else g["dir"]

    log_f = open(LOG_PATH, "a", buffering=1)

    def log(msg):
        line = "[wind_gust] %s" % msg
        print(line, flush=True)
        log_f.write(line + "\n")

    # Мир ищем по живому топику — имя мира не хардкодим. ЖДЁМ с ретраем:
    # capture_scene запускает нас сразу после `make wait` (готовность nav), а
    # Gazebo к этому моменту может ещё поднимать мир/плагины — мгновенный выход
    # «топика нет» убивал публикатор молча (E2E 2026-09-04, пустой лог).
    topic = None
    t0 = time.time()
    while topic is None:
        out = subprocess.run(["gz", "topic", "-l"],
                             capture_output=True, text=True).stdout
        m = re.search(r"^/world/([^/]+)/wind$", out, re.M)
        if m:
            topic = m.group(0)
            break
        if time.time() - t0 > 120:
            log("СТОП: топика /world/*/wind нет 120 с — плагин WindEffects не "
                "загружен (WIND_SPD=0 без WIND_GUST при старте контейнера?)")
            sys.exit(1)
        if time.time() - t0 < 3:
            log("топика /world/*/wind ещё нет — жду Gazebo...")
        time.sleep(2)

    br = math.radians(base_dir)
    gr = math.radians(gdir)
    bx, by = base_spd * math.cos(br), base_spd * math.sin(br)
    gx, gy = g["spd"] * math.cos(gr), g["spd"] * math.sin(gr)

    node = Node()
    pub = node.advertise(topic, Wind)
    sim_t = [None]
    node.subscribe(Clock, "/clock", lambda msg: sim_t.__setitem__(
        0, msg.sim.sec + msg.sim.nsec * 1e-9))

    stop = [False]
    for sg in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sg, lambda *_: stop.__setitem__(0, True))

    def send(vx, vy):
        w = Wind()
        w.linear_velocity.x = vx
        w.linear_velocity.y = vy
        w.enable_wind = True
        pub.publish(w)

    def envelope(t):
        """0..1 в sim-времени t; None = база (вне порыва)."""
        if t < g["at"]:
            return None
        ph = t - g["at"]
        if g["every"] > 0:
            ph %= g["every"]
        cyc = g["rise"] + g["hold"] + g["fall"]
        if ph >= cyc:
            return None
        if ph < g["rise"]:
            return 0.5 * (1 - math.cos(math.pi * ph / g["rise"])) if g["rise"] > 0 else 1.0
        if ph < g["rise"] + g["hold"]:
            return 1.0
        f = (ph - g["rise"] - g["hold"]) / g["fall"] if g["fall"] > 0 else 1.0
        return 0.5 * (1 + math.cos(math.pi * f))

    log("профиль: %s | база %.1f м/с @%.0f° | порыв %.1f м/с @%.0f° | топик %s"
        % (spec, base_spd, base_dir, g["spd"], gdir, topic))

    t0 = time.time()
    while sim_t[0] is None:
        if time.time() - t0 > 30:
            sys.exit("wind_gust: /clock молчит 30 с — Gazebo не бежит?")
        time.sleep(0.1)
    send(bx, by)                          # синхронизация: seed = честная база
    log("sim_t=%.1f старт, база опубликована; первый порыв at=%.0f" % (sim_t[0], g["at"]))

    active = False
    last_log = 0.0
    while not stop[0]:
        t = sim_t[0]
        e = envelope(t)
        if e is not None:
            if not active:
                log("sim_t=%.2f ПОРЫВ: фронт %.1f с → %.1f м/с" % (t, g["rise"], g["spd"]))
                active, last_log = True, t
            send(bx + e * (gx - bx), by + e * (gy - by))
            if t - last_log >= 1.0:
                v = math.hypot(bx + e * (gx - bx), by + e * (gy - by))
                log("sim_t=%.2f |v|=%.1f м/с (e=%.2f)" % (t, v, e))
                last_log = t
        elif active:
            send(bx, by)                  # выход из порыва — точная база
            log("sim_t=%.2f порыв кончился, база %.1f м/с" % (t, base_spd))
            active = False
            if g["every"] <= 0:
                log("профиль конечный (every=0) — выхожу")
                return
        time.sleep(1.0 / RATE)

    send(bx, by)
    log("sim_t=%.2f остановлен (сигнал), база %.1f м/с восстановлена"
        % (sim_t[0], base_spd))


def main():
    args = sys.argv[1:]
    fg = "--fg" in args
    inc = "--in-container" in args
    args = [a for a in args if a not in ("--fg", "--in-container")]
    spec = args[0] if args else os.environ.get("WIND_GUST", "")
    if not spec.strip():
        sys.exit("wind_gust: дай спеку аргументом или env WIND_GUST "
                 "(пример: \"spd=12 at=60 rise=2 hold=5 fall=4 every=30\")")
    parse_spec(spec)                      # валидация ДО запуска в контейнере
    if inc:
        in_container_main(spec)
    else:
        host_main(spec, fg)


if __name__ == "__main__":
    main()
