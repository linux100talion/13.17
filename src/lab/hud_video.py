#!/usr/bin/env python3
"""Пост-рендер debug-HUD на видео из rosbag → scene_hud.mp4.

HUD живёт только в FPV-потоке OpenHD :5600, который никем не записывается;
scene.mp4 остаётся ЧИСТЫМ потоком камеры («как видела камера» — вход нейросетей
и анализа сцены, оверлей там был бы порчей данных). Этот скрипт восстанавливает
оверлей из записанных топиков ТЕМ ЖЕ кодом, что рисует живой поток
(nav_pkg/hud_renderer.py, импорт из bind-mounted исходников — состояние репы,
не устаревший colcon-install) → кадры совпадают с тем, что видел пилот.

Честная деградация: чего нет в bag — того нет и на пост-рендере (как у живого
HUD с молчащим источником). В freefly-серии пишутся /mission/status, /odometry,
/mavros/state, /flow_dbg, /flow_dbg2 (+/feature с 2026-08-23); /nn1/drift и
/nn2/scene появятся, когда поедут в TOPICS_EXTRA.

ВРЕМЯ: рендерер меряет возрасты «часами вызывающего». Здесь это sim-время:
у стемпованных сообщений — header.stamp, у /mission/status (String, без
header) — sim-время последнего стемпованного сообщения до него в порядке
записи (ошибка ≤ интервала кадра, пороги HUD 0.5–3 с её не чувствуют).
Кадры рисуются на своём header.stamp → RTF-независимо, как в make_video.py.

Память: кадры НЕ копятся (make_video.py держит весь полёт в RAM — 4361 кадр
960×540 ≈ 6.8 ГБ); fps меряется по первым кадрам, дальше — потоковая запись.

Запускается ВНУТРИ nav-контейнера (нужен cv_bridge из /opt/overlay):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash; \
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash; \
    python3 /lab/hud_video.py'
Параметры через env:
  SCENE_BAG      путь к rosbag     (default /root/sim_ws/output/scene_bag)
  SCENE_HUD_MP4  выходной файл     (default /root/sim_ws/output/scene_img/scene_hud.mp4)
  SCENE_TOPIC    топик изображения (default /image_color)
  SCENE_FPS      FPS видео; 0 = авто по первым кадрам (default 0)
  SCENE_MAXW     макс. ширина кадра, px; 0 = не масштабировать (default 1280)
  SCENE_FEAT_DOTS точки фич трекера на кадре; 0 = только счётчик FEAT (default 1)
"""
import os
import sys

import cv2
import rosbag2_py
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import String

# рендерер — из исходников (bind mount), не из colcon-install: пост-рендер
# должен совпадать с тем, что закоммичено, даже до пересборки контейнера
sys.path.insert(0, "/root/sim_ws/src/nav")
from nav_pkg.hud_renderer import HudRenderer                       # noqa: E402

BAG = os.environ.get("SCENE_BAG", "/root/sim_ws/output/scene_bag")
MP4 = os.environ.get("SCENE_HUD_MP4", "/root/sim_ws/output/scene_img/scene_hud.mp4")
TOPIC = os.environ.get("SCENE_TOPIC", "/image_color")
FPS_ENV = float(os.environ.get("SCENE_FPS", "0"))
MAXW = int(os.environ.get("SCENE_MAXW", "1280"))
FEAT_DOTS = os.environ.get("SCENE_FEAT_DOTS", "1") == "1"
FPS_PROBE_N = 90          # кадров на авто-оценку fps (~3 sim-с при 30 Гц)


def stamp(m) -> float:
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    os.makedirs(os.path.dirname(MP4), exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    have = {t.name for t in reader.get_all_topics_and_types()}

    # источники HUD → (тип, обработчик); отсутствующие в bag просто выпадают
    hud = HudRenderer()
    now_sim = [0.0]        # sim-часы реплея: последний виденный header.stamp
    scale = [1.0]          # кадр/камера после MAXW-даунскейла (для точек фич)

    def on_status(m):
        hud.set_status(m.data, now_sim[0])       # String без header — см. докстринг

    def on_odom(m):
        now_sim[0] = stamp(m)
        hud.add_odom(now_sim[0])

    def on_state(m):
        now_sim[0] = stamp(m)
        hud.set_state(m.mode, m.armed, now_sim[0])

    def on_feat(m):
        now_sim[0] = stamp(m)
        # каналы feature_tracker: [id, u, v, vx, vy]; u,v — пиксели кадра
        # камеры; при даунскейле MAXW домножаем на коэффициент кадра
        pts = None
        if FEAT_DOTS and len(m.channels) >= 3:
            s = scale[0]
            pts = [(u * s, v * s) for u, v in
                   zip(m.channels[1].values, m.channels[2].values)]
        hud.set_feat(len(m.points), now_sim[0], pts)

    def on_roll(m):
        now_sim[0] = stamp(m)
        hud.set_cmd_roll(m.vector.x, now_sim[0])

    def on_pitch(m):
        now_sim[0] = stamp(m)
        hud.set_cmd_pitch(m.vector.x)

    def on_drift(m):
        now_sim[0] = stamp(m)
        hud.set_drift(m.vector.x, m.vector.y, m.vector.z, now_sim[0])

    handlers = {
        "/mission/status": (String, on_status),
        "/odometry": (Odometry, on_odom),
        "/mavros/state": (None, on_state),       # тип лениво: mavros_msgs опционален
        "/feature": (PointCloud, on_feat),
        "/feature_tracker/feature": (PointCloud, on_feat),
        "/flow_dbg": (Vector3Stamped, on_roll),
        "/flow_dbg2": (Vector3Stamped, on_pitch),
        "/nn1/drift": (Vector3Stamped, on_drift),
        "/nn2/scene": (String, lambda m: hud.set_scene(m.data)),
    }
    if "/mavros/state" in have:
        from mavros_msgs.msg import State
        handlers["/mavros/state"] = (State, on_state)
    missing = [t for t in handlers if t not in have]
    if missing:
        print(f"⚠️ в bag нет {' '.join(sorted(missing))} — этих строк HUD не будет")
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=[TOPIC] + [t for t in handlers if t in have]))

    bridge = CvBridge()
    writer = None
    probe = []             # (t_sim, кадр) до оценки fps — потом поток
    n_frames, t0, t_last = 0, None, None

    def render(img_msg):
        nonlocal t_last
        t = stamp(img_msg)
        now_sim[0] = t
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        if MAXW > 0 and img.shape[1] > MAXW:
            h = int(img.shape[0] * MAXW / img.shape[1])
            scale[0] = MAXW / img.shape[1]
            img = cv2.resize(img, (MAXW, h), interpolation=cv2.INTER_AREA)
        hud.draw(img, t)
        t_last = t
        return t, img

    def open_writer(fps, size):
        w = cv2.VideoWriter(MP4, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if not w.isOpened():
            raise SystemExit(f"⚠️ VideoWriter не открылся для {MP4} "
                             f"(codec mp4v / size {size})")
        return w

    while reader.has_next():
        topic, data, _bag_t = reader.read_next()
        if topic != TOPIC:
            typ, fn = handlers[topic]
            fn(deserialize_message(data, typ))
            continue
        t, img = render(deserialize_message(data, Image))
        n_frames += 1
        if t0 is None:
            t0 = t
        if writer is None:
            probe.append((t, img))
            if len(probe) >= FPS_PROBE_N:
                span = probe[-1][0] - probe[0][0]
                fps = FPS_ENV if FPS_ENV > 0 else \
                    max(1.0, min((len(probe) - 1) / span if span > 0 else 10.0, 60.0))
                writer = open_writer(fps, (img.shape[1], img.shape[0]))
                for _, fr in probe:
                    writer.write(fr)
                probe.clear()
        else:
            writer.write(img)

    if n_frames == 0:
        raise SystemExit(f"⚠️ в bag нет сообщений {TOPIC} — нечего кодировать")
    if writer is None:      # короткий bag: кадров меньше пробы
        span = probe[-1][0] - probe[0][0]
        fps = FPS_ENV if FPS_ENV > 0 else \
            max(1.0, min((len(probe) - 1) / span if span > 0 else 10.0, 60.0))
        writer = open_writer(fps, (probe[0][1].shape[1], probe[0][1].shape[0]))
        for _, fr in probe:
            writer.write(fr)
    writer.release()
    print(f"Записано {n_frames} кадров с HUD → {MP4}")
    print(f"  длительность ~{(t_last - t0):.1f}с (sim)")


if __name__ == "__main__":
    main()
