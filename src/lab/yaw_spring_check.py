#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yaw_spring_check — разбор «ПРУЖИНЫ» курса по joystick-серии spring/{long,short}/{left,right}/{N}.

Вопрос: после отпускания yaw-стика борт ВОЗВРАЩАЕТСЯ к прежнему курсу (пружина)
или остаётся на новом. И если возвращается — кто тянет: НАШ контур (PWM в
/flow_dbg6 ненулевой во время возврата) или что-то вне его (PWM≈0, борт едет сам).

Этим скриптом ДОКАЗАНА пружина 2026-08-27 (yaw_ab_ki60_win03/spring): err@rel
наматывался до 350–430°, возврат 92–96% разворота, пик обратного PWM = kd/leak·S·err
±3% — виноват D-член, кормившийся разностью ошибки с шагом утечки (см.
control.md «ПРУЖИНА курса»). После фикса (_dot + BS_YAW_PILOT_GAIN) на повторе
серии ждём: err@rel ≈ 0, return% ≈ 0, pwm после отпускания ≈ 0.

По каждому прогону:
  /joy                      axes[3] = yaw-стик (AETR CH4)
  /model/iris_cam/odometry  истинный курс Gazebo (unwrap), z
  /flow_dbg6                x=уставка курса (ед. сигнала), y=ошибка, z=PWM рыскания
  /flow_dbg                 z=конфиденс полнокадрового LK (исключить гейтинг)

Времена — по recv-штампу бэга (один рекордер, одна шкала).
Сегмент нажатия: |axes[3]| > 0.15 с группировкой пауз < 0.4 c.
Колонки на нажатие:
  Δpress      = yaw(отпуск) − yaw(нажатие), истинный (ENU: влево +)
  Δback       = yaw(конец окна до следующего нажатия) − yaw(отпуск)
  return%     = −Δback/Δpress·100 (100% = вернулся полностью; >100% = разматывает
                долг ПРОШЛЫХ нажатий)
  t50         = время до половины возврата, с
  sp_move°    = ход уставки за нажатие (ед./S) — насколько уставка убежала от борта
  err@rel°    = ошибка контура на отпускании — «намотка пружины»
  pwm(press / 3с после / пик окна) — у пружины знак после отпускания ПЕРЕВОРАЧИВАЕТСЯ

Запускать В КОНТЕЙНЕРЕ (или в одноразовом из образа sim-nav):
  docker run --rm \
    -v .../docker/sim/output/joystick/<серия>/spring:/spring:ro \
    -v .../src/lab:/lab:ro sim-nav:latest \
    bash -lc 'source /opt/ros/humble/setup.bash; python3 /lab/yaw_spring_check.py /spring'

Аргумент — корень серии (дефолт /spring) с раскладкой {long,short}/{left,right}/{1,2};
env BAG — вместо раскладки разобрать ОДИН bag-каталог. S (0.324), STICK_THR (0.15).
"""
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped

S = float(os.environ.get('S', '0.324'))          # px/кадр на °/с; deg = ед./S
STICK_THR = float(os.environ.get('STICK_THR', '0.15'))
GAP_S = 0.4


def read_bag(bag_dir):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    joy_t, joy_v = [], []
    od_t, od_yaw, od_z = [], [], []
    d6_t, d6_sp, d6_err, d6_pwm = [], [], [], []
    d1_t, d1_conf = [], []
    while r.has_next():
        topic, data, t = r.read_next()
        ts = t * 1e-9
        if topic == '/flow_dbg':
            m = deserialize_message(data, Vector3Stamped)
            d1_t.append(ts); d1_conf.append(m.vector.z)
        elif topic == '/joy':
            m = deserialize_message(data, Joy)
            if len(m.axes) > 3:
                joy_t.append(ts); joy_v.append(m.axes[3])
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(data, Odometry)
            q = m.pose.pose.orientation
            od_t.append(ts)
            od_yaw.append(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z)))
            od_z.append(m.pose.pose.position.z)
        elif topic == '/flow_dbg6':
            m = deserialize_message(data, Vector3Stamped)
            d6_t.append(ts); d6_sp.append(m.vector.x)
            d6_err.append(m.vector.y); d6_pwm.append(m.vector.z)
    if not joy_t or not od_t:
        raise RuntimeError(f'в {bag_dir} нет /joy или /model/iris_cam/odometry')
    t0 = min(joy_t[0], od_t[0])
    return {
        'joy': (np.array(joy_t) - t0, np.array(joy_v)),
        'od': (np.array(od_t) - t0, np.degrees(np.unwrap(np.array(od_yaw))),
               np.array(od_z)),
        'd6': (np.array(d6_t) - t0 if d6_t else np.array([]),
               np.array(d6_sp), np.array(d6_err), np.array(d6_pwm)),
        'd1': (np.array(d1_t) - t0 if d1_t else np.array([]), np.array(d1_conf)),
    }


def segments(jt, jv):
    """Нажатия yaw-стика: список (t0, t1, средний стик)."""
    on = np.abs(jv) > STICK_THR
    segs, cur0, last_on = [], None, None
    for i in range(len(jt)):
        if on[i]:
            if cur0 is None:
                cur0 = jt[i]
            last_on = jt[i]
        elif cur0 is not None and jt[i] - last_on > GAP_S:
            segs.append((cur0, last_on))
            cur0 = None
    if cur0 is not None:
        segs.append((cur0, last_on))
    return [(a, b, float(np.mean(jv[(jt >= a) & (jt <= b)]))) for a, b in segs]


def yaw_at(ot, oy, t):
    i = min(max(np.searchsorted(ot, t), 0), len(ot) - 1)
    return oy[i]


def analyze(name, bag_dir):
    d = read_bag(bag_dir)
    jt, jv = d['joy']
    ot, oy, oz = d['od']
    d6t, d6sp, d6err, d6pwm = d['d6']
    d1t, d1conf = d['d1']
    segs = segments(jt, jv)
    print(f"\n=== {name} ===  bag {ot[-1] - ot[0]:.0f} c, z_max {oz.max():.1f} м, "
          f"нажатий: {len(segs)}")
    for k, (a, b, stick) in enumerate(segs):
        nxt = segs[k + 1][0] if k + 1 < len(segs) else ot[-1]
        y0, y1 = yaw_at(ot, oy, a), yaw_at(ot, oy, b)
        win = (ot >= b) & (ot <= nxt)
        if not win.any():
            continue
        wy, wt = oy[win], ot[win]
        d_press, d_back = y1 - y0, wy[-1] - y1
        ret = -d_back / d_press * 100 if abs(d_press) > 1e-6 else float('nan')
        t50 = float('nan')
        if abs(d_back) > 1.0:
            half = y1 + d_back / 2
            crossed = np.nonzero((wy - half) * np.sign(d_back) >= 0)[0]
            if len(crossed):
                t50 = wt[crossed[0]] - b
        line = (f"  [{k}] t={a:6.1f}с dur={b - a:4.1f}с stick={stick:+.2f} | "
                f"Δpress={d_press:+7.1f}° Δback={d_back:+7.1f}° "
                f"return={ret:5.0f}% t50={t50:4.1f}с")
        if len(d6t):
            m_press = (d6t >= a) & (d6t <= b)
            m_post = (d6t >= b) & (d6t <= min(b + 3.0, nxt))
            m_back = (d6t >= b) & (d6t <= nxt)
            sp_move = ((d6sp[m_press][-1] - d6sp[m_press][0]) / S
                       if m_press.any() else float('nan'))
            err_rel = d6err[m_post][0] / S if m_post.any() else float('nan')
            pwm_press = np.mean(d6pwm[m_press]) if m_press.any() else float('nan')
            pwm_post = np.mean(d6pwm[m_post]) if m_post.any() else float('nan')
            pwm_back_max = (d6pwm[m_back][np.argmax(np.abs(d6pwm[m_back]))]
                            if m_back.any() else float('nan'))
            line += (f" | sp_move={sp_move:+7.1f}° err@rel={err_rel:+6.1f}° "
                     f"pwm(press)={pwm_press:+6.1f} pwm(3с после)={pwm_post:+6.1f} "
                     f"pwm(окно,пик)={pwm_back_max:+6.1f}")
        if len(d1t):
            m = (d1t >= a) & (d1t <= b)
            if m.any():
                c = d1conf[m]
                line += (f" | conf min={c.min():.2f} "
                         f"низкая({100 * np.mean(c < 0.20):.0f}%)")
        print(line)


def main():
    bag = os.environ.get('BAG')
    if bag:
        analyze(bag, bag)
        return
    root = sys.argv[1] if len(sys.argv) > 1 else '/spring'
    found = False
    for dur in ('long', 'short'):
        for side in ('left', 'right'):
            for n in sorted(os.listdir(os.path.join(root, dur, side))
                            if os.path.isdir(os.path.join(root, dur, side)) else []):
                p = os.path.join(root, dur, side, n, 'bag')
                if os.path.isdir(p):
                    found = True
                    analyze(f"{dur}/{side}/{n}", p)
    if not found:
        raise SystemExit(f'в {root} не нашлось {{long,short}}/{{left,right}}/N/bag')


if __name__ == '__main__':
    main()
