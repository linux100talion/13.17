#!/usr/bin/env python3
"""hud_status — статус гейта LOITER-на-VINS одной строкой для /mission/status.

Мотив (полёт lv1_joy_20260822_232043): LOITER не залатчился, потому что VINS
инициализировался только ПОСЛЕ посадки — пилот щёлкал CH6 вслепую, гейт молча
держал. Теперь лётная нода публикует статус, openhd_streamer рисует его баннером
в FPV, а bag отдаёт переходы в ленту событий joy_timeline.

ЕДИНСТВЕННЫЙ источник правды: ТЕ ЖЕ условия, которыми Freefly._mode_target и
LoiterHold пускают штатный LOITER (extnav_ready + свежий VINS + в воздухе
>1.5 м), а не параллельная оценка стримера. Градации:
  st=READY — гейт открыт: центр CH6 даст LOITER;
  st=WAIT  — VINS жив, но гейт закрыт (why: extnav — очередь EK3_SRC1_* ещё не
             пройдена; stale — odom протух с fresh, но ещё не 3×fresh;
             ground — на земле, LOITER невозможен по построению);
  st=DEAD  — VINS-одометрии нет (why: no_odom — не было вовсе; stale — молчит
             дольше 3×fresh — гистерезис выхода Freefly из LOITER).

Отдельно поле ekf= — прогрев EKF полётника: до взлёта VINS мёртв по построению
(монокуляру нужен параллакс) и баннер гейта всегда красный, а пилоту нужен
сигнал «борт готов, можно взлетать». ekf=1 ровно по тому же критерию, каким
WaitEkfPos (step.py) пускает арм: свежий /mavros/local_position — EKF держит
позицию. Рисуется отдельным баннером EKF READY/WARMUP (hud_renderer, до арма).

Живёт в application (не в ноде): чистая функция от снапшота, тестируется
оффлайн как handover (test_hud_status.py).
"""

# Свежесть позиции EKF — зеркало fresh_sec у WaitEkfPos (step.py) и гейта
# GPS-kill в bootstrap_node: одна правда «EKF держит позицию».
EKF_FRESH_SEC = 2.0


def hud_status(s, fresh_sec: float) -> str:
    """Строка "k=v k=v ..." для /mission/status по снапшоту DroneState."""
    ekf = int(s.now_sim - s.ekf_pos_last_sim < EKF_FRESH_SEC)
    age = s.now_sim - s.vins_last_sim
    if s.vins_odom_count == 0:
        st, why = 'DEAD', 'no_odom'
    elif age > 3.0 * fresh_sec:
        st, why = 'DEAD', 'stale'
    elif not s.extnav_ready:
        st, why = 'WAIT', 'extnav'
    elif age >= fresh_sec:
        st, why = 'WAIT', 'stale'
    elif (s.rel_alt or 0.0) <= 1.5:
        st, why = 'WAIT', 'ground'
    else:
        st, why = 'READY', '-'
    # res/rat — диагностика детектора зрелости (ripeness.py): residual
    # «поза/скорость» (м/с; тихо < 0.15) и вертикальный ratio VINS/rel_alt
    # (зрел в [0.8,1.25]); -1 = ещё нет данных. Рисуются мелкой строкой HUD.
    return (f"st={st} why={why} ekf={ekf} extnav={int(s.extnav_ready)} "
            f"odom={s.vins_odom_count} age={min(age, 999.0):.1f} "
            f"alt={(s.rel_alt or 0.0):.1f} "
            f"res={s.vins_res:.2f} rat={s.vins_ratio:.2f}")
