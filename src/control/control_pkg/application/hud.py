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

Живёт в application (не в ноде): чистая функция от снапшота, тестируется
оффлайн как handover (test_hud_status.py).
"""


def hud_status(s, fresh_sec: float) -> str:
    """Строка "k=v k=v ..." для /mission/status по снапшоту DroneState."""
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
    return (f"st={st} why={why} extnav={int(s.extnav_ready)} "
            f"odom={s.vins_odom_count} age={min(age, 999.0):.1f} "
            f"alt={(s.rel_alt or 0.0):.1f}")
