#!/usr/bin/env python3
"""hud_status — статус гейта LOITER-на-VINS одной строкой для /mission/status.

Мотив (полёт lv1_joy_20260822_232043): LOITER не залатчился, потому что VINS
инициализировался только ПОСЛЕ посадки — пилот щёлкал CH6 вслепую, гейт молча
держал. Теперь лётная нода публикует статус, openhd_streamer рисует его баннером
в FPV, а bag отдаёт переходы в ленту событий joy_timeline.

ЕДИНСТВЕННЫЙ источник правды: ТЕ ЖЕ условия, которыми Freefly._mode_target и
LoiterHold пускают штатный LOITER (extnav_ready + свежий VINS + в воздухе
>loiter_alt, конфиг-ручка — см. config.loiter_alt), а не параллельная оценка
стримера. Градации:
  st=READY — гейт открыт: центр CH6 даст LOITER;
  st=WAIT  — VINS жив, но гейт закрыт (why: extnav — очередь EK3_SRC1_* ещё не
             пройдена; stale — odom протух с fresh, но ещё не 3×fresh;
             ground — на земле, LOITER невозможен по построению);
  st=DEAD  — VINS-одометрии нет (why: no_odom — не было вовсе; stale — молчит
             дольше 3×fresh — гистерезис выхода Freefly из LOITER).

Отдельно поля высот и канала зрения (palt=/ipm=/ipmf=) — диагностика демпфера у
земли: см. комментарий у их формирования ниже.

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
# Центр стика (1500): стики пилота в статусе — смещение от него, а не сырой PWM.
RC_CENTER = 1500


def hud_status(s, fresh_sec: float, loiter_alt: float = 1.5) -> str:
    """Строка "k=v k=v ..." для /mission/status по снапшоту DroneState.

    loiter_alt — гейт «в воздухе» (м): обязан совпадать с cfg.loiter_alt лётной
    ноды (bootstrap передаёт его сам); дефолт 1.5 — легаси для старых вызовов."""
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
    elif (s.rel_alt or 0.0) <= loiter_alt:
        st, why = 'WAIT', 'ground'
    else:
        st, why = 'READY', '-'
    # res/rat — диагностика детектора зрелости (ripeness.py): residual
    # «поза/скорость» (м/с; тихо < 0.15) и вертикальный ratio VINS/rel_alt
    # (зрел в [0.8,1.25]); -1 = ещё нет данных. Рисуются мелкой строкой HUD.
    # zekf — высота глазами EKF3 (z того же local_position, чьей свежестью
    # считается ekf=). Пара к alt= (баро при alt_src=baro): расхождение —
    # диагноз вертикали EKF прямо в FPV (прогон 174603: EKF z −0.27 м к
    # истине → гейт IPM alt<0.5 душил демпфер на истинных 0.7 м). Протух
    # local_position (после GPS-kill) — честное «--», не последнее значение.
    zekf = f"{s.ekf_z:.1f}" if (ekf and s.ekf_z is not None) else "--"
    # palt — ВЫСОТА ПЕРЦЕПЦИИ (perc_alt_src + латч perc_alt_zero): ровно то число,
    # по которому судит гейт земли IPM. Третье в строке не от жадности: разбор
    # 183305 упёрся ровно в то, что HUD показывал rel_alt и ekf_z, а канал закрывал
    # ТРЕТЬЮ высоту — у неё свой источник и своё смещение. ipm/ipmf — состояние
    # самого канала (ipm_ok + код причины брака кадра, таблица — FlowEstimator).
    palt = f"{s.perc_alt:.1f}" if s.perc_alt is not None else "--"
    # rcr/rcp/rct/rcy — СТИКИ ПИЛОТА глазами ноды (сырой PWM − центр, как их видит
    # арбитр и стек: pilot_* снапшота), sw — тумблер авто/ручной. Зачем в статусе:
    # /joy в bag штампован стеночным временем джойстик-ноды, и при плавающем RTF
    # его не выровнять с sim-временем (прогон lv2_joy_20260829_153405: сдвиг
    # «плывёт» −2…−5 с по полёту) — толчок пилота приходилось угадывать по целям
    # осей. Здесь стик лежит в той же строке и с тем же sim-штампом, что и всё
    # остальное. HUD эти поля не рисует (парсер k=v лишнее игнорирует).
    return (f"st={st} why={why} ekf={ekf} extnav={int(s.extnav_ready)} "
            f"odom={s.vins_odom_count} age={min(age, 999.0):.1f} "
            f"alt={(s.rel_alt or 0.0):.1f} zekf={zekf} palt={palt} "
            f"ipm={int(s.ipm_ok)} ipmf={s.ipm_fail} "
            f"res={s.vins_res:.2f} rat={s.vins_ratio:.2f} "
            f"rcr={s.pilot_roll - RC_CENTER} rcp={s.pilot_pitch - RC_CENTER} "
            f"rct={s.pilot_throttle - RC_CENTER} rcy={s.pilot_yaw - RC_CENTER} "
            f"sw={s.pilot_switch}")
