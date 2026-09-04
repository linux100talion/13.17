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

⚠️ st/why — гейт ЯРУСА LOITER, а не живость VINS (разбор ab_noise 2026-08-30:
на 35-58 с bag VINS шёл 10 Гц, а баннер писал «VINS WAIT (extnav)» — ждала
очередь зрелости EK3_SRC1_*, потом «(ground)» — баро 0.3 м < loiter_alt 0.5).
Живость VINS в HUD — строка ODO (собственный замер стримера); баннер и блок
режимов говорят про ЛЕСЕНКУ SF-мастера: потолок SC, активный ярус и гейт
каждого яруса (поля lvl/tier/lat/t1/w1 + пороги vmin/lalt/ripe/rsec/rcnt —
см. LadderState и _ladder_fields). Ярус 1 (VinsHold) судится ЗЕРКАЛОМ
VinsHandover.vins_ready (odom ≥ vins_min, свежесть), ярус 2 — st/why выше,
ярус 0 (демпфер) — ipm/ipmf. Ту же лесенку ведёт Freefly._ladder_*: активный
ярус берётся оттуда (Freefly.ladder_state), не пересчитывается.

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

from dataclasses import dataclass


@dataclass(frozen=True)
class LadderState:
    """Состояние лесенки SF-мастера (Freefly._ladder_*) для строки статуса:
    level — потолок SC (0 демпфер / 1 +VinsHold / 2 +штатный LOITER), tier —
    активный ярус, latch_age — сколько сим-секунд LOITER уже выбран, а FCU режим
    не подтвердил («requires position», ре-ассерт идёт); −1 = не латчим. Отдаёт
    Freefly.ladder_state(); у планов без лесенки (легаси-селектор, миссии с
    loiter<t>) — None, и полей лесенки в строке нет (HUD рисует голый гейт)."""
    level: int
    tier: int
    latch_age: float = -1.0


# Имена ярусов лесенки — общие для лога ноды, HUD (ASCII: Hershey-шрифт OpenCV
# кириллицу не рисует) и ленты joy_timeline.
TIER_NAMES = {0: "DAMPER", 1: "VINS", 2: "LOITER"}

# Свежесть позиции EKF — зеркало fresh_sec у WaitEkfPos (step.py) и гейта
# GPS-kill в bootstrap_node: одна правда «EKF держит позицию».
EKF_FRESH_SEC = 2.0
# Центр стика (1500): стики пилота в статусе — смещение от него, а не сырой PWM.
RC_CENTER = 1500
# Состояния мягкой посадки (land= в статусе; SoftLand.land_state) — общие для
# лога ноды, HUD (ASCII) и ленты joy_timeline: pos — режим LAND FCU, позицию
# держит EKF (стек пуст); damper/vinshold — снижение в ALT_HOLD под нашим
# стеком (стик = наклон); touch — касание, ждём дизарм.
LAND_NAMES = {'pos': 'FCU POS', 'damper': 'DAMPER', 'vinshold': 'VINSHOLD',
              'touch': 'TOUCHDOWN'}


def loiter_gate(s, fresh_sec: float, loiter_alt: float):
    """Гейт ЯРУСА 2 (штатный LOITER-на-VINS) → (st, why). Ровно условие
    Freefly._mode_target / LoiterHold: extnav_ready + свежий VINS + в воздухе.
    Порядок фиксирован: no_odom → stale(DEAD, 3×fresh — гистерезис выхода) →
    extnav → stale(WAIT) → ground → READY."""
    age = s.now_sim - s.vins_last_sim
    if s.vins_odom_count == 0:
        return 'DEAD', 'no_odom'
    if age > 3.0 * fresh_sec:
        return 'DEAD', 'stale'
    if not s.extnav_ready:
        return 'WAIT', 'extnav'
    if age >= fresh_sec:
        return 'WAIT', 'stale'
    if (s.rel_alt or 0.0) <= loiter_alt:
        return 'WAIT', 'ground'
    return 'READY', '-'


def vinshold_gate(s, fresh_sec: float, vins_min: int):
    """Гейт ЯРУСА 1 (VinsHold нашего стека) → (st, why): зеркало
    VinsHandover.vins_ready — odom ≥ vins_min (BS_VINS_MIN, в LV-профилях 300) и
    свежесть < fresh. Градации как у LOITER (DEAD/stale по 3×fresh — гистерезис
    спуска Freefly._ladder_tier); своя причина «odom» — счётчик ниже порога."""
    age = s.now_sim - s.vins_last_sim
    if s.vins_odom_count == 0:
        return 'DEAD', 'no_odom'
    if age > 3.0 * fresh_sec:
        return 'DEAD', 'stale'
    if s.vins_odom_count < vins_min:
        return 'WAIT', 'odom'
    if age >= fresh_sec:
        return 'WAIT', 'stale'
    return 'READY', '-'


def hud_status(s, fresh_sec: float, loiter_alt: float = 1.5, ladder=None,
               vins_min: int = 0, ripe_sec: float = 0.0, ripe_min: int = 0,
               land=None) -> str:
    """Строка "k=v k=v ..." для /mission/status по снапшоту DroneState.

    loiter_alt — гейт «в воздухе» (м): обязан совпадать с cfg.loiter_alt лётной
    ноды (bootstrap передаёт его сам); дефолт 1.5 — легаси для старых вызовов.
    ladder — LadderState лесенки SF-мастера (None = полей лесенки нет);
    vins_min — порог яруса 1 (VinsHandover.min_count); ripe_sec/ripe_min —
    пороги зрелости очереди extnav (bootstrap: ripe_sec/ripe_min) — HUD рисует
    по ним ПРОГРЕСС ожидания «extnav», а не голое слово.
    land — состояние мягкой посадки (SoftLand.land_state: ключ LAND_NAMES);
    None = шаг посадки не активен, поля land= в строке нет. Кнопка посадки
    пилота (sa=) — всегда: её фронт в ленте joy_timeline объясняет переход."""
    ekf = int(s.now_sim - s.ekf_pos_last_sim < EKF_FRESH_SEC)
    age = s.now_sim - s.vins_last_sim
    st, why = loiter_gate(s, fresh_sec, loiter_alt)
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
    # t — sim-время снапшота. /mission/status — String без header: в bag он лежит по
    # СТЕНОЧНОМУ времени приёма, а RTF плывёт, и стики/раму из него было не выровнять
    # с истиной Gazebo (ab_frame: сдвиг −2…−5 с) — та же болезнь, что у /joy.
    return (f"t={s.now_sim:.2f} st={st} why={why} ekf={ekf} extnav={int(s.extnav_ready)} "
            f"odom={s.vins_odom_count} age={min(age, 999.0):.1f} "
            f"alt={(s.rel_alt or 0.0):.1f} zekf={zekf} palt={palt} "
            f"ipm={int(s.ipm_ok)} ipmf={s.ipm_fail} "
            f"res={s.vins_res:.2f} rat={s.vins_ratio:.2f} "
            f"rcr={s.pilot_roll - RC_CENTER} rcp={s.pilot_pitch - RC_CENTER} "
            f"rct={s.pilot_throttle - RC_CENTER} rcy={s.pilot_yaw - RC_CENTER} "
            f"sw={s.pilot_switch} sa={int(bool(getattr(s, 'pilot_land', False)))}"
            + (f" land={land}" if land else "")
            + _gate_fields(s, loiter_alt, ripe_sec, ripe_min)
            + _ladder_fields(s, ladder, fresh_sec, vins_min)
            + _frame_fields(s)
            + _wind_fields(s))


def _gate_fields(s, loiter_alt, ripe_sec, ripe_min) -> str:
    """Пороги гейта LOITER и прогресс зрелости extnav: lalt — loiter_alt (для
    «ground 0.3<0.5m» в HUD), ripe — сим-секунд от ПЕРВОЙ одометрии (−1 = не
    было), rsec/rcnt — ripe_sec/ripe_min очереди EK3_SRC1_* (время потока И
    счётчик, оба обязаны пройти). Всегда: гейт LOITER есть и без лесенки."""
    ripe = (min(s.now_sim - s.vins_first_sim, 999.0)
            if s.vins_first_sim > -1e8 else -1.0)
    return f" lalt={loiter_alt:g} ripe={ripe:.1f} rsec={ripe_sec:g} rcnt={ripe_min}"


def _ladder_fields(s, ladder, fresh_sec, vins_min) -> str:
    """Лесенка SF-мастера: lvl — потолок SC, tier — активный ярус (правда
    Freefly, не пересчёт), lat — возраст незалатченного LOITER (−1 = не латчим),
    t1/w1 — гейт яруса 1 (vinshold_gate), vmin — его порог. Только при живой
    лесенке — «чего нет в источниках, того нет и в строке»."""
    if ladder is None:
        return ""
    t1, w1 = vinshold_gate(s, fresh_sec, vins_min)
    return (f" lvl={ladder.level} tier={ladder.tier} lat={ladder.latch_age:.1f}"
            f" t1={t1} w1={w1} vmin={vins_min}")


def _wind_fields(s) -> str:
    """Оценка ветра тримом активного стабилизатора (стрелка ветра HUD): wnp/wnr
    — трим тангажа/крена в PWM КАНАЛОВ (валюта seed_trim: наклон трима смотрит
    ПРОТИВ ветра), wns — датчик источника ('ipm' демпфер / 'vins' DpVins; нода
    выбирает по АКТИВНОМУ стеку яруса; на ярусе 2 LOITER — выученный трим
    DpVins по текущему vins_yaw: стек пуст, но ветер при передаче FCU не
    исчез). Направление и силу из PWM считает рендерер (общая формула для FPV
    и scene_hud.mp4). spd= — |скорость| ТОГО ЖЕ датчика (нода: ipm_vfwd/vlat
    или vins_vx/vy), рисуется слева от компаса. Только при живом источнике —
    «чего нет в источниках, того нет и в строке»."""
    if not getattr(s, 'wind_src', ''):
        return ""
    out = f" wnp={s.wind_p:.0f} wnr={s.wind_r:.0f} wns={s.wind_src}"
    # |скорость| того же датчика (spd=, м/с) — рядом с компасом слева; -1 = нет
    vm = getattr(s, 'vel_mag', -1.0)
    if vm >= 0.0:
        out += f" spd={vm:.2f}"
    return out


def _frame_fields(s) -> str:
    """Рама станции в осях курса (StationFrame): позиция и гвоздь, м. Только когда
    рама активна — «чего нет в источниках, того нет и в строке»."""
    if not getattr(s, 'st_frame', 0):
        return ""
    pin = (f"{s.st_px:.2f}", f"{s.st_py:.2f}") if s.st_px == s.st_px else ("--", "--")
    return f" sf=1 sx={s.st_x:.2f} sy={s.st_y:.2f} spx={pin[0]} spy={pin[1]}"
