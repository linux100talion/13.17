#!/usr/bin/env python3
"""
scene_objects.py — убрать со сцены Gazebo все объекты, стоящие на земле,
и вернуть их обратно.

Зачем: для калибровочных/контрольных прогонов нужна ЧИСТАЯ площадка (нечего
задеть при runaway, поток от земли без деревьев/огня/стен), а для VINS/NN —
полная сцена. Скрипт переключает мир между этими двумя состояниями.

Что убирается: все <include> мира, КРОМЕ grass_plane (земля с текстурой) и
iris_cam (дрон). Т.е. mili_map (стены/дома/техника/солдаты), oak_tree_*,
pine_tree_*, fire_*. Свет, физика, system-плагины и <scene> не трогаются.

--radius R — убрать только БЛИЖНИЕ (ближе R метров от спавна дрона), дальние
оставить. Это рабочий режим для лётных экспериментов: борт в висении уезжает
на десятки метров (замер: 34 м за 10 с), и первым, во что он врезается, стоит
крепость в 35 м — прогон обрывается кувырком на середине окна. Радиус-очистка
даёт площадку, а дальний план для потока остаётся: перцепт смотрит на землю в
~19 м и на объекты за ней, и голая земля до горизонта его обедняет.
Расстояние берётся от <pose> самого include (для mili_map это его origin —
карта расставлена вокруг него, так что мерка грубая, но крепость она ловит).

Как убирается: блок <include> НЕ удаляется из файла, а оборачивается в
XML-комментарий с маркером SCENE-CLEAR — поэтому restore возвращает объект
на исходное место с исходной позой, а diff остаётся читаемым.

    python3 src/lab/scene_objects.py status              # что сейчас на сцене
    python3 src/lab/scene_objects.py clear               # убрать ВСЕ объекты с земли
    python3 src/lab/scene_objects.py clear --radius 50   # убрать только ближе 50 м
    python3 src/lab/scene_objects.py restore             # вернуть всё обратно

Мир правится на хосте (bind mount ./worlds:rw), Gazebo читает SDF при старте —
изменение применяется на СЛЕДУЮЩЕМ прогоне (make restart-all / capture_scene.sh).
Стек скрипт не трогает: дисциплина прогона (см. корневой CLAUDE.md) требует
поднимать стек целиком отдельной командой.
"""

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# Модели, которые остаются на сцене всегда (не «объекты на земле»).
KEEP_DEFAULT = ("grass_plane", "iris_cam")

WORLD_DEFAULT = (
    Path(__file__).resolve().parents[2] / "docker/sim/worlds/mili_fortress.sdf"
)

MARK = "SCENE-CLEAR"

# Верхнеуровневый <include> ... </include> вместе с отступом строки.
RE_INCLUDE = re.compile(r"[ \t]*<include>.*?</include>[ \t]*\n?", re.S)
# Любой XML-комментарий (нужен, чтобы не трогать уже закомментированные include,
# например отключённый mt_background).
RE_COMMENT = re.compile(r"<!--.*?-->", re.S)
# Наш маркерный блок: <!-- SCENE-CLEAR ... \n <include>...</include>\n -->
RE_CLEARED = re.compile(
    r"[ \t]*<!--[ \t]*" + MARK + r"[^\n]*\n(.*?)^[ \t]*-->[ \t]*\n?", re.S | re.M
)

RE_NAME = re.compile(r"<name>\s*([^<\s]+)\s*</name>")
RE_URI = re.compile(r"<uri>\s*model://([^<\s]+)\s*</uri>")
RE_POSE = re.compile(r"<pose>\s*([-\d.eE+]+)\s+([-\d.eE+]+)")

# Модель, от которой отсчитывается радиус (спавн дрона).
ORIGIN_MODEL = "iris_cam"


def xy(body):
    """(x, y) из <pose> блока; без позы — начало координат (значит «ближний»)."""
    m = RE_POSE.search(body)
    return (float(m.group(1)), float(m.group(2))) if m else (0.0, 0.0)


def _comment_spans(text):
    """Диапазоны символов, занятые XML-комментариями."""
    return [m.span() for m in RE_COMMENT.finditer(text)]


def _in_comment(span, spans):
    return any(a <= span[0] and span[1] <= b for a, b in spans)


def active_includes(text):
    """Список (match, model, name) для include ВНЕ комментариев."""
    spans = _comment_spans(text)
    out = []
    for m in RE_INCLUDE.finditer(text):
        if _in_comment(m.span(), spans):
            continue
        body = m.group()
        uri = RE_URI.search(body)
        model = uri.group(1) if uri else "?"
        nm = RE_NAME.search(body)
        out.append((m, model, nm.group(1) if nm else model))
    return out


def cleared_includes(text):
    """Список (match, model, name) для убранных (закомментированных) include."""
    out = []
    for m in RE_CLEARED.finditer(text):
        body = m.group(1)
        uri = RE_URI.search(body)
        model = uri.group(1) if uri else "?"
        nm = RE_NAME.search(body)
        out.append((m, model, nm.group(1) if nm else model))
    return out


def validate(text, path):
    """SDF должен остаться валидным XML — иначе Gazebo молча не поднимет мир."""
    try:
        ET.fromstring(text)
    except ET.ParseError as e:
        sys.exit(f"ОШИБКА: результат — невалидный XML ({path}): {e}")


def origin_xy(text, override=None):
    """Спавн дрона — точка отсчёта радиуса. Нет дрона в файле → (0, 0) или override.

    Вложенная модель (mili_map/model.sdf) про дрон не знает: она вставлена в мир
    со смещением, поэтому спавн ей передают снаружи, пересчитанным в её локальные
    координаты (мировой спавн МИНУС поза её include)."""
    if override is not None:
        return override
    for m, model, _name in active_includes(text):
        if model == ORIGIN_MODEL:
            return xy(m.group())
    return (0.0, 0.0)


def model_sdf(model, world):
    """Путь к model.sdf вложенной модели: ищем рядом с миром (worlds/*/<model>/)."""
    root = Path(world).parent
    for cand in (root / model / "model.sdf", *root.glob(f"*/{model}/model.sdf")):
        if cand.is_file():
            return cand
    return None


def nested_files(world):
    """Все model.sdf рядом с миром, в которых есть наш маркер (для restore/status)."""
    return sorted(p for p in Path(world).parent.glob("*/*/model.sdf")
                  if MARK in p.read_text(encoding="utf-8"))


def do_clear(text, keep, radius=None, origin=None):
    """radius=None → убрать все объекты; radius=R → только ближе R м от спавна."""
    ox, oy = origin_xy(text, origin)
    removed = []
    pieces, last = [], 0
    for m, model, name in active_includes(text):
        if model in keep:
            continue
        if radius is not None:
            x, y = xy(m.group())
            if math.hypot(x - ox, y - oy) >= radius:
                continue
        body = m.group()
        indent = re.match(r"[ \t]*", body).group()
        if not body.endswith("\n"):
            body += "\n"
        block = (
            # Заголовок ДОЛЖЕН быть одной строкой: restore возвращает всё, что
            # между ним и закрывающим -->.
            f"{indent}<!-- {MARK} {name} ({model}) — вернуть: scene_objects.py restore\n"
            f"{body}"
            f"{indent}-->\n"
        )
        pieces.append(text[last:m.start()])
        pieces.append(block)
        last = m.end()
        removed.append(name)
    pieces.append(text[last:])
    return "".join(pieces), removed


def do_restore(text):
    restored = []
    pieces, last = [], 0
    for m, _model, name in cleared_includes(text):
        pieces.append(text[last:m.start()])
        pieces.append(m.group(1))
        last = m.end()
        restored.append(name)
    pieces.append(text[last:])
    return "".join(pieces), restored


def do_status(text, path, origin=None):
    act = active_includes(text)
    clr = cleared_includes(text)
    ox, oy = origin_xy(text, origin)
    print(f"мир: {path}   спавн дрона: ({ox:g}, {oy:g})")
    print(f"на сцене  ({len(act)}), по удалению от спавна:")
    rows = sorted(((math.hypot(*(a - b for a, b in zip(xy(m.group()), (ox, oy)))), name, model)
                   for m, model, name in act))
    for dist, name, model in rows:
        print(f"    + {dist:6.1f} м  {name:<16} model://{model}")
    print(f"убрано    ({len(clr)}):")
    for m, model, name in clr:
        dist = math.hypot(*(a - b for a, b in zip(xy(m.group(1)), (ox, oy))))
        print(f"    - {dist:6.1f} м  {name:<16} model://{model}")
    if not clr:
        print("    (ничего)")


def main():
    ap = argparse.ArgumentParser(
        description="Убрать/вернуть объекты, стоящие на земле, в SDF-мире Gazebo.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Применяется на следующем прогоне: make restart-all / capture_scene.sh",
    )
    ap.add_argument("action", choices=("clear", "restore", "status"))
    ap.add_argument("--world", type=Path, default=WORLD_DEFAULT, help="путь к SDF-миру")
    ap.add_argument(
        "--keep",
        action="append",
        default=[],
        metavar="MODEL",
        help=f"имя модели, которую не убирать (плюс к дефолту: {', '.join(KEEP_DEFAULT)})",
    )
    ap.add_argument(
        "--radius",
        type=float,
        default=None,
        metavar="R",
        help="убрать только объекты БЛИЖЕ R метров от спавна дрона (дальние оставить); "
             "без флага clear убирает все",
    )
    ap.add_argument("-n", "--dry-run", action="store_true", help="не писать файл")
    args = ap.parse_args()

    path = args.world
    if not path.is_file():
        sys.exit(f"ОШИБКА: мир не найден: {path}")
    text = path.read_text(encoding="utf-8")

    if args.action == "status":
        do_status(text, path)
        for f in nested_files(path):
            sub = f.read_text(encoding="utf-8")
            print(f"внутри {f.parent.name} убрано ({len(cleared_includes(sub))}):")
            for _m, model, name in cleared_includes(sub):
                print(f"    - {name:<16} model://{model}")
        return

    keep = set(KEEP_DEFAULT) | set(args.keep)
    # (файл, новый текст, имена) — мир и все составные модели, куда пришлось зайти
    edits = []
    if args.action == "clear":
        new, names = do_clear(text, keep, args.radius)
        verb = "убрано" + (f" (ближе {args.radius:g} м)" if args.radius else "")
        edits.append((path, new, names))
        # Составные модели (mili_map — 66 include внутри) радиусом по своей позе не
        # чистятся: их origin далеко, а содержимое разложено вокруг спавна. Заходим
        # внутрь КАЖДОЙ оставшейся на сцене составной модели и режем тем же радиусом,
        # пересчитав спавн в её локальные координаты.
        wx, wy = origin_xy(text)
        for m, model, _name in active_includes(new):
            if model in keep:
                continue
            f = model_sdf(model, path)
            if f is None:
                continue
            sub = f.read_text(encoding="utf-8")
            if "<include>" not in sub:
                continue
            px, py = xy(m.group())
            sub_new, sub_names = do_clear(sub, keep, args.radius, (wx - px, wy - py))
            if sub_names:
                edits.append((f, sub_new, [f"{model}/{n}" for n in sub_names]))
    else:
        new, names = do_restore(text)
        verb = "возвращено"
        edits.append((path, new, names))
        for f in nested_files(path):
            sub_new, sub_names = do_restore(f.read_text(encoding="utf-8"))
            if sub_names:
                edits.append((f, sub_new, [f"{f.parent.name}/{n}" for n in sub_names]))

    edits = [(f, t, n) for f, t, n in edits if n]
    if not edits:
        print(f"нечего делать ({args.action}): мир уже в нужном состоянии — {path}")
        return

    for f, t, _n in edits:
        validate(t, f)
    total = sum(len(n) for _f, _t, n in edits)
    for f, t, n in edits:
        if not args.dry_run:
            f.write_text(t, encoding="utf-8")
        print(f"{'[dry-run] ' if args.dry_run else ''}{verb} {len(n)} в {f.name}: {', '.join(n)}")
    print(f"итого {total}; файлов правлено: {len(edits)}")
    if args.dry_run:
        return
    print("применить: cd docker/sim && make restart-all && make wait")


if __name__ == "__main__":
    main()
