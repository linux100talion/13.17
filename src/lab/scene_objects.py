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

Как убирается: блок <include> НЕ удаляется из файла, а оборачивается в
XML-комментарий с маркером SCENE-CLEAR — поэтому restore возвращает объект
на исходное место с исходной позой, а diff остаётся читаемым.

    python3 src/lab/scene_objects.py status     # что сейчас на сцене
    python3 src/lab/scene_objects.py clear      # убрать объекты с земли
    python3 src/lab/scene_objects.py restore    # вернуть всё обратно

Мир правится на хосте (bind mount ./worlds:rw), Gazebo читает SDF при старте —
изменение применяется на СЛЕДУЮЩЕМ прогоне (make restart-all / capture_scene.sh).
Стек скрипт не трогает: дисциплина прогона (см. корневой CLAUDE.md) требует
поднимать стек целиком отдельной командой.
"""

import argparse
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


def do_clear(text, keep):
    removed = []
    pieces, last = [], 0
    for m, model, name in active_includes(text):
        if model in keep:
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


def do_status(text, path):
    act = active_includes(text)
    clr = cleared_includes(text)
    print(f"мир: {path}")
    print(f"на сцене  ({len(act)}):")
    for _m, model, name in act:
        print(f"    + {name:<16} model://{model}")
    print(f"убрано    ({len(clr)}):")
    for _m, model, name in clr:
        print(f"    - {name:<16} model://{model}")
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
    ap.add_argument("-n", "--dry-run", action="store_true", help="не писать файл")
    args = ap.parse_args()

    path = args.world
    if not path.is_file():
        sys.exit(f"ОШИБКА: мир не найден: {path}")
    text = path.read_text(encoding="utf-8")

    if args.action == "status":
        do_status(text, path)
        return

    keep = set(KEEP_DEFAULT) | set(args.keep)
    if args.action == "clear":
        new, names = do_clear(text, keep)
        verb = "убрано"
    else:
        new, names = do_restore(text)
        verb = "возвращено"

    if not names:
        print(f"нечего делать ({args.action}): мир уже в нужном состоянии — {path}")
        return

    validate(new, path)
    if args.dry_run:
        print(f"[dry-run] {verb} {len(names)}: {', '.join(names)}")
        return

    path.write_text(new, encoding="utf-8")
    print(f"{verb} {len(names)}: {', '.join(names)}")
    print(f"мир: {path}")
    print("применить: cd docker/sim && make restart-all && make wait")


if __name__ == "__main__":
    main()
