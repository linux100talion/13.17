#!/usr/bin/env python3
"""load.py — ЕДИНСТВЕННЫЙ способ собрать ручки прогона из профилей src/control/profiles.

Профиль — файл `<ярус>/<имя>.txt`: строки `KEY=VALUE`, комментарии `#`, пустые строки и
директива `include <файл>` (путь относительно каталога самого профиля). Семантика:

  • `include baseline.txt` ставит ВСЕ ключи включённого файла, а строки ниже в этом же
    файле их ПЕРЕКРЫВАЮТ — кандидат = эталон + дельта. Include рекурсивный, цикл = ошибка.
  • Между РАЗНЫМИ перечисленными профилями один и тот же ключ — ОШИБКА (dphold/ и dpvins/
    не имеют права спорить). Взаимоисключающие каталоги (dpvins/ vs vinshold/) в один
    вызов не подавать.
  • Пустое значение (`BS_KF_ALT_HOLD=`) — легально: bootstrap_arch2.sh пропускает пустые
    (`[ -n … ]`), нода берёт None. До этапа «нода без дефолтов» это способ сказать «выкл».
  • Любая другая строка (без `=`, ключ не из [A-Z][A-Z0-9_]*, неизвестная директива) —
    ошибка с именем файла и номером строки. Профиль обязан быть однозначным.

Профили НЕ предназначены для прямого `source` в bash: строка `include` там упадёт
(«command not found») — это намеренно, тихого чтения половины файла быть не должно.

Использование:
  python3 src/control/profiles/load.py dphold/baseline dpvins/brake5_stop vins/scale25 …
      → `export KEY='VALUE'` построчно (для `eval` в cmd/<имя>/<имя>.sh)
  … --format plain   → `KEY=VALUE` (сравнение, мета)
  … --format json    → {"KEY": "VALUE"}
  … --origin         → в plain/shell добавляется `# файл:строка` — откуда пришло значение
  … --diff <RUN>.env → сверка с метой прогона: ключи, чьи значения различаются; ключи,
                       которых в мете нет (не доехали до env — раньше это был дефолт ноды),
                       считаются отдельно. Код выхода 1 при расхождениях. Заменяет check.sh.

Имя профиля — `каталог/имя` или `каталог/имя.txt` относительно src/control/profiles,
либо путь к файлу. Порядок перечисления = порядок вывода.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import sys
from typing import Dict, List, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
KEY_RE = re.compile(r'^([A-Z][A-Z0-9_]*)=(.*)$')
INCLUDE_RE = re.compile(r'^include\s+(\S+)\s*$')


class ProfileError(SystemExit):
    """Ошибка профиля: сообщение в stderr, код выхода 2 (1 оставлен за --diff)."""

    def __init__(self, msg: str):
        print(f'profiles: {msg}', file=sys.stderr)
        super().__init__(2)


Origin = Tuple[str, int]                       # (файл, строка)
Resolved = Dict[str, Tuple[str, Origin]]       # KEY -> (VALUE, откуда)


def resolve_name(name: str) -> str:
    """'dpvins/brake5_stop' | 'dpvins/brake5_stop.txt' | путь к файлу → абсолютный путь."""
    cands = [name, name + '.txt',
             os.path.join(HERE, name), os.path.join(HERE, name + '.txt')]
    for c in cands:
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise ProfileError(f'профиль не найден: {name!r} (искал {cands[2]} / .txt и как путь)')


def resolve_file(path: str, stack: Tuple[str, ...] = ()) -> Resolved:
    """Один профиль с включениями: включённое — первым, свои строки перекрывают."""
    if path in stack:
        raise ProfileError('цикл include: ' + ' → '.join(stack + (path,)))
    out: Resolved = {}
    with open(path, encoding='utf-8') as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.rstrip('\n')
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            m = INCLUDE_RE.match(s)
            if m:
                inc = os.path.join(os.path.dirname(path), m.group(1))
                if not os.path.isfile(inc):
                    raise ProfileError(f'{rel(path)}:{lineno}: include {m.group(1)!r} — файла нет')
                out.update(resolve_file(inc, stack + (path,)))
                continue
            m = KEY_RE.match(line)
            if not m:
                raise ProfileError(f'{rel(path)}:{lineno}: не разобрал строку {line!r} '
                                   f'(ожидаю KEY=VALUE, `# коммент` или `include файл`)')
            key, val = m.group(1), m.group(2).rstrip()
            out[key] = (val, (path, lineno))
    return out


def rel(path: str) -> str:
    try:
        return os.path.relpath(path, HERE)
    except ValueError:
        return path


def load(names: List[str]) -> Resolved:
    """Несколько профилей: дубль ключа между ними — ошибка."""
    merged: Resolved = {}
    owner: Dict[str, str] = {}
    for name in names:
        path = resolve_name(name)
        part = resolve_file(path)
        for key, (val, org) in part.items():
            if key in merged:
                raise ProfileError(
                    f'дубль ключа {key}: {owner[key]} и {rel(path)} '
                    f'({rel(org[0])}:{org[1]}) — профили спорят, реши в одном месте')
            merged[key] = (val, org)
            owner[key] = rel(path)
    return merged


def read_env_file(path: str) -> Dict[str, str]:
    """<RUN>.env (мета freefly_lv) или любой KEY=VALUE файл: последнее вхождение побеждает."""
    out: Dict[str, str] = {}
    with open(path, encoding='utf-8') as fh:
        for raw in fh:
            m = KEY_RE.match(raw.rstrip('\n'))
            if m:
                out[m.group(1)] = m.group(2).rstrip()
    return out


def emit(res: Resolved, fmt: str, origin: bool) -> str:
    lines = []
    if fmt == 'json':
        return json.dumps({k: v for k, (v, _) in res.items()}, ensure_ascii=False, indent=1)
    for k, (v, (f, n)) in res.items():
        tail = f'   # {rel(f)}:{n}' if origin else ''
        if fmt == 'shell':
            lines.append(f'export {k}={shlex.quote(v)}{tail}')
        else:
            lines.append(f'{k}={v}{tail}')
    return '\n'.join(lines)


def diff(res: Resolved, env_path: str) -> int:
    env = read_env_file(env_path)
    n_diff = n_skip = 0
    for k, (v, (f, n)) in res.items():
        if k not in env:
            n_skip += 1
            continue
        if env[k] != v:
            print(f'≠ {k}: профиль={v}  env={env[k]}   ({rel(f)}:{n})')
            n_diff += 1
    extra = sorted(k for k in env if k.startswith('BS_') and k not in res)
    print(f'расхождений: {n_diff}; ключей только в профилях (в мете нет — до этапа '
          f'«нода без дефолтов» это дефолт ноды): {n_skip}; '
          f'BS_-ключей меты вне профилей: {len(extra)}'
          + (f' → {" ".join(extra)}' if extra else ''))
    return 1 if n_diff else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='сборка ручек прогона из профилей')
    ap.add_argument('profiles', nargs='+', metavar='ПРОФИЛЬ')
    ap.add_argument('--format', choices=['shell', 'plain', 'json'], default='shell')
    ap.add_argument('--origin', action='store_true', help='помечать, из какого файла ключ')
    ap.add_argument('--diff', metavar='RUN.env', help='сверить с метой прогона')
    a = ap.parse_args(argv)
    res = load(a.profiles)
    if a.diff:
        return diff(res, a.diff)
    print(emit(res, a.format, a.origin))
    return 0


if __name__ == '__main__':
    sys.exit(main())
