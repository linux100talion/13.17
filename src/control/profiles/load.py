#!/usr/bin/env python3
"""load.py — ЕДИНСТВЕННЫЙ способ собрать ручки прогона из профилей src/control/profiles.

Профиль — файл `<ярус>/<имя>.txt`: строки `KEY=VALUE`, комментарии `#`, пустые строки и
директива `include <файл>` (путь относительно каталога самого профиля). Семантика:

  • `include baseline.txt` ставит ВСЕ ключи включённого файла, а строки ниже в этом же
    файле их ПЕРЕКРЫВАЮТ — кандидат = эталон + дельта. Include рекурсивный, цикл = ошибка.
  • Между РАЗНЫМИ перечисленными профилями один и тот же ключ — ОШИБКА (dphold/ и dpvins/
    не имеют права спорить).
  • Пустое значение (`BS_KF_ALT_HOLD=`) — легально только у Optional-полей ноды (= None).
  • Любая другая строка (без `=`, ключ не из [A-Z][A-Z0-9_]*, неизвестная директива) —
    ошибка с именем файла и номером строки. Профиль обязан быть однозначным.
  • СТРОГАЯ СХЕМА (по умолчанию): схема = поля BootstrapConfig (mission_pkg/config.py,
    ключ BS_<ПОЛЕ>) + EXTRA_KEYS (SITL/скрипты/ветер). Ключ вне схемы → ошибка;
    поле ноды без ключа → ошибка; значение не того типа → ошибка. `--no-strict` —
    только сборка (частичные наборы, отладка одного каталога).

Профили НЕ предназначены для прямого `source` в bash: строка `include` там упадёт
(«command not found») — это намеренно, тихого чтения половины файла быть не должно.

Использование:
  python3 src/control/profiles/load.py dphold/baseline dpvins/brake5_stop … world/wind2_gust5
      → `export KEY='VALUE'` построчно (eval в bootstrap_arch2.sh / freefly_lv.sh)
  … --format plain   → `KEY=VALUE` (мета прогона <RUN>.env)
  … --format json    → {"KEY": "VALUE"}
  … --origin         → в plain/shell добавляется `# файл:строка` — откуда пришло значение
  … --diff <RUN>.env → сверка с метой прогона: расходящиеся ключи (код 1), ключи профилей,
                       которых в мете нет, BS_-ключи меты вне профилей. Заменяет check.sh.

Имя профиля — `каталог/имя` или `каталог/имя.txt` относительно src/control/profiles,
либо путь к файлу. Порядок перечисления = порядок вывода. BASELINE_STACK — эталонный
стек (тесты `BootstrapConfig.baseline()`, `check.sh` без аргументов).
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import shlex
import sys
from typing import Dict, List, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
KEY_RE = re.compile(r'^([A-Z][A-Z0-9_]*)=(.*)$')
INCLUDE_RE = re.compile(r'^include\s+(\S+)\s*$')

# Эталонный стек = то, чем летает cmd/bl (WT=1). Менять вместе с cmd/bl/bl.sh.
BASELINE_STACK = ['dphold/baseline', 'dpvins/brake5_stop', 'vinshold/baseline',
                  'vins/scale25', 'loiter/guard', 'wind/trim',
                  'mission/baseline', 'legacy/baseline', 'world/wind2_gust5']


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


def load(names: List[str], strict: bool = True) -> Resolved:
    """Несколько профилей: дубль ключа между ними — ошибка; strict — сверка со схемой."""
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
    if strict:
        check_schema(merged, names)
    return merged


def schema_module():
    """mission_pkg.config рядом по дереву репы (хост: src/mission; контейнер:
    /root/sim_ws/src/mission) — без установки пакета."""
    for base in (os.path.join(HERE, '..', '..', 'mission'), '/root/sim_ws/src/mission'):
        p = os.path.join(base, 'mission_pkg', 'config.py')
        if os.path.isfile(p):
            spec = importlib.util.spec_from_file_location('mission_pkg_config', p)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    raise ProfileError('не нашёл mission_pkg/config.py (схема ключей) — ожидаю src/mission рядом')


def check_schema(res: Resolved, names: List[str]) -> None:
    """Ключи ⊆ поля ноды ∪ EXTRA_KEYS; поля ноды ⊆ ключи; значения парсятся по типам."""
    cfg = schema_module()
    C = cfg.BootstrapConfig
    required = C.required_keys()            # BS_KEY -> поле
    allowed = set(required) | set(cfg.EXTRA_KEYS)
    unknown = sorted(k for k in res if k not in allowed)
    missing = sorted(k for k in required if k not in res)
    problems = []
    if unknown:
        problems.append('ключи вне схемы (не поле BootstrapConfig и не EXTRA_KEYS): '
                        + ', '.join(f'{k} ({rel(res[k][1][0])}:{res[k][1][1]})' for k in unknown))
    if missing:
        problems.append(f'нет {len(missing)} полей ноды: ' + ' '.join(missing))
    if not problems:
        try:
            C.from_mapping({k: v for k, (v, _o) in res.items()}, 'profiles')
        except SystemExit as e:
            problems.append(str(e))
    if problems:
        raise ProfileError('стек ' + ' '.join(names) + ' не проходит схему: '
                           + ' | '.join(problems))


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
    print(f'расхождений: {n_diff}; ключей профилей, которых в мете нет: {n_skip}; '
          f'BS_-ключей меты вне профилей: {len(extra)}'
          + (f' → {" ".join(extra)}' if extra else ''))
    return 1 if n_diff else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='сборка ручек прогона из профилей')
    ap.add_argument('profiles', nargs='+', metavar='ПРОФИЛЬ')
    ap.add_argument('--format', choices=['shell', 'plain', 'json'], default='shell')
    ap.add_argument('--origin', action='store_true', help='помечать, из какого файла ключ')
    ap.add_argument('--diff', metavar='RUN.env', help='сверить с метой прогона')
    ap.add_argument('--no-strict', action='store_true',
                    help='не сверять со схемой BootstrapConfig (частичный набор)')
    a = ap.parse_args(argv)
    res = load(a.profiles, strict=not a.no_strict)
    if a.diff:
        return diff(res, a.diff)
    print(emit(res, a.format, a.origin))
    return 0


if __name__ == '__main__':
    sys.exit(main())
