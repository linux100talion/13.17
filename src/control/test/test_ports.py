#!/usr/bin/env python3
"""Порты ↔ адаптеры: сверка сигнатур БЕЗ импорта ROS (AST).

Зачем. До 2026-09-01 ports.py не импортировал никто: Protocol'ы были декларацией
без проверки, и DebugSink молча разъехался с RosDebugSink (1 метод в порте против
6 в адаптере). Импортировать адаптеры в тесте нельзя — на хосте нет rclpy/mavros
(ros_telemetry/mavros_actuator/ros_io не импортируются вне контейнера), поэтому
адаптерная сторона сверяется ПО AST файла: имя класса → его def'ы → имена
параметров. Правило соответствия:
  - каждый метод порта есть в адаптере;
  - имена параметров порта совпадают с началом списка параметров адаптера
    (Protocol зовут и позиционно, и по имени — имена входят в контракт);
  - лишние параметры адаптера обязаны иметь дефолт (arm(value=True) — ок).

Запуск:  python3 src/control/test/test_ports.py
"""
import ast
import inspect
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain import ports                                 # noqa: E402

INFRA = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'control_pkg', 'infrastructure')

# порт → его адаптеры (файл, класс); MavrosActuator держит два порта — одна шина
BINDINGS = {
    'Clock':      [('ros_clock.py', 'RosClock')],
    'Telemetry':  [('ros_telemetry.py', 'RosTelemetry')],
    'RcOutput':   [('mavros_actuator.py', 'MavrosActuator')],
    'FlightMode': [('mavros_actuator.py', 'MavrosActuator')],
    'PilotInput': [('ros_pilot.py', 'JoyPilot'), ('ros_pilot.py', 'RosPilot'),
                   ('ros_pilot.py', 'ScriptedPilot')],
    'Logger':     [('ros_io.py', 'RosLogger')],
    'DebugSink':  [('ros_io.py', 'RosDebugSink')],
}

results = []


def check(name, ok, detail=""):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail and not ok else ""))


def port_methods(proto):
    """Методы Protocol'а: имя → список имён параметров (без self)."""
    out = {}
    for n, fn in vars(proto).items():
        if n.startswith('_') or not callable(fn):
            continue
        out[n] = [p for p in inspect.signature(fn).parameters if p != 'self']
    return out


def adapter_methods(path, klass):
    """def'ы класса из AST: имя → (имена параметров без self, число дефолтов,
    есть ли *args/**kw)."""
    tree = ast.parse(open(path, encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == klass:
            out = {}
            for f in node.body:
                if isinstance(f, ast.FunctionDef):
                    args = [a.arg for a in f.args.args if a.arg != 'self']
                    out[f.name] = (args, len(f.args.defaults),
                                   f.args.vararg is not None or f.args.kwarg is not None)
            return out
    return None


def conforms(pm, am):
    """Метод порта pm (имена параметров) против метода адаптера am — или причина."""
    names, ndef, var = am
    if names[:len(pm)] != pm:
        return f"параметры {names} ≠ порт {pm}"
    extra = len(names) - len(pm)
    if extra > ndef and not var:
        return f"лишние параметры без дефолта: {names[len(pm):]}"
    return None


for pname, targets in BINDINGS.items():
    proto = getattr(ports, pname)
    pms = port_methods(proto)
    for fname, klass in targets:
        am = adapter_methods(os.path.join(INFRA, fname), klass)
        if am is None:
            check(f"{pname} ↔ {klass}", False, f"класс не найден в {fname}")
            continue
        bad = []
        for m, params in pms.items():
            if m not in am:
                bad.append(f"нет метода {m}")
            else:
                why = conforms(params, am[m])
                if why:
                    bad.append(f"{m}: {why}")
        check(f"{pname} ↔ {klass} ({fname})", not bad, '; '.join(bad))

# runtime-подстраховка: все порты — runtime_checkable (issubclass работает там,
# где адаптеры импортируемы: контейнер sim / Orin)
check("все порты runtime_checkable",
      all(getattr(getattr(ports, p), '_is_runtime_protocol', False) for p in BINDINGS))

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ПОРТЫ OK" if ok_all else "❌ ПОРТЫ РАЗЪЕХАЛИСЬ С АДАПТЕРАМИ")
sys.exit(0 if ok_all else 1)
