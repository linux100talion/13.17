#!/usr/bin/env python3
"""sysmon.py — здоровье Orin в bag: /orin/sysmon (diagnostic_msgs/DiagnosticArray), 1 Гц.

Зачем. Видеокодер H.264 работает на CPU (у Orin Nano нет NVENC), стример сидит на своём
ядре 5 (etc/systemd/system.conf.d/cpuaffinity.conf) — после каждого прогона надо видеть,
сколько набежало: температуры, частоты (сброс частоты = перегрев/питание), загрузку ядер.
Запускается рядом с записью bag (auto_bag_m.sh), на ХОСТЕ — только /sys и /proc, ROS из
/opt/ros/humble. Пороги cpu-thermal Orin: 70 °C passive, 99 °C сброс частоты, 104 °C авария.

Статусы (name → values key: value):
  thermal  — <тип зоны>_C для каждой thermal_zone (cpu, gpu, soc0..2, tj, …), °C
  cpu      — cpu<i>_load_pct (за последний период), cpu<i>_mhz (текущая частота)
  board    — fan_pwm (0..255), mem_used_mb, mem_total_mb, power_mode (nvpmodel)
level: WARN, если cpu-thermal ≥ 70 °C; ERROR, если ≥ 95 °C.
"""
import glob
import os
import subprocess

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node


def _read(path, default=None):
    try:
        with open(path) as f:
            return f.read().strip()
    except Exception:        # зоны cv0..2 (блок выключен) отдают EAGAIN — в Python это то
        return default       # OSError, то TypeError изнутри codecs (замер 2026-10-02)


def _cpu_times():
    out = {}
    with open('/proc/stat') as f:
        for line in f:
            p = line.split()
            if p[0].startswith('cpu') and p[0] != 'cpu':
                v = [int(x) for x in p[1:]]
                out[p[0]] = (sum(v), v[3] + v[4])        # всего, idle+iowait
    return out


def _power_mode():
    try:
        r = subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True, timeout=2)
        for line in r.stdout.splitlines():
            if 'Power Mode' in line:
                return line.split(':', 1)[1].strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return '?'


class SysMon(Node):
    def __init__(self):
        super().__init__('orin_sysmon')
        self.pub = self.create_publisher(DiagnosticArray, '/orin/sysmon', 10)
        self.zones = sorted(glob.glob('/sys/class/thermal/thermal_zone*'))
        self.fan = (glob.glob('/sys/devices/platform/pwm-fan/hwmon/hwmon*/pwm1') or [None])[0]
        self.mode = _power_mode()
        self.prev = _cpu_times()
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f"sysmon: {len(self.zones)} термозон, режим питания {self.mode}")

    @staticmethod
    def _kv(k, v):
        return KeyValue(key=k, value=str(v))

    def tick(self):
        now = self.get_clock().now().to_msg()
        msg = DiagnosticArray()
        msg.header.stamp = now

        th = DiagnosticStatus(name='thermal', hardware_id='orin')
        cpu_t = None
        for z in self.zones:
            t = _read(z + '/temp')
            if t is None:
                continue
            c = int(t) / 1000.0
            kind = _read(z + '/type', os.path.basename(z))
            th.values.append(self._kv(kind.replace('-thermal', '') + '_C', f'{c:.1f}'))
            if kind == 'cpu-thermal':
                cpu_t = c
        th.level = (DiagnosticStatus.ERROR if cpu_t and cpu_t >= 95 else
                    DiagnosticStatus.WARN if cpu_t and cpu_t >= 70 else DiagnosticStatus.OK)
        th.message = f'cpu {cpu_t:.1f} C' if cpu_t is not None else 'cpu-thermal нет'

        cur = _cpu_times()
        cs = DiagnosticStatus(name='cpu', hardware_id='orin', level=DiagnosticStatus.OK)
        for name in sorted(cur, key=lambda s: int(s[3:])):
            tot, idle = cur[name]
            ptot, pidle = self.prev.get(name, (tot, idle))
            dt = tot - ptot
            load = 100.0 * (1 - (idle - pidle) / dt) if dt > 0 else 0.0
            cs.values.append(self._kv(f'{name}_load_pct', f'{load:.0f}'))
            khz = _read(f'/sys/devices/system/cpu/{name}/cpufreq/scaling_cur_freq')
            if khz:
                cs.values.append(self._kv(f'{name}_mhz', int(khz) // 1000))
        self.prev = cur

        bd = DiagnosticStatus(name='board', hardware_id='orin', level=DiagnosticStatus.OK)
        if self.fan:
            bd.values.append(self._kv('fan_pwm', _read(self.fan, '?')))
        mem = {}
        for line in (_read('/proc/meminfo', '') or '').splitlines():
            k, _, v = line.partition(':')
            mem[k] = int(v.split()[0]) // 1024 if v.strip() else 0
        if mem:
            bd.values.append(self._kv('mem_total_mb', mem.get('MemTotal', 0)))
            bd.values.append(self._kv('mem_used_mb', mem.get('MemTotal', 0) - mem.get('MemAvailable', 0)))
        bd.values.append(self._kv('power_mode', self.mode))

        msg.status = [th, cs, bd]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SysMon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
