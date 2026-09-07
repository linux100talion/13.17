#!/usr/bin/env python3
"""Офлайн-тест загрузчика профилей (без ROS): python3 src/control/profiles/test_load.py

Проверяет контракт load.py на временных файлах + инварианты РЕАЛЬНЫХ профилей репо:
активный стек cmd/bl собирается без дублей, каждый кандидат = baseline + дельта,
у mission/ и legacy/ нет пересечений с ярусами.
"""
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import load  # noqa: E402


def w(d, name, text):
    p = os.path.join(d, name)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, 'w') as fh:
        fh.write(text)
    return p


class TmpProfiles(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='prof_')
        w(self.d, 'a/baseline.txt', '# эталон\nBS_X=1\nBS_Y=two\nBS_EMPTY=\n')
        w(self.d, 'a/cand.txt', '# кандидат\ninclude baseline.txt\nBS_Y=three\nBS_Z=0.5\n')
        w(self.d, 'b/baseline.txt', 'BS_B=7\n')

    def test_include_delta(self):
        r = load.resolve_file(os.path.join(self.d, 'a/cand.txt'))
        self.assertEqual({k: v for k, (v, _) in r.items()},
                         {'BS_X': '1', 'BS_Y': 'three', 'BS_EMPTY': '', 'BS_Z': '0.5'})
        self.assertTrue(r['BS_Y'][1][0].endswith('cand.txt'))      # откуда — файл дельты
        self.assertTrue(r['BS_X'][1][0].endswith('baseline.txt'))  # унаследованный — из эталона

    def test_duplicate_across_profiles_fails(self):
        w(self.d, 'b/dup.txt', 'BS_X=9\n')
        with self.assertRaises(SystemExit) as cm:
            load.load([os.path.join(self.d, 'a/cand.txt'), os.path.join(self.d, 'b/dup.txt')], strict=False)
        self.assertEqual(cm.exception.code, 2)

    def test_no_duplicate_ok(self):
        r = load.load([os.path.join(self.d, 'a/cand.txt'), os.path.join(self.d, 'b/baseline.txt')], strict=False)
        self.assertEqual(set(r), {'BS_X', 'BS_Y', 'BS_EMPTY', 'BS_Z', 'BS_B'})

    def test_bad_line_fails(self):
        p = w(self.d, 'a/bad.txt', 'BS_OK=1\nlowercase=2\n')
        with self.assertRaises(SystemExit):
            load.resolve_file(p)
        p = w(self.d, 'a/bad2.txt', 'BS_OK=1\nBS_NOEQ\n')
        with self.assertRaises(SystemExit):
            load.resolve_file(p)

    def test_missing_include_fails(self):
        p = w(self.d, 'a/noinc.txt', 'include nope.txt\n')
        with self.assertRaises(SystemExit):
            load.resolve_file(p)

    def test_include_cycle_fails(self):
        w(self.d, 'c/x.txt', 'include y.txt\nBS_A=1\n')
        p = w(self.d, 'c/y.txt', 'include x.txt\nBS_B=1\n')
        with self.assertRaises(SystemExit):
            load.resolve_file(p)

    def test_shell_quoting(self):
        p = w(self.d, 'a/q.txt', "BS_S=spd=5 at=30 rise=2\nBS_Q=it's\n")
        out = load.emit(load.resolve_file(p), 'shell', False)
        env = subprocess.run(['bash', '-c', out + '\nprintf "%s|%s" "$BS_S" "$BS_Q"'],
                             capture_output=True, text=True).stdout
        self.assertEqual(env, "spd=5 at=30 rise=2|it's")

    def test_diff(self):
        envf = w(self.d, 'run.env', 'BS_X=1\nBS_Y=other\nBS_ALIEN=5\n')
        r = load.load([os.path.join(self.d, 'a/cand.txt')], strict=False)
        self.assertEqual(load.diff(r, envf), 1)     # BS_Y расходится


class RepoProfiles(unittest.TestCase):
    STACK = load.BASELINE_STACK

    def test_active_stack_loads(self):
        r = load.load(self.STACK)                       # строгая схема: все поля, ничего лишнего
        self.assertGreater(len(r), 200)
        for k in ('BS_STAB', 'BS_VINS_STAB', 'BS_PILOT', 'BS_MISSION', 'BS_FENCE',
                  'BS_WIND_TRIM', 'BS_CONTROL_MODE'):
            self.assertIn(k, r)

    def test_selector_lives_in_vins(self):
        # dpvins/ и vinshold/ — только гейны, грузятся вместе без спора; селектор — vins/
        r = load.load(['dpvins/baseline', 'vinshold/baseline', 'vins/baseline'], strict=False)
        self.assertEqual(r['BS_VINS_STAB'][0], 'dpvins')
        self.assertTrue(r['BS_VINS_STAB'][1][0].endswith('vins/baseline.txt'))
        self.assertTrue(r['BS_VINS_I_LATCH'][1][0].endswith('vins/baseline.txt'))
        r = load.load(self.STACK[:3] + ['vins/vinshold'] + self.STACK[4:])   # строго: полный стек
        self.assertEqual(r['BS_VINS_STAB'][0], 'vinshold')      # откат — одним профилем

    def test_no_selector_in_gain_dirs(self):
        for sub in ('dpvins', 'vinshold'):
            for f in os.listdir(os.path.join(HERE, sub)):
                if f.endswith('.txt'):
                    r = load.resolve_file(os.path.join(HERE, sub, f))
                    self.assertNotIn('BS_VINS_STAB', r, f'{sub}/{f}')
                    self.assertNotIn('BS_VINS_I_LATCH', r, f'{sub}/{f}')

    def test_every_candidate_is_baseline_plus_delta(self):
        for sub in os.listdir(HERE):
            d = os.path.join(HERE, sub)
            if not os.path.isdir(d) or not os.path.isfile(os.path.join(d, 'baseline.txt')):
                continue
            base = load.resolve_file(os.path.join(d, 'baseline.txt'))
            for f in os.listdir(d):
                if f == 'baseline.txt' or not f.endswith('.txt'):
                    continue
                text = open(os.path.join(d, f)).read()
                # цепочка допустима: кандидат может наследовать не эталон напрямую, а
                # другого кандидата (loiter/rth = guard + дельта возврата). Проверяем
                # СМЫСЛ — ключи эталона на месте, — а не букву 'include baseline.txt'
                self.assertRegex(text, r'(?m)^include \S+\.txt$',
                                 f'{sub}/{f}: кандидат без include')
                cand = load.resolve_file(os.path.join(d, f))
                self.assertTrue(set(base) <= set(cand), f'{sub}/{f}: потерял ключи эталона')

    def test_strict_schema(self):
        # без mission/ не хватает полей — строгий загрузчик падает; лишний ключ — тоже
        with self.assertRaises(SystemExit):
            load.load([p for p in self.STACK if not p.startswith('mission/')])
        d = tempfile.mkdtemp(prefix='prof_')
        w(d, 'x/extra.txt', 'BS_NO_SUCH_FIELD=1\n')
        with self.assertRaises(SystemExit):
            load.load(self.STACK + [os.path.join(d, 'x/extra.txt')])
        cfgmod = load.schema_module()
        cfg = cfgmod.BootstrapConfig.from_profiles(self.STACK)
        self.assertEqual(cfg.pilot, 'joy')
        self.assertIsNone(cfg.kf_alt_hold)

    def test_replay_profile(self):
        r = load.load(['mission/replay'], strict=False)
        self.assertEqual(r['BS_PILOT'][0], 'replay')
        self.assertEqual(r['BS_MISSION'][0], 'freefly')


if __name__ == '__main__':
    unittest.main(verbosity=1)
