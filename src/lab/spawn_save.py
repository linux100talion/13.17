#!/usr/bin/env python3
# ============================================================================
# spawn_save.py — СОХРАНИТЬ точку старта под именем.
#
# Из каталога прогона берётся ровно одно: ПОСЛЕДНЯЯ поза борта
# (`/model/iris_cam/odometry` в bag'е — семь чисел: xyz + кватернион). Она
# кладётся в docker/sim/output/spawn/<имя> — крошечный текстовый файл, после
# чего сам прогон (видео, десятки ГБ bag'а) можно удалять.
#
#   python3 src/lab/spawn_save.py docker/sim/output/joystick/lv1_joy_20260824_140447
#     → спросит имя (например among_trees)
#   python3 src/lab/spawn_save.py <прогон> among_trees      # имя сразу
#   python3 src/lab/spawn_save.py --list                    # что сохранено
#   python3 src/lab/spawn_save.py --show among_trees        # показать пресет
#
# Дальше прогон стартует оттуда просто по имени:
#   SPAWN_POSE=among_trees bash src/lab/freefly_lv.sh
#   SPAWN_POSE=among_trees make -C docker/sim fresh-start
#
# Имя разрешает scripts/sim_up.sh: не 6 чисел → ищет /root/output/spawn/<имя>
# (это и есть docker/sim/output/spawn/, каталог смонтирован в контейнер).
# ⚠️ output/ не под git — пресеты живут рядом с прогонами, как bag'и.
# ============================================================================
import datetime
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spawn_pose
from spawn_pose import find_db3

# .../src/lab/spawn_save.py → корень репы на три уровня выше
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SPAWN_DIR = os.path.join(ROOT, 'docker', 'sim', 'output', 'spawn')
NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')


def listing():
    if not os.path.isdir(SPAWN_DIR) or not os.listdir(SPAWN_DIR):
        print(f"пресетов нет ({SPAWN_DIR})")
        return
    print(f"пресеты спавна ({SPAWN_DIR}):")
    for n in sorted(os.listdir(SPAWN_DIR)):
        path = os.path.join(SPAWN_DIR, n)
        pose, src = '?', '?'
        for line in open(path):
            if line.startswith('# из прогона:'):
                src = line.split(':', 1)[1].strip()
            elif not line.lstrip().startswith('#') and line.strip():
                pose = line.strip()
                break
        print(f"  {n:24s} {pose:46s} ← {os.path.basename(src)}")


def show(name):
    path = os.path.join(SPAWN_DIR, name)
    if not os.path.isfile(path):
        raise SystemExit(f"ОШИБКА: нет пресета '{name}' ({path})")
    sys.stdout.write(open(path).read())


def save(run, name, keep_rp=False, dz=0.0):
    db3 = find_db3(run)
    rows = spawn_pose.tail(spawn_pose.sqlite3.connect(db3), spawn_pose.GT, 200)
    if not rows:
        raise SystemExit(
            f"ОШИБКА: в bag'е нет {spawn_pose.GT} — позу взять неоткуда.\n"
            f"  (прогон должен писать этот топик: он есть в дефолтном "
            f"TOPICS_EXTRA у freefly_lv.sh)")
    samples = [spawn_pose.read_odom(d) for _, d in rows]
    t_end, p, q = samples[-1]
    yaw = spawn_pose.yaw_of(q)
    roll, pitch = spawn_pose.rp_of(q) if keep_rp else (0.0, 0.0)
    move = max(math.dist(s[1], p) for s in samples if t_end - s[0] <= 2.0) \
        if any(t_end - s[0] <= 2.0 for s in samples) else 0.0

    os.makedirs(SPAWN_DIR, exist_ok=True)
    path = os.path.join(SPAWN_DIR, name)
    with open(path, 'w') as f:
        f.write(f"# точка старта «{name}»\n")
        f.write(f"# из прогона: {os.path.abspath(run)}\n")
        f.write(f"# снято: {datetime.date.today().isoformat()}"
                f" (последняя поза {spawn_pose.GT})\n")
        f.write(f"# место: x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} м,"
                f" курс {math.degrees(yaw):.2f}° (ENU: 0 = нос на восток)\n")
        f.write(f"# покой на хвосте записи: сдвиг {move:.3f} м\n")
        f.write("# формат: x y z roll pitch yaw (метры/радианы, оси мира Gazebo)\n")
        f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2] + dz:.4f} "
                f"{roll:.4f} {pitch:.4f} {yaw:.5f}\n")

    print(f"сохранено: {path}")
    print(f"  место:  x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} м, "
          f"курс {math.degrees(yaw):.2f}°")
    if move > 0.20:
        print("  ⚠️ борт в конце записи ещё двигался — это последний кадр "
              "прогона, а не «место посадки»")
    print(f"  старт оттуда:  SPAWN_POSE={name} bash src/lab/freefly_lv.sh")


def usage():
    print(__doc__ or '', end='')
    print("использование:\n"
          "  spawn_save.py <каталог прогона|bag|.db3> [имя] [--keep-rp] [--dz N]\n"
          "  spawn_save.py --list | --show <имя>")


def cli():
    args = [a for a in sys.argv[1:]]
    if not args or args[0] in ('-h', '--help'):
        usage()
        return 0
    if args[0] == '--list':
        listing()
        return 0
    if args[0] == '--show':
        if len(args) < 2:
            raise SystemExit("ОШИБКА: --show <имя>")
        show(args[1])
        return 0

    keep_rp = '--keep-rp' in args
    dz = 0.0
    if '--dz' in args:
        dz = float(args[args.index('--dz') + 1])
    pos = [a for i, a in enumerate(args)
           if not a.startswith('--') and not (i and args[i - 1] == '--dz')]
    run = pos[0]
    name = pos[1] if len(pos) > 1 else ''
    if not name:
        try:
            name = input("под каким именем сохранить точку старта? ").strip()
        except EOFError:
            name = ''
    if not NAME_RE.match(name or ''):
        raise SystemExit("ОШИБКА: имя — буквы/цифры/._- , без пробелов и слэшей "
                         "(например among_trees)")
    path = os.path.join(SPAWN_DIR, name)
    if os.path.isfile(path):
        try:
            if input(f"пресет '{name}' уже есть — перезаписать? [y/N] ")\
                    .strip().lower() not in ('y', 'yes', 'д', 'да'):
                print("отменено")
                return 1
        except EOFError:
            raise SystemExit(f"ОШИБКА: пресет '{name}' уже есть")
    save(run, name, keep_rp, dz)
    return 0


if __name__ == '__main__':
    sys.exit(cli())
