---
name: vins-fork-location
description: Форк VINS-MONO-ROS2 на этом ноуте лежит в /home/andriy/VINS-MONO-ROS2 (VINS_SRC в docker/sim/.env), а не в /root; git в контейнере делает файлы .git root-овыми
metadata:
  type: reference
---

Bind mount `vins_oss` берётся из `${VINS_SRC:-/root/VINS-MONO-ROS2}` (docker-compose.yml);
на этом боксе `.env` задаёт **`/home/andriy/VINS-MONO-ROS2`** — доступен с хоста без root.
Remote форка — https (без кредов в контейнере); пушить с хоста по ssh:
`git push git@github.com:linux100talion/VINS-MONO-ROS2 1317_debug`.
⚠️ Любой `git status/commit` ВНУТРИ контейнера (root) оставляет root-овые файлы в `.git`
(index, COMMIT_EDITMSG, FETCH_HEAD) → с хоста commit падает «Permission denied»; лечится
`docker run --rm -v /home/andriy/VINS-MONO-ROS2:/w --entrypoint chown sim-nav -R 1000:1000 /w/.git`.
Связано: [[vins-ground-stand-init]].
