#!/usr/bin/env bash
# ============================================================================
# bootstrap_gce.sh — поднять Claude Code + репу 13.17 на свежем GCE-GPU инстансе.
#
# Запускать НА GCE-боксе (Ubuntu 22.04), не в Termux. «Скопировать Claude» = это:
# ставим программу Claude Code заново (npm) + клонируем репу (контекст/план едет в
# ней) + опц. восстанавливаем папку памяти. Сам диалог не переносится — новая сессия
# стартует с чистого листа, но репа (c3_gce_setup.txt/c3_TODO.txt/howto) + память
# дают преемственность.
#
# Использование:
#   chmod +x bootstrap_gce.sh
#   ./bootstrap_gce.sh [REPO_DIR] [MEMORY_TARBALL]
#     REPO_DIR        — куда клонировать (по умолчанию ~/13.17)
#     MEMORY_TARBALL  — опц. путь к tar с папкой памяти (scp'нул с телефона); если
#                       задан, распакуем под правильный project-hash GCE (см. ниже).
# ============================================================================
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/linux100talion/13.17.git}"
REPO_DIR="${1:-$HOME/13.17}"
MEMORY_TARBALL="${2:-}"

echo "== 1/5 системные пакеты (git, curl, build-essential) =="
sudo apt-get update -y
sudo apt-get install -y git curl ca-certificates build-essential

echo "== 2/5 Node.js LTS (через NodeSource) =="
if ! command -v node >/dev/null 2>&1; then
  curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
  sudo apt-get install -y nodejs
fi
echo "   node $(node --version), npm $(npm --version)"

echo "== 3/5 Claude Code (npm, глобально) =="
if ! command -v claude >/dev/null 2>&1; then
  sudo npm install -g @anthropic-ai/claude-code
fi
echo "   claude: $(command -v claude)"

echo "== 4/5 репозиторий 13.17 =="
if [ ! -d "$REPO_DIR/.git" ]; then
  # приватная репа -> нужен токен/gh; при отказе склонируй вручную и перезапусти
  git clone "$REPO_URL" "$REPO_DIR"
fi
echo "   репа: $REPO_DIR ($(git -C "$REPO_DIR" rev-parse --abbrev-ref HEAD))"

echo "== 5/5 память Claude (опц.) =="
# Папка памяти привязана к ХЭШУ рабочего каталога проекта. На телефоне это было
# -root-13-17 (путь /root/13.17); на GCE путь иной -> хэш иной. Поэтому кладём
# содержимое в каталог памяти, соответствующий REPO_DIR на ЭТОЙ машине.
PROJ_HASH="$(echo "$REPO_DIR" | sed 's#/#-#g')"           # грубая нормализация пути в hash-имя
MEM_DIR="$HOME/.claude/projects/${PROJ_HASH}/memory"
if [ -n "$MEMORY_TARBALL" ] && [ -f "$MEMORY_TARBALL" ]; then
  mkdir -p "$MEM_DIR"
  tar -xzf "$MEMORY_TARBALL" -C "$MEM_DIR" --strip-components=1 || \
    tar -xzf "$MEMORY_TARBALL" -C "$MEM_DIR"
  echo "   память распакована -> $MEM_DIR"
else
  echo "   (память не передана — пропуск; план всё равно в репе: "
  echo "    $REPO_DIR/src/nav/tools/nn2_route/c3_gce_setup.txt)"
fi

cat <<EOF

================================================================================
ГОТОВО. Дальше ВРУЧНУЮ:
  1. Авторизуй Claude:    cd "$REPO_DIR" && claude    (логин по подсказке)
     или экспортни ключ:  export ANTHROPIC_API_KEY=...   перед запуском.
  2. Проверь GPU:         nvidia-smi
  3. Зависимости проекта (torch/CUDA, faiss, ROS2, cv_bridge) — по чек-листу:
     $REPO_DIR/src/nav/tools/nn2_route/c3_gce_setup.txt  (разделы 1–2)
  4. Прогон пайплайна (c)-основного — B1–B6 из того же файла.
Память (если не передал tarball'ом): на телефоне выполни
  tar -czf mem.tgz -C ~/.claude/projects/-root-13-17 memory
  scp mem.tgz <gce>:~   и перезапусти скрипт со 2-м аргументом ~/mem.tgz
================================================================================
EOF
