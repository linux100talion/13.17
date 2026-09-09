---
name: scratchpad-persistent-tmpdir
description: Scratchpad/temp Claude Code перенесён из /tmp в /usr/local/DATA/Calude/ctmp через CLAUDE_CODE_TMPDIR в ~/.claude/settings.json
metadata:
  type: user
---

С 2026-09-07 на ноуте стоит `~/.claude/settings.json` → `"env": {"CLAUDE_CODE_TMPDIR":
"/usr/local/DATA/Calude/ctmp"}`. Переносится ВЕСЬ per-uid временный корень:
`$CLAUDE_CODE_TMPDIR/claude-<uid>/<слаг-проекта>/<session-id>/scratchpad`, плюс
shell-снимки, сокеты, $TMPDIR дочерних процессов. Бэкап прежнего конфига —
`~/.claude/settings.json.bak-2026-09-07`.

**Why:** `/tmp` тут не отдельный маунт, но systemd-tmpfiles чистит его (`D /tmp 1777
root root 30d` — на буте и по возрасту), скретчпады прошлых сессий пропадали.

**How to apply:** проверено эмпирически на 2.1.263 — работает env-переменная процесса
и блок `env` в ПОЛЬЗОВАТЕЛЬСКОМ settings.json (проверял через `CLAUDE_CONFIG_DIR`
с копией конфига); проектный `.claude/settings.json` в новом (недоверенном) каталоге
env НЕ применил. Путь корня держать коротким: `<путь>/claude-<uid>/` ≤ 44 байт, иначе
$TMPDIR дочерних процессов откатывается в /tmp (лимит пути AF_UNIX 104). Каталог никто
не подчищает по возрасту — растёт сам. Фоновые джобы скретчпада не получают вовсе,
у них `$CLAUDE_JOB_DIR/tmp` под `~/.claude/jobs/` (живёт до удаления джоба).
