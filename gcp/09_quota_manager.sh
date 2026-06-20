#!/bin/bash
# Просмотр и запрос квот, нужных для GPU-инстанса (1× Tesla T4).
#
# Для создания GPU-VM должны быть НЕНУЛЕВЫМИ сразу несколько квот:
#   • GPUS_ALL_REGIONS            — глобальный потолок на ВСЕ GPU (часто = 0 у
#                                   новых проектов и режет всё, даже при наличии
#                                   региональной квоты);
#   • NVIDIA_T4_GPUS             — региональная (on-demand T4);
#   • PREEMPTIBLE_NVIDIA_T4_GPUS — региональная (spot/preemptible T4).
# Скрипт показывает их лимиты/usage и умеет запросить повышение через
# Cloud Quotas API (`gcloud alpha quotas preferences create`). Запрос идёт на
# ревью Google; для 1×T4 на проекте с биллингом обычно аппрувится за секунды.
#
# Использование:
#   ./09_quota_manager.sh show           # только показать текущие квоты
#   ./09_quota_manager.sh ensure         # запросить всё нужное до значения 1
#   ./09_quota_manager.sh ensure 2       # … до значения 2
#   ZONE=europe-west1-b ./09_quota_manager.sh show   # другой регион

set -e

PROJECT="drone-13-17-workspace-2026"
ZONE="${ZONE:-europe-west4-a}"
REGION="${ZONE%-*}"          # europe-west4-a → europe-west4
SERVICE="compute.googleapis.com"

# Нужные квоты: "метрика|scope|quota-id|dimensions"
#   метрика  — имя в `gcloud compute … describe` (для чтения лимита/usage)
#   scope    — global | region
#   quota-id — id в Cloud Quotas API (для запроса повышения)
#   dimensions — пусто для global, region=$REGION для региональных
QUOTAS=(
  "GPUS_ALL_REGIONS|global|GPUS-ALL-REGIONS-per-project|"
  "NVIDIA_T4_GPUS|region|NVIDIA-T4-GPUS-per-project-region|region=$REGION"
  "PREEMPTIBLE_NVIDIA_T4_GPUS|region|PREEMPTIBLE-NVIDIA-T4-GPUS-per-project-region|region=$REGION"
)

# ── читаем лимиты в ассоциативный кэш (global + региональные) ────────────────
declare -A LIMIT USAGE
load_quotas() {
    local gj rj
    gj=$(gcloud compute project-info describe --project="$PROJECT" \
            --format="json(quotas)" 2>/dev/null)
    rj=$(gcloud compute regions describe "$REGION" --project="$PROJECT" \
            --format="json(quotas)" 2>/dev/null)
    local parsed
    parsed=$(python3 - "$gj" "$rj" <<'PY'
import sys, json
out = {}
for blob in sys.argv[1:3]:
    if not blob:
        continue
    for q in json.loads(blob).get("quotas", []):
        out[q["metric"]] = (q["limit"], q["usage"])
for m, (lim, use) in out.items():
    print(f"{m}\t{lim}\t{use}")
PY
)
    while IFS=$'\t' read -r m lim use; do
        [ -n "$m" ] && LIMIT[$m]=$lim && USAGE[$m]=$use
    done <<< "$parsed"
}

show() {
    echo "📊 Квоты GPU для проекта $PROJECT (регион $REGION):"
    printf "    %-28s %8s %8s\n" "QUOTA" "LIMIT" "USAGE"
    for row in "${QUOTAS[@]}"; do
        IFS='|' read -r metric scope qid dims <<< "$row"
        local lim="${LIMIT[$metric]:-?}" use="${USAGE[$metric]:-?}"
        printf "    %-28s %8s %8s\n" "$metric" "$lim" "$use"
    done
}

ensure() {
    local want="${1:-1}"
    echo "🎯 Гарантируем лимит ≥ $want для нужных GPU-квот..."
    local pending=0
    for row in "${QUOTAS[@]}"; do
        IFS='|' read -r metric scope qid dims <<< "$row"
        local lim="${LIMIT[$metric]:-0}"
        # числовое сравнение (лимиты приходят как 0.0 / 1.0)
        if awk "BEGIN{exit !(${lim:-0} >= $want)}"; then
            echo "  ✓ $metric уже $lim (≥ $want) — пропускаем."
            continue
        fi
        echo "  ↑ $metric = $lim < $want — запрашиваем $want..."
        local prefid="auto-$(echo "$qid" | tr 'A-Z' 'a-z' | tr -cs 'a-z0-9' '-')-${want}"
        prefid="${prefid:0:63}"
        local dimflag=()
        [ -n "$dims" ] && dimflag=(--dimensions="$dims")
        local out rc
        out=$(gcloud alpha quotas preferences create \
                --service="$SERVICE" --project="$PROJECT" \
                --quota-id="$qid" --preferred-value="$want" \
                --preference-id="$prefid" \
                "${dimflag[@]}" \
                --email="andriykutsevol@gmail.com" \
                --justification="Single NVIDIA T4 for torch/CUDA/ROS2 dev runs on dev-workspace-1317" 2>&1) && rc=0 || rc=$?
        if [ "${rc:-0}" -eq 0 ]; then
            echo "    → запрос отправлен (preference $prefid)."
            pending=1
        elif echo "$out" | grep -qi "already exists"; then
            echo "    → запрос уже существовал ($prefid) — оставляем как есть."
            pending=1
        else
            echo "    ✗ не удалось ($out)"
        fi
    done
    if [ "$pending" -eq 1 ]; then
        echo "⏳ Запросы на ревью. Часто аппрувятся за секунды — перепроверь:"
        echo "     ./09_quota_manager.sh show"
    fi
}

CMD="${1:-show}"
case "$CMD" in
    show)
        load_quotas; show ;;
    ensure)
        load_quotas
        echo "── ДО ──"; show; echo
        ensure "${2:-1}"; echo
        echo "── ПОСЛЕ (повторное чтение) ──"; load_quotas; show ;;
    *)
        echo "Использование: $0 {show|ensure [VALUE]}"; exit 1 ;;
esac
