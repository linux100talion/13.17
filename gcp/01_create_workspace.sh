#!/bin/bash
# Скрипт для первичного создания воркспейса

PROJECT="drone-13-17-workspace-2026"
ZONE="us-central1-a"
INSTANCE_NAME="dev-workspace-137"

echo "🚀 Создаем инстанс $INSTANCE_NAME в зоне $ZONE..."

gcloud compute instances create $INSTANCE_NAME \
    --project=$PROJECT \
    --zone=$ZONE \
    --machine-type=n1-standard-8 \
    --maintenance-policy=TERMINATE \
    --accelerator=type=nvidia-tesla-t4,count=1 \
    --image-family=ubuntu-2404-lts-amd64 \
    --image-project=ubuntu-os-cloud \
    --boot-disk-size=120GB \
    --boot-disk-type=pd-balanced

echo "✅ Готово! Подключайся командой: gcloud compute ssh $INSTANCE_NAME --zone=$ZONE"
