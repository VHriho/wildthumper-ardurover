#!/usr/bin/env bash
# video_relay.sh - пересилає відео камер з WSL у Windows (Mission Planner / QGC).
# GstCameraPlugin шле UDP на 127.0.0.1:5600-5603 всередині WSL, а цей скрипт
# переправляє кожен потік на IP Windows-хоста (він змінюється після перезапуску WSL).
#
#   ./video_relay.sh            - усі 4 камери
#   ./video_relay.sh 5600       - лише передня
set -e
WIN_IP=$(ip route show default | awk '{print $3; exit}')
PORTS=${*:-5600 5601 5602 5603}
command -v socat >/dev/null || { echo "Встановіть socat: sudo apt install socat"; exit 1; }
echo "Windows IP: $WIN_IP"
PIDS=()
for P in $PORTS; do
    socat -u UDP4-RECV:$P,reuseaddr UDP4-SENDTO:$WIN_IP:$P &
    PIDS+=($!)
    echo "  UDP $P -> $WIN_IP:$P"
done
trap 'kill ${PIDS[*]} 2>/dev/null' EXIT INT TERM
echo "Пересилання працює. Ctrl+C - зупинити."
wait
