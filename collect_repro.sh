#!/usr/bin/env bash
# collect_repro.sh — збирає в один архів усе, що потрібно для перевірки
# та відтворення симуляції WildThumper XL (ArduRover SITL + Gazebo).
#
# Запуск:  bash ~/my_gz_models/collect_repro.sh
# Номери логів можна передати аргументами:  bash collect_repro.sh 30 31 32 33
set -u

AP=~/ardupilot                 # вихідники ArduPilot, звідки запускається sim_vehicle.py
MODELS=~/my_gz_models          # світ, модель, параметри, скрипти
LOGS="${*:-30 31 32 33}"       # місія + тести failsafe
OUT=~/wildthumper_repro_$(date +%Y%m%d)

rm -rf "$OUT"
mkdir -p "$OUT"/{firmware,logs,params,mission,lua,gazebo,tools}

echo "== 1. Прошивка (SITL) і точна версія коду"
cp "$AP/build/sitl/bin/ardurover" "$OUT/firmware/" 2>/dev/null || echo "  ! немає build/sitl/bin/ardurover"
{
  echo "commit:   $(git -C "$AP" rev-parse HEAD)"
  echo "describe: $(git -C "$AP" describe --always --dirty --tags)"
  echo "branch:   $(git -C "$AP" rev-parse --abbrev-ref HEAD)"
  echo "remote:   $(git -C "$AP" remote get-url origin)"
} > "$OUT/firmware/ardupilot_version.txt"
git -C "$AP" status --short    > "$OUT/firmware/git_status.txt"
git -C "$AP" diff              > "$OUT/firmware/local_changes.diff"   # порожній = код без змін
git -C "$AP" submodule status  > "$OUT/firmware/submodules.txt"

echo "== 2. Логи місій"
for n in $LOGS; do
  f=$(printf "%s/logs/%08d.BIN" "$AP" "$n")
  if [ -f "$f" ]; then cp "$f" "$OUT/logs/"; else echo "  ! немає $f"; fi
done
cp "$AP"/mav.tlog "$AP"/mav.parm "$OUT/logs/" 2>/dev/null   # телеметрія MAVProxy останнього запуску

echo "== 3. Параметри, місія, геозона, Lua"
cp "$MODELS"/*.param "$OUT/params/" 2>/dev/null
cp "$MODELS"/*.waypoints "$MODELS"/*.poly "$MODELS"/*.plan "$OUT/mission/" 2>/dev/null
cp "$AP"/scripts/*.lua "$OUT/lua/" 2>/dev/null
cp "$MODELS"/test_*.lua "$OUT/lua/" 2>/dev/null

echo "== 4. Gazebo: світ, модель, рельєф, сенсори"
rsync -a --exclude '*.BIN' --exclude '__pycache__' --exclude '*.tar.gz' "$MODELS"/ "$OUT/gazebo/"
AGZ=$(find ~ -maxdepth 4 -type d -name ardupilot_gazebo 2>/dev/null | head -1)
[ -n "$AGZ" ] && echo "ardupilot_gazebo: $AGZ  commit $(git -C "$AGZ" rev-parse HEAD 2>/dev/null)" > "$OUT/gazebo/ardupilot_gazebo_version.txt"

echo "== 5. Інструменти й команди запуску"
cp "$MODELS"/lidar_to_mavlink.py "$MODELS"/video_relay.sh "$OUT/tools/" 2>/dev/null
grep -hE "sim_vehicle|gz sim|lidar_to_mavlink|video_relay|GZ_SIM_" ~/.bash_history ~/.bashrc 2>/dev/null \
  | tail -40 > "$OUT/tools/launch_commands_history.txt"
{
  echo "Ubuntu:   $(lsb_release -ds 2>/dev/null)"
  echo "Kernel:   $(uname -r)"
  echo "Gazebo:   $(gz sim --versions 2>/dev/null | head -1)"
  echo "Python:   $(python3 --version 2>&1)"
  echo "pymavlink: $(python3 -c 'import pymavlink;print(pymavlink.__version__)' 2>/dev/null)"
  echo "MAVProxy: $(mavproxy.py --version 2>/dev/null | head -1)"
} > "$OUT/tools/versions.txt"
cp /mnt/c/Users/*/.wslconfig "$OUT/tools/" 2>/dev/null

echo "== 6. Контрольні суми й архів"
( cd "$OUT" && find . -type f ! -name SHA256SUMS -exec sha256sum {} + | sort -k2 > SHA256SUMS )
tar -czf "$OUT.tar.gz" -C "$(dirname "$OUT")" "$(basename "$OUT")"
du -sh "$OUT.tar.gz"
echo "Готово: $OUT.tar.gz"
