#!/usr/bin/env bash
# Record the demo video: N episodes per IK method, then join them into one file.
#
# Usage (from the Lab3/ folder, venv active):
#   bash scripts/record_videos.sh                 # HEAL, 5 episodes/method, 1280x720
#   bash scripts/record_videos.sh franka          # Franka
#   bash scripts/record_videos.sh heal 6 1920x1080
#
# Output:
#   videos/<robot>/<method>_nominal_<time>/video.mp4   one clip per method
#   videos/<robot>_all_methods.mp4                    joined video (Mink -> DLS -> QP)
# Each HEAL episode is ~11-12 s of simulated time, so 5 episodes ~= 1 min per method.
set -euo pipefail

ROBOT="${1:-heal}"
EPISODES="${2:-5}"
SIZE="${3:-1280x720}"
SEED="${SEED:-0}"

cd "$(dirname "$0")/.."
command -v ffmpeg >/dev/null || { echo "ffmpeg missing: sudo apt install -y ffmpeg"; exit 1; }

LIST="$(mktemp)"
for m in mink dls qp; do
  echo "=== recording $ROBOT / $m : $EPISODES episodes ==="
  python scripts/"$ROBOT"/ik_"$m"/run_"$m"_pick_place.py \
      -n "$EPISODES" --seed "$SEED" --record --video-size "$SIZE" --no-snapshots --out videos
  clip="$(ls -td "$PWD"/videos/"$ROBOT"/"$m"_nominal_*/ | head -1)video.mp4"
  dur="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$clip")"
  echo "    clip: $clip  (${dur%.*} s)"
  echo "file '$clip'" >> "$LIST"
done

OUT="videos/${ROBOT}_all_methods.mp4"
ffmpeg -y -v error -f concat -safe 0 -i "$LIST" -c copy "$OUT"
rm -f "$LIST"
echo
echo "joined video: $OUT  ($(ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT" | cut -d. -f1) s)"
