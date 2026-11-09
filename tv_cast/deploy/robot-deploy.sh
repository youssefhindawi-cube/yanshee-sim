#!/usr/bin/env bash
# Deploy the caster + prebuilt slideshow to the Yanshee and install the play-once service.
# The robot then casts to the TV; the laptop is not part of the stream.
# Prereq: key-based SSH to the robot (run `ssh-copy-id pi@<robot-ip>` once).
set -euo pipefail
ROBOT=${1:-pi@192.168.89.80}
TV=${2:-192.168.89.9}
MP4=${3:-$HOME/robot-slideshow.mp4}
HERE=$(cd "$(dirname "$0")/.." && pwd)
SSH="ssh -o StrictHostKeyChecking=no -o ConnectTimeout=12"
SCP="scp -o StrictHostKeyChecking=no -q"

echo "== copy caster + slideshow to $ROBOT =="
$SCP "$HERE/robot/rcast.py" "$ROBOT:/home/pi/rcast.py"
[ -f "$MP4" ] && $SCP "$MP4" "$ROBOT:/home/pi/robot-slideshow.mp4" || echo "  (no $MP4 yet; build it with build/build-slideshow.sh)"

echo "== install the play-once service (TV=$TV) =="
sed "s#--tv 192.168.89.9#--tv $TV#" "$HERE/robot/rcast-slideshow.service" | \
  $SSH "$ROBOT" 'sudo tee /etc/systemd/system/rcast-slideshow.service >/dev/null; sudo systemctl daemon-reload'

echo "== done. service is installed, stopped, manual-start. To cast once: =="
echo "   ssh $ROBOT 'sudo systemctl start rcast-slideshow'"
