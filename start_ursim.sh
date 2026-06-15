#!/usr/bin/env bash
#
# Convenience script to start the official Universal Robots URSim e-series Docker image
# with the ports needed for:
#   - Polyscope UI (VNC / noVNC)   → http://localhost:6080  or VNC client on :5900
#   - RTDE from host Python        → port 30004  (used by force_mode_example.py, external controllers, etc.)
#   - Dashboard (optional)         → port 29999
#
# Your current command was:
#   docker run --rm -it universalrobots/ursim_e-series
#
# That command does NOT publish any ports, so:
#   - You cannot reach the Polyscope UI from the host at localhost:6080/5900
#   - Your Python RTDE scripts on the host cannot connect (RTDE on "localhost" will fail)
#
# Usage:
#   chmod +x start_ursim.sh
#   ./start_ursim.sh
#
# Then in another terminal:
#   python force_mode_example.py
#
# And open the UI in your browser:
#   http://localhost:6080
#
# Tips:
# - First boot can take 30-90 seconds before Polyscope is fully up.
# - Inside Polyscope: Power on the robot → put it in REMOTE mode before running RTDE forceMode or external controllers.
# - Ctrl-C in this terminal will stop the container (because of --rm).
#
# For more persistence (recommended for serious work), add volume mounts so programs survive container restarts.
# Example (uncomment the -v lines below and create the dirs on your host):
#   mkdir -p "$HOME/.ursim/programs" "$HOME/.ursim/urcaps"
#
# Advanced: If you want the container to have the same network as the host (simpler for some people):
#   --network host   (then you can remove most -p lines, but it has other trade-offs on Linux)

set -e

echo "Starting URSim e-series (UR5e) with host port forwarding..."
echo "VNC/noVNC UI will be at: http://localhost:6080"
echo "RTDE will be reachable from host Python on: localhost (port 30004)"
echo ""

docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  universalrobots/ursim_e-series

# Optional volume mounts (uncomment if you want to persist programs/urcaps):
#  -v "$HOME/.ursim/programs:/root/programs" \
#  -v "$HOME/.ursim/urcaps:/urcaps" \
#  universalrobots/ursim_e-series

echo ""
echo "URSim container exited."
