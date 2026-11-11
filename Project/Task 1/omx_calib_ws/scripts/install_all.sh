#!/usr/bin/env bash
# =============================================================================
# One laptop, one workspace: ROBOTIS arm stack + RealSense + peg vision + calibration.
# Ubuntu 24.04 + ROS 2 Jazzy.  Usage (from the workspace root, e.g. ~/omx_calib_ws):
#     bash scripts/install_all.sh
# Safe to re-run: every step skips what is already there.
# =============================================================================
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO=jazzy
RS_ROS_COMMIT="7a87097c"          # realsense-ros 4.59.1
step() { echo; echo "==================== $* ===================="; }

step "0. System check"
. /etc/os-release
[[ "${VERSION_ID:-}" == "24.04" ]] || { echo "ERROR: Ubuntu 24.04 needed (found ${PRETTY_NAME:-?})"; exit 1; }
[[ -f /opt/ros/$DISTRO/setup.bash ]] || { echo "ERROR: install ROS 2 Jazzy first:
  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html  (ros-jazzy-desktop + ros-dev-tools)"; exit 1; }
if dpkg -l | grep -q "^ii  ros-$DISTRO-librealsense2"; then
  echo "ERROR: ros-$DISTRO-librealsense2 conflicts with the RealSense apt SDK:"
  echo "  sudo apt remove 'ros-$DISTRO-librealsense2*' 'ros-$DISTRO-realsense2-*'"; exit 1
fi
set +u; source /opt/ros/$DISTRO/setup.bash; set -u

step "1. Base packages"
sudo apt-get update
sudo apt-get install -y git curl gnupg lsb-release python3-pip python3-numpy python3-yaml \
  python3-rosdep python3-vcstool python3-colcon-common-extensions \
  ros-$DISTRO-ros2-control ros-$DISTRO-ros2-controllers ros-$DISTRO-control-msgs \
  ros-$DISTRO-xacro ros-$DISTRO-robot-state-publisher ros-$DISTRO-rqt-image-view \
  ros-$DISTRO-tf2-ros ros-$DISTRO-message-filters ros-$DISTRO-std-srvs
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update

step "2. RealSense SDK (apt, includes udev rules)"
if [[ ! -f /etc/apt/sources.list.d/librealsense.list ]]; then
  sudo mkdir -p /etc/apt/keyrings
  curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
    gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
  echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
  sudo apt-get update
fi
sudo apt-get install -y librealsense2 librealsense2-gl librealsense2-utils librealsense2-dev librealsense2-udev-rules

step "3. U2D2 / serial port (lessons from the first hardware tests)"
# latency 1 ms (default 16 ms -> 'no status packet' and an arm that ignores commands) + permissions
echo 'KERNEL=="ttyUSB*", DRIVERS=="ftdi_sio", MODE="0666", ATTR{device/latency_timer}="1"' | \
  sudo tee /etc/udev/rules.d/99-omx-u2d2.rules > /dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger
if dpkg -s brltty >/dev/null 2>&1; then
  read -r -p "brltty is installed and can grab the U2D2. Remove it? [Y/n] " a
  [[ "${a:-Y}" =~ ^[Nn] ]] || sudo apt-get remove -y brltty
fi
id -nG "$USER" | grep -qw dialout || { sudo usermod -aG dialout "$USER"; echo "Added to dialout: LOG OUT and back in."; }

step "4. Sources"
mkdir -p "$WS/src/third_party"
if [[ ! -d "$WS/src/third_party/realsense-ros" ]]; then
  git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master "$WS/src/third_party/realsense-ros"
fi
git -C "$WS/src/third_party/realsense-ros" fetch --all --quiet || true
git -C "$WS/src/third_party/realsense-ros" checkout "$RS_ROS_COMMIT"
if ros2 pkg prefix open_manipulator_bringup >/dev/null 2>&1; then
  echo "ROBOTIS open_manipulator already installed at $(ros2 pkg prefix open_manipulator_bringup) - not cloning."
  echo "(Source that workspace BEFORE this one in every terminal.)"
else
  vcs import "$WS/src" < "$WS/omx_robotis.repos"
  # only the OpenMANIPULATOR-X bringup/description/teleop are needed: skip MoveIt, GUI, OMY controllers
  for d in open_manipulator open_manipulator_playground open_manipulator_moveit_config \
           open_manipulator_gui open_manipulator_collision ros2_controller; do
    [[ -d "$WS/src/robotis/open_manipulator/$d" ]] && touch "$WS/src/robotis/open_manipulator/$d/COLCON_IGNORE"
  done
fi

step "5. Dependencies (rosdep)"
cd "$WS"
rosdep install --from-paths src -i -y --rosdistro $DISTRO --skip-keys=librealsense2 || \
  echo "WARNING: rosdep reported missing keys above - check them before building."

step "6. Python packages"
python3 -m pip install --user --break-system-packages --no-deps -r "$WS/requirements.txt"
python3 -c "import cv2, numpy; print('OpenCV', cv2.__version__, '| NumPy', numpy.__version__); \
assert hasattr(cv2.aruco, 'ArucoDetector'); assert numpy.__version__.startswith('1.'), 'NumPy 2 in ~/.local: remove it'"

step "7. Build"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

step "8. ~/.bashrc"
add_line() { grep -qxF "$1" "$HOME/.bashrc" || echo "$1" >> "$HOME/.bashrc"; }
add_line "source /opt/ros/$DISTRO/setup.bash"
add_line "source $WS/install/setup.bash"
tail -3 "$HOME/.bashrc"

echo
echo "DONE. Open a NEW terminal (log out/in if you were added to dialout), then:"
echo "  ros2 run omx_calibration preflight"
echo "  ros2 launch omx_calibration bringup.launch.py       # terminal 1"
echo "  ros2 run omx_calibration wizard                     # terminal 2"
