#!/usr/bin/env bash
# =============================================================================
# setup_akida_venv.sh
#
# One-shot script to create the Python venv required by the Akida
# object-detection node (my_ugv_vision/object_detector).
#
# Run once after cloning / pulling this branch:
#   bash ugv_ws/setup_akida_venv.sh
#
# What it does:
#   1. Creates ugv_ws/akida_venv/ (Python 3.12, inherits system site-packages
#      so rclpy / cv_bridge from the ROS2 install are still reachable).
#   2. Installs akida==2.19.1, numpy, and opencv-python into the venv.
#      akida version is pinned to match ugv_object_detect_model.fbz.
#   3. Verifies the install and prints the detected Akida devices.
#
# After this script completes, launch the camera with:
#   ros2 launch my_ugv_vision akida_camera.launch.py
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/akida_venv"
PYTHON="${PYTHON:-python3}"

echo "==================================================================="
echo " Akida venv setup"
echo " Workspace : ${SCRIPT_DIR}"
echo " Venv path : ${VENV_DIR}"
echo "==================================================================="

# ── 1. Python version check ───────────────────────────────────────────
PY_VER=$("${PYTHON}" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "[1/4] Python version: ${PY_VER}"
if [[ "${PY_VER}" != "3.12" ]]; then
    echo "      WARNING: tested with Python 3.12; you have ${PY_VER}."
    echo "      Adjust the AKIDA_VENV_SITE path in:"
    echo "        ugv_ws/src/my_ugv_vision/launch/akida_camera.launch.py"
    echo "      if the venv uses a different minor version."
fi

# ── 2. Create the venv ────────────────────────────────────────────────
echo "[2/4] Creating venv at ${VENV_DIR} …"
if [[ -d "${VENV_DIR}" ]]; then
    echo "      Venv already exists — skipping creation."
else
    "${PYTHON}" -m venv "${VENV_DIR}" --system-site-packages
    echo "      Done."
fi

# Prevent colcon from scanning inside the venv (avoids numpy/akida CMake errors)
touch "${VENV_DIR}/COLCON_IGNORE"

PIP="${VENV_DIR}/bin/pip"
VENV_PYTHON="${VENV_DIR}/bin/python"

# ── 3. Install dependencies ───────────────────────────────────────────
echo "[3/4] Installing akida==2.19.1, numpy, opencv-python …"
"${PIP}" install --upgrade pip --quiet
# akida version is pinned to match ugv_object_detect_model.fbz (v2.19.1)
"${PIP}" install "akida==2.19.1" numpy opencv-python --quiet
echo "      Done."

# ── 4. Verify ─────────────────────────────────────────────────────────
echo "[4/4] Verifying install …"
"${VENV_PYTHON}" - <<'PYEOF'
import akida, numpy, cv2
print(f"  akida        {akida.__version__}")
print(f"  numpy        {numpy.__version__}")
print(f"  opencv       {cv2.__version__}")

devices = akida.devices()
if devices:
    print(f"  Akida HW     {devices[0].desc}")
else:
    print("  Akida HW     none detected (software emulation will be used)")
PYEOF

echo ""
echo "==================================================================="
echo " Setup complete."
echo ""
echo " To launch the camera with Akida detection:"
echo "   ros2 launch my_ugv_vision akida_camera.launch.py"
echo ""
echo " To launch without Akida (camera only):"
echo "   ros2 launch my_ugv_vision akida_camera.launch.py use_akida:=false"
echo "==================================================================="
