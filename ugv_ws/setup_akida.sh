#!/usr/bin/env bash
# =============================================================================
# setup_akida.sh
#
# Installs the BrainChip Akida Python dependencies so the system Python
# used by ROS2 can import them — no virtual environment required.
#
# Run once after cloning / checking out the branch:
#   bash ugv_ws/setup_akida.sh
#
# What it does:
#   1. Locates a pip binary (system pip, apt, or pip from PATH).
#   2. Installs akida==2.19.1, numpy, and opencv-python directly into the
#      user site-packages directory (~/.local/lib/pythonX.Y/site-packages).
#      No sudo required.  akida is pinned to match ugv_object_detect_model.fbz.
#   3. Verifies the install with the system Python.
#
# After this script completes, build and launch normally:
#   cd ugv_ws && colcon build --symlink-install
#   source install/setup.bash
#   ros2 launch my_ugv_vision akida_camera.launch.py
# =============================================================================
set -euo pipefail

PYTHON="${PYTHON:-python3}"
PACKAGES=("akida==2.19.1" "numpy" "opencv-python")

echo "==================================================================="
echo " Akida dependency setup  (no venv)"
echo " Python : $("${PYTHON}" --version)"
echo " Target : user site-packages"
echo "==================================================================="

# ── 1. Python version check ───────────────────────────────────────────
PY_VER=$("${PYTHON}" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "[1/3] Python version: ${PY_VER}"
if [[ "${PY_VER}" < "3.10" ]]; then
    echo "ERROR: Python 3.10+ required (found ${PY_VER})."
    exit 1
fi

# ── 2. Locate or install pip ──────────────────────────────────────────
echo "[2/3] Locating pip …"

# Resolve user site-packages path from Python itself (always correct)
USER_SITE=$("${PYTHON}" -c "import site; print(site.getusersitepackages())")
mkdir -p "${USER_SITE}"
echo "      User site-packages: ${USER_SITE}"

# Find any available pip — prefer system Python's, fall back to any pip on PATH
PIP_CMD=""

if "${PYTHON}" -m pip --version &>/dev/null 2>&1; then
    PIP_CMD="${PYTHON} -m pip"
    echo "      Found: ${PIP_CMD}"
elif command -v pip3 &>/dev/null; then
    PIP_CMD="pip3"
    echo "      Found: pip3"
elif command -v pip &>/dev/null; then
    PIP_CMD="pip"
    echo "      Found: pip"
else
    # Try apt — works on Ubuntu/Debian with sudo
    echo "      No pip on PATH. Trying: sudo apt-get install -y python3-pip"
    if sudo apt-get install -y python3-pip 2>/dev/null; then
        PIP_CMD="${PYTHON} -m pip"
    else
        echo "      apt unavailable or no sudo. Cannot install pip automatically."
        echo "      Please install pip manually:"
        echo "        sudo apt-get install python3-pip"
        echo "      Then re-run this script."
        exit 1
    fi
fi

# ── 3. Install packages directly to user site-packages ────────────────
echo "[3/3] Installing ${PACKAGES[*]} …"
echo "      Using: ${PIP_CMD}"

# --target installs directly into the user site-packages directory.
# This works with ANY pip binary regardless of which Python it belongs to,
# and requires no sudo.
${PIP_CMD} install \
    --target "${USER_SITE}" \
    --upgrade \
    --quiet \
    "${PACKAGES[@]}"

echo "      Done."

# ── Verify ────────────────────────────────────────────────────────────
echo ""
echo " Verifying with system Python …"
"${PYTHON}" - <<'PYEOF'
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
echo " Build and launch:"
echo "   cd ugv_ws && colcon build --symlink-install"
echo "   source install/setup.bash"
echo "   ros2 launch my_ugv_vision akida_camera.launch.py"
echo "==================================================================="
