#!/usr/bin/env bash
# One-shot Debian/WSL2 setup for the host (docs/BUILD_GUIDE.md Part 2.3-2.10). Safe to run again:
# every step checks first and only does what is missing.
#
#   host/scripts/setup_env.sh               install what is missing, then verify (Part 2.10)
#   host/scripts/setup_env.sh --dry-run     print what it would do; change nothing
#   host/scripts/setup_env.sh --check-only  only the verification
#   options: --no-livox  --no-python  --no-platformio
#
# Does NOT install CUDA, cuDNN or TensorRT (Part 2.4: use the versions already validated on this
# machine; never mix major versions) and does NOT build OpenCV (Part 2.5: a long CUDA source build,
# CUDA_ARCH_BIN=12.0 for Blackwell, run deliberately). It checks both at the end.
#
# The apt list is what the build actually finds (CMakeLists.txt find_package calls), plus the
# GStreamer plugins the camera pipelines use (MJPG decode, Part 6) and the OSRM tools (Part 2.7).
# Livox SDK2 is built from source into ~/.local/livox, no sudo (Part 2.8, amended): the host's
# CMake looks there.
set -euo pipefail

DRY=0 CHECK_ONLY=0 LIVOX=1 PYTHON=1 PIO=1
for a in "$@"; do
    case "$a" in
        --dry-run) DRY=1 ;;
        --check-only) CHECK_ONLY=1 ;;
        --no-livox) LIVOX=0 ;;
        --no-python) PYTHON=0 ;;
        --no-platformio) PIO=0 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "unknown option $a (see --help)" >&2; exit 2 ;;
    esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LIVOX_PREFIX="$HOME/.local/livox"
LIVOX_SRC="$HOME/src/Livox-SDK2"

run() {  # print, and run unless --dry-run
    echo "+ $*"
    if [ "$DRY" -eq 0 ]; then "$@"; fi
}
step() { printf '\n=== %s\n' "$*"; }

APT_PACKAGES=(
    # toolchain (2.3)
    build-essential cmake git pkg-config curl wget unzip python3-pip python3-venv
    # libraries the build finds
    libeigen3-dev libglm-dev libspdlog-dev libsdl2-dev libglew-dev libgl-dev
    flatbuffers-compiler libflatbuffers-dev libyaml-cpp-dev libgtest-dev libgoogle-glog-dev
    libpcl-dev                        # 2.6
    # camera: GStreamer, its MJPG/YUYV decoders, and V4L2 tools (Part 6)
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base
    gstreamer1.0-plugins-good gstreamer1.0-libav v4l-utils ffmpeg
    osrm-tools                        # 2.7
)

install_apt() {
    step "Debian packages (Part 2.3, 2.6, 2.7)"
    local missing=()
    for p in "${APT_PACKAGES[@]}"; do
        dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "install ok installed" && continue
        # Built from source instead (as OSRM is on the development machine): leave it be.
        [ "$p" = osrm-tools ] && command -v osrm-extract >/dev/null 2>&1 && continue
        missing+=("$p")
    done
    if [ "${#missing[@]}" -eq 0 ]; then
        echo "all ${#APT_PACKAGES[@]} packages present"
        return
    fi
    echo "missing: ${missing[*]}"
    run sudo apt-get update
    run sudo apt-get install -y "${missing[@]}"
}

install_groups() {
    step "Device access: the hub's serial port and the camera"
    # /dev/ttyACM* belongs to dialout, /dev/video* to video. Without the groups the host needs
    # sudo, which it must never run under.
    for g in dialout video; do
        if id -nG "$USER" | tr ' ' '\n' | grep -qx "$g"; then
            echo "$USER is in $g"
        else
            run sudo usermod -aG "$g" "$USER"
            echo "  (takes effect at the next login: wsl --shutdown, then reopen)"
        fi
    done
    # Appendix D: ModemManager probes new ACM devices with AT commands, which the hub reads as
    # frame errors (fault 4) for a second or two after every connect.
    if systemctl is-active --quiet ModemManager 2>/dev/null; then
        run sudo systemctl disable --now ModemManager
    else
        echo "ModemManager not running"
    fi
}

install_livox() {
    step "Livox SDK2 into $LIVOX_PREFIX (Part 2.8)"
    if [ -f "$LIVOX_PREFIX/include/livox_lidar_api.h" ] && ls "$LIVOX_PREFIX"/lib/liblivox_lidar_sdk_static.* >/dev/null 2>&1; then
        echo "already installed"
        return
    fi
    if [ ! -d "$LIVOX_SRC" ]; then
        run mkdir -p "$(dirname "$LIVOX_SRC")"
        run git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$LIVOX_SRC"
    fi
    run cmake -S "$LIVOX_SRC" -B "$LIVOX_SRC/build" -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$LIVOX_PREFIX"
    run cmake --build "$LIVOX_SRC/build" -j"$(nproc)"
    run cmake --install "$LIVOX_SRC/build"
    echo "  the host build finds it at the next 'cmake -S . -B build'"
}

install_python() {
    step "Python packages for the tools (host/scripts/requirements.txt)"
    if [ -z "${VIRTUAL_ENV:-}" ]; then
        echo "no virtual environment active: activate the one you use for this project (Debian's"
        echo "system Python refuses pip installs), then run again; skipped"
        return
    fi
    run python3 -m pip install -r "$REPO/host/scripts/requirements.txt"
}

install_platformio() {
    step "PlatformIO for the firmware (Part 2.9)"
    if command -v pio >/dev/null 2>&1; then
        echo "pio present: $(pio --version 2>/dev/null)"
        return
    fi
    if [ -z "${VIRTUAL_ENV:-}" ]; then
        echo "no virtual environment active: activate one and run again; skipped"
        return
    fi
    run python3 -m pip install platformio
    run pio pkg install --global --platform ststm32
}

# ---------------------------------------------------------------------------------------------
# Part 2.10: verify. Each check prints PASS / FAIL / WARN; the script's status is 1 on any FAIL.
FAILS=0
check() {  # check "what" command...
    local what="$1"; shift
    if "$@" >/dev/null 2>&1; then echo "PASS $what"; else echo "FAIL $what"; FAILS=$((FAILS + 1)); fi
}
warn() { echo "WARN $*"; }

verify() {
    step "Verification (Part 2.10)"
    check "nvcc (CUDA toolkit, Part 2.4)" command -v nvcc
    if command -v nvidia-smi >/dev/null 2>&1; then
        local cap
        cap="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1)"
        if [ "$cap" = "12.0" ]; then echo "PASS GPU compute capability 12.0 (Blackwell)"
        else warn "GPU compute capability '$cap' (the guide expects 12.0; CUDA_ARCH_BIN must match)"; fi
    else
        echo "FAIL nvidia-smi (no GPU visible in WSL2)"; FAILS=$((FAILS + 1))
    fi
    check "TensorRT (libnvinfer, Part 2.4)" sh -c "ldconfig -p | grep -q libnvinfer.so"
    if pkg-config --exists opencv4 2>/dev/null; then
        # 4.10 or newer (docs/decisions.md: a 4.14 source build is accepted; 4.10 predates CUDA 13),
        # and built WITH CUDA: its CUDA modules exist.
        local v; v="$(pkg-config --modversion opencv4)"
        if printf '4.10.0\n%s\n' "$v" | sort -V -C; then echo "PASS OpenCV $v (>= 4.10)"
        else echo "FAIL OpenCV $v (the build needs 4.10 or newer, Part 2.5)"; FAILS=$((FAILS + 1)); fi
        check "OpenCV built with CUDA (libopencv_cudaimgproc)" sh -c "ldconfig -p | grep -q libopencv_cudaimgproc"
    else
        echo "FAIL OpenCV 4 (pkg-config opencv4, Part 2.5)"; FAILS=$((FAILS + 1))
    fi
    check "GStreamer (gst-inspect-1.0 jpegdec, Part 6)" gst-inspect-1.0 jpegdec
    check "OSRM tools (osrm-extract, Part 2.7)" command -v osrm-extract
    check "Livox SDK2 in $LIVOX_PREFIX (Part 2.8)" test -f "$LIVOX_PREFIX/include/livox_lidar_api.h"
    check "PlatformIO (pio, Part 2.9)" command -v pio
    if ls /dev/video* >/dev/null 2>&1; then echo "PASS a camera at $(ls /dev/video* | head -1)"
    else warn "no /dev/video*: attach the camera (attach_usb_devices.ps1, Part 2.2) when it is here"; fi
    if ls /dev/ttyACM* >/dev/null 2>&1; then echo "PASS a USB serial port at $(ls /dev/ttyACM* | head -1)"
    else warn "no /dev/ttyACM*: attach the hub (attach_usb_devices.ps1) when it is here"; fi
    if grep -qs "networkingMode=mirrored" "/mnt/c/Users/${WINUSER:-$USER}/.wslconfig" 2>/dev/null ||
       ip -o addr 2>/dev/null | grep -q "192.168.1."; then
        echo "PASS networking: mirrored, or an address on the LiDAR's subnet"
    else
        warn "WSL2 mirrored networking not confirmed (Part 2.1); the LiDAR needs it (Appendix C)"
    fi
    echo
    if [ "$FAILS" -eq 0 ]; then echo "environment OK"; else echo "$FAILS check(s) FAILED"; fi
}

if [ "$CHECK_ONLY" -eq 0 ]; then
    [ "$DRY" -eq 1 ] && echo "(dry run: nothing is changed)"
    install_apt
    install_groups
    [ "$LIVOX" -eq 1 ] && install_livox
    [ "$PYTHON" -eq 1 ] && install_python
    [ "$PIO" -eq 1 ] && install_platformio
fi
verify
[ "$FAILS" -eq 0 ]
