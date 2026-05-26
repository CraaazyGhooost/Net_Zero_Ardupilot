#!/bin/bash
# flash_mico.sh — Flash ArduPilot firmware to MicoAir743v2 (STM32H743) under WSL
#
# Usage:
#   ./flash_mico.sh [upload|dfu|build|setup]
#
#   upload  — Build + upload via serial bootloader (normal update, default)
#   dfu     — Flash via STM32 DFU mode (first time, recovery, or bootloader update)
#   build   — Build only (no flash)
#   setup   — Install prerequisites (dfu-util, etc.)
#
# Modes explained:
#   - "upload" uses the ArduPilot bootloader protocol over a serial port. After the
#     first flash with DFU, this is the daily workflow. In WSL2, this transparently
#     uses Windows Python to access COM ports.
#   - "dfu" requires the board to be in DFU mode: hold the BOOT button, then plug in
#     USB. The STM32 enumerates as a DFU device (USB ID 0483:df11). This mode is
#     used once to put the ArduPilot bootloader on the board.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BOARD="MicoAir743v2"
BUILD_DIR="${SCRIPT_DIR}/build/${BOARD}"
APJ="${BUILD_DIR}/bin/arducopter.apj"
HEX="${BUILD_DIR}/bin/arducopter_with_bl.hex"
BIN="${BUILD_DIR}/bin/arducopter.bin"
STLINK_CFG="${SCRIPT_DIR}/Tools/debug/openocd-h7.cfg"

# Colours
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

usage() {
    echo "Usage: $0 [upload|dfu|build|setup] [--port COMx]"
    exit 1
}

setup_prereqs() {
    echo -e "${CYAN}==> Installing prerequisites...${NC}"

    # dfu-util (Linux/WSL side)
    if ! command -v dfu-util &>/dev/null; then
        echo "Installing dfu-util..."
        sudo apt-get update -qq && sudo apt-get install -y dfu-util
    else
        echo -e "${GREEN}  dfu-util already installed${NC}"
    fi

    # Windows Python check (for serial upload via WSL2)
    if [[ "$(uname -r)" == *microsoft* || "$(uname -r)" == *WSL* ]]; then
        echo ""
        echo -e "${YELLOW}WSL detected. For serial upload to work, Windows needs:${NC}"
        echo "  1. Python 3.9+ installed on Windows (not WSL)"
        echo "  2. Run in PowerShell/CMD as Administrator:"
        echo "     pip install pyserial empy"
        echo ""
        echo -e "${YELLOW}For DFU mode via WSL, you need usbipd to attach the USB device:${NC}"
        echo "  In Windows PowerShell (Admin):"
        echo "    usbipd wsl list"
        echo "    usbipd wsl attach --busid <BUSID>"
        echo ""
        echo -e "${YELLOW}Alternative for DFU: use Windows-native dfu-util:${NC}"
        echo "  Download from: https://dfu-util.sourceforge.net/"
        echo "  Then: dfu-util.exe -a 0 -D build\\MicoAir743v2\\bin\\arducopter_with_bl.hex"
    fi
}

# ── Build ──────────────────────────────────────────────────────────────

do_build() {
    echo -e "${CYAN}==> Building ArduCopter for ${BOARD}...${NC}"
    cd "${SCRIPT_DIR}"
    ./waf configure --board "${BOARD}"
    ./waf copter

    if [[ -f "${APJ}" ]]; then
        echo -e "${GREEN}  Build OK${NC}"
        echo "  APJ:  $(du -h "${APJ}" | cut -f1)"
        echo "  HEX:  $(du -h "${HEX}" | cut -f1)"
        echo "  BIN:  $(du -h "${BIN}" | cut -f1)"
    else
        echo -e "${RED}  Build failed: ${APJ} not found${NC}"
        exit 1
    fi
}

# ── DFU Flash ──────────────────────────────────────────────────────────

# Ensure the combined bootloader+firmware hex exists.
# The waf build normally generates this, but if not, we create it here.
_generate_combined_hex() {
    local hex_file="${HEX}"
    local bin_file="${BIN}"
    local bl_bin="${SCRIPT_DIR}/Tools/bootloaders/${BOARD}_bl.bin"
    local make_hex="${SCRIPT_DIR}/Tools/scripts/make_intel_hex.py"

    if [[ -f "${hex_file}" ]]; then
        return 0
    fi

    if [[ ! -f "${bin_file}" ]]; then
        echo -e "${RED}Firmware .bin not found at ${bin_file}${NC}"
        return 1
    fi
    if [[ ! -f "${bl_bin}" ]]; then
        echo -e "${RED}Bootloader not found at ${bl_bin}${NC}"
        return 1
    fi

    echo -e "${YELLOW}Generating combined bootloader+firmware hex...${NC}"
    python3 "${make_hex}" "${bin_file}" 128
    # make_intel_hex.py outputs the hex alongside the bin
    if [[ -f "${hex_file}" ]]; then
        echo -e "${GREEN}  Combined hex generated: $(du -h "${hex_file}" | cut -f1)${NC}"
    else
        echo -e "${RED}  Failed to generate ${hex_file}${NC}"
        return 1
    fi
}

do_dfu() {
    local hex_file="${HEX}"

    if [[ ! -f "${BIN}" ]]; then
        echo -e "${YELLOW}Firmware not built yet, building now...${NC}"
        do_build
    fi

    _generate_combined_hex || exit 1

    echo -e "${CYAN}==> DFU Flash for ${BOARD}${NC}"
    echo ""
    echo -e "${YELLOW}Instructions:${NC}"
    echo "  1. Disconnect USB from the board"
    echo "  2. Hold down the BOOT button"
    echo "  3. Plug in USB while holding BOOT"
    echo "  4. Release BOOT button after 2 seconds"
    echo "  5. Press Enter to continue..."
    read -r

    # Check if dfu-util can see the device
    echo ""
    echo -e "${CYAN}Looking for STM32 DFU device...${NC}"

    if ! command -v dfu-util &>/dev/null; then
        echo -e "${RED}dfu-util not found. Run '$0 setup' first.${NC}"
        exit 1
    fi

    # List DFU devices
    if dfu-util -l 2>&1 | grep -q "Found DFU"; then
        echo -e "${GREEN}  DFU device found${NC}"
    else
        echo -e "${RED}  No DFU device found!${NC}"
        echo ""
        echo -e "${YELLOW}If you're on WSL, the USB device may not be attached. Try:${NC}"
        echo "  1. In Windows PowerShell (Admin), run:"
        echo "     usbipd wsl list"
        echo "     usbipd wsl attach --busid <BUSID for STM32 DFU>"
        echo ""
        echo "  2. OR use Windows-native dfu-util:"
        echo "     In Windows CMD/PowerShell:"
        echo "     dfu-util.exe -a 0 -D $(wslpath -w "${hex_file}")"
        echo ""
        exit 1
    fi

    echo ""
    echo -e "${CYAN}Erasing and flashing firmware + bootloader...${NC}"
    echo "  File: ${hex_file}"

    # Erase chip first, then flash the hex (includes bootloader at 0x08000000)
    dfu-util -a 0 -s 0x08000000:mass-erase:force:leave -D "${hex_file}" || {
        # Alternative: flash without mass erase if the above fails
        echo -e "${YELLOW}  Mass erase failed, trying direct flash...${NC}"
        dfu-util -a 0 -D "${hex_file}"
    }

    echo ""
    echo -e "${GREEN}DFU flash complete!${NC}"
    echo "  Disconnect and reconnect the board (without BOOT button)."
    echo "  The board should now start ArduPilot with the new bootloader."
}

# ── Serial Upload ──────────────────────────────────────────────────────

do_upload() {
    local port="${1:-}"

    echo -e "${CYAN}==> Uploading via serial bootloader...${NC}"

    if [[ ! -f "${APJ}" ]]; then
        echo -e "${YELLOW}Firmware not built yet, building now...${NC}"
        do_build
    fi

    cd "${SCRIPT_DIR}"
    ./waf configure --board "${BOARD}"

    if [[ -n "${port}" ]]; then
        echo "  Port: ${port}"
        ./waf copter --upload --upload-port "${port}"
    else
        echo "  Auto-detecting port..."
        ./waf copter --upload
    fi
}

# ── Main ───────────────────────────────────────────────────────────────

MODE="${1:-upload}"
PORT=""

# Parse --port if provided
if [[ $# -ge 3 ]] && [[ "$2" == "--port" ]]; then
    PORT="$3"
fi

case "${MODE}" in
    upload)
        do_upload "${PORT}"
        ;;
    dfu)
        do_dfu
        ;;
    build)
        do_build
        ;;
    setup)
        setup_prereqs
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        # Maybe the user passed a port directly
        if [[ "${MODE}" == /dev/* ]] || [[ "${MODE}" == COM* ]]; then
            do_upload "${MODE}"
        else
            echo -e "${RED}Unknown mode: ${MODE}${NC}"
            usage
        fi
        ;;
esac
