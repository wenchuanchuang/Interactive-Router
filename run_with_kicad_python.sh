#!/usr/bin/env bash
set -euo pipefail

python_bin="${KICAD_PYTHON:-/usr/bin/python3}"

if [[ ! -x "$python_bin" ]]; then
    echo "KiCad Python was not found or is not executable: $python_bin"
    echo "Set KICAD_PYTHON to the Python that can import pcbnew."
    exit 1
fi

if ! "$python_bin" -c "import pcbnew" >/dev/null 2>&1; then
    echo "The selected Python cannot import pcbnew: $python_bin"
    echo "On Ubuntu with KiCad packages, this is usually /usr/bin/python3."
    exit 1
fi

if ! "$python_bin" -c "import PyQt5" >/dev/null 2>&1; then
    echo "The selected Python does not have PyQt5 installed."
    echo "Install the required packages first:"
    echo "  $python_bin -m pip install --user -r requirements.txt"
    exit 1
fi

needs_build=0
router_core="$(find build-linux -maxdepth 1 -name 'router_core*.so' -print -quit 2>/dev/null || true)"
if [[ -z "$router_core" ]]; then
    needs_build=1
elif find cpp_core CMakeLists.txt -type f -newer "$router_core" -print -quit | grep -q .; then
    needs_build=1
fi

if [[ "$needs_build" -eq 1 ]]; then
    ./build_router_core.sh
fi

exec "$python_bin" -m router_app.main "$@"
