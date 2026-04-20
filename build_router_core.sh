#!/usr/bin/env bash
set -euo pipefail

python_bin="${ROUTER_BUILD_PYTHON:-/usr/bin/python3}"
build_dir="${ROUTER_BUILD_DIR:-build-linux}"
cmake_exe="${CMAKE_EXE:-cmake}"

if [[ ! -x "$python_bin" ]]; then
    echo "Build Python was not found or is not executable: $python_bin"
    echo "Set ROUTER_BUILD_PYTHON to the Python used by KiCad pcbnew."
    exit 1
fi

if ! command -v "$cmake_exe" >/dev/null 2>&1; then
    echo "CMake was not found. Install CMake or set CMAKE_EXE."
    exit 1
fi

if ! "$python_bin" -c "import pybind11" >/dev/null 2>&1; then
    echo "Build Python does not have pybind11 installed."
    echo "Install the required packages first:"
    echo "  $python_bin -m pip install --user -r requirements.txt"
    exit 1
fi

pybind_cmake_dir="$("$python_bin" -m pybind11 --cmakedir)"

"$cmake_exe" -S . -B "$build_dir" \
    -Dpybind11_DIR="$pybind_cmake_dir" \
    -DPython_EXECUTABLE="$python_bin"

"$cmake_exe" --build "$build_dir"

echo "router_core build finished in $build_dir."
