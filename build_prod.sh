#!/bin/sh
set -eu
cd "$(dirname "$0")"

if ! command -v g++ >/dev/null 2>&1; then
  echo "ERROR: g++ 未安装；请安装 build-essential。" >&2
  exit 2
fi
if ! command -v make >/dev/null 2>&1; then
  echo "ERROR: make 未安装；请安装 build-essential。" >&2
  exit 2
fi
if ! command -v pkg-config >/dev/null 2>&1; then
  echo "ERROR: pkg-config 未安装；无法检测 OpenCV4 开发包。" >&2
  exit 2
fi
if ! pkg-config --exists opencv4; then
  echo "ERROR: 未检测到 OpenCV4 开发包（pkg-config 模块 opencv4）。请安装 libopencv-dev 并确认 opencv4.pc 可被 pkg-config 找到。" >&2
  exit 3
fi
echo "OpenCV4: $(pkg-config --modversion opencv4)"
make clean all
