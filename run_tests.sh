#!/bin/sh
set -eu
cd "$(dirname "$0")"
CXX="${CXX:-g++}"
STD="-std=c++17 -Wall -Wextra -Wpedantic"
TMP="${TMPDIR:-/tmp}/crossfire_tests"
mkdir -p "$TMP"

echo "=== [1/7] 配置层 ==="
$CXX $STD -Iconfig tests/test_config.cpp config/config.cpp -o "$TMP/t_config"
(cd config && "$TMP/t_config" > "$TMP/config.log" 2>&1
tail -1 "$TMP/config.log")

echo "=== [2/7] 决策状态机 ==="
$CXX $STD -Idecision tests/test_state_machine.cpp decision/event_log.cpp decision/state_machine.cpp -o "$TMP/t_sm"
"$TMP/t_sm" > "$TMP/state_machine.log" 2>&1
tail -1 "$TMP/state_machine.log"

echo "=== [3/7] HAL + 控制适配（测试 OpenCV shim） ==="
$CXX $STD -Ihal -Itests/stub -Iconfig -Iapp tests/test_hal.cpp hal/hal_fake.cpp hal/voice_cmd.cpp control/drive_controller.cpp app/hardware_self_test.cpp -o "$TMP/t_hal"
"$TMP/t_hal" > "$TMP/hal.log" 2>&1
tail -1 "$TMP/hal.log"

echo "=== [4/7] 锥桶纯逻辑 ==="
$CXX $STD -Iperception tests/test_cone_logic.cpp perception/cone_logic.cpp -o "$TMP/t_cone"
"$TMP/t_cone" > "$TMP/cone.log" 2>&1
tail -1 "$TMP/cone.log"

echo "=== [5/7] 转向控制 ==="
$CXX $STD -Icontrol tests/test_steering.cpp control/steering.cpp -o "$TMP/t_steer"
"$TMP/t_steer" > "$TMP/steering.log" 2>&1
tail -1 "$TMP/steering.log"

echo "=== [6/7] 极端场景 Mock ==="
$CXX $STD -Idecision -Iperception tests/mock_scenarios.cpp decision/event_log.cpp decision/state_machine.cpp perception/cone_logic.cpp -o "$TMP/t_mock"
"$TMP/t_mock" > "$TMP/mock.log" 2>&1
tail -1 "$TMP/mock.log"

echo "=== [7/7] IMU 帧解析回归 ==="
CC="${CC:-gcc}"
$CC -std=gnu11 -Wall -Wextra -Idrivers tests/test_imu_parse.c drivers/imu.c -o "$TMP/t_imu"
"$TMP/t_imu" > "$TMP/imu.log" 2>&1
tail -1 "$TMP/imu.log"

echo "=== 语法校验（OpenCV test shim，仅校验 C++ 接口与语法） ==="
$CXX $STD -fsyntax-only -Itests/stub -Iapp -Iconfig -Ihal -Idecision -Iperception -Icontrol -Idrivers \
  app/main.cpp app/mission_controller.cpp app/hardware_self_test.cpp config/config.cpp hal/hal_linux.cpp hal/voice_cmd.cpp \
  decision/event_log.cpp decision/state_machine.cpp perception/color_detect.cpp perception/cone_detect.cpp \
  perception/camp_detect.cpp perception/cone_logic.cpp perception/vision_pipeline.cpp \
  control/steering.cpp control/drive_controller.cpp tests/vision_synthetic.cpp tests/vision_benchmark.cpp
if [ "$(uname -s)" = "Linux" ]; then
  # pca9685.c 依赖 <linux/i2c-dev.h>，仅 Linux 可语法校验
  gcc -std=gnu11 -Wall -Wextra -Wpedantic -fsyntax-only -Idrivers drivers/pid.c drivers/pca9685.c drivers/imu.c
else
  gcc -std=gnu11 -Wall -Wextra -Wpedantic -fsyntax-only -Idrivers drivers/pid.c drivers/imu.c
  echo "(非 Linux 环境：跳过 pca9685.c 的 linux/i2c-dev.h 语法校验)"
fi

echo ""
if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists opencv4; then
  echo "=== OpenCV synthetic vision scenarios ==="
  $CXX $STD -Iconfig -Iperception tests/vision_synthetic.cpp config/config.cpp \
    perception/vision_pipeline.cpp perception/cone_detect.cpp $(pkg-config --cflags --libs opencv4) -o "$TMP/t_vision"
  "$TMP/t_vision"
  echo "OpenCV4 detected. Production build command: make clean all"
else
  echo "OpenCV4 not installed in this environment; production image/HAL runtime was not tested."
  echo "The full C++ source passed syntax-only checks against a test shim; this does not replace a real OpenCV build."
  echo "On Orange Pi/RK3588: sudo apt install -y build-essential pkg-config libopencv-dev v4l-utils i2c-tools && make clean all"
fi
echo "=== 核心逻辑测试全部通过 ==="
