CXX ?= g++
CC ?= gcc
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -pthread
CFLAGS ?= -std=gnu11 -O2 -Wall -Wextra
CPPFLAGS += -Iapp -Iconfig -Ihal -Idecision -Iperception -Icontrol -Idrivers
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null)
OPENCV_LIBS := $(shell pkg-config --libs opencv4 2>/dev/null)
BUILD_DIR := build

CPP_SRCS := app/main.cpp app/mission_controller.cpp app/hardware_self_test.cpp \
 config/config.cpp hal/hal_linux.cpp hal/voice_cmd.cpp \
 decision/event_log.cpp decision/state_machine.cpp \
 perception/color_detect.cpp perception/cone_detect.cpp perception/camp_detect.cpp \
 perception/cone_logic.cpp perception/vision_pipeline.cpp \
 control/steering.cpp control/drive_controller.cpp
C_SRCS := drivers/pid.c drivers/pca9685.c drivers/imu.c
CPP_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CPP_SRCS))
C_OBJS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SRCS))

.PHONY: all clean test run self-test check-opencv bench-vision
all: check-opencv demo_new

check-opencv:
	@pkg-config --exists opencv4 || { echo "ERROR: OpenCV4 development package missing. Install libopencv-dev."; exit 1; }

DEPFLAGS = -MMD -MP

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(DEPFLAGS) $(OPENCV_CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DEPFLAGS) -c $< -o $@

demo_new: $(CPP_OBJS) $(C_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV_LIBS) -pthread -lstdc++fs

bench-vision: check-opencv
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(OPENCV_CFLAGS) tests/vision_benchmark.cpp config/config.cpp perception/vision_pipeline.cpp perception/cone_detect.cpp perception/cone_logic.cpp -o $(BUILD_DIR)/vision_benchmark $(OPENCV_LIBS)
	./$(BUILD_DIR)/vision_benchmark config/config.ini

test:
	./run_tests.sh

run: all
	./demo_new config/config.ini

self-test: all
	./demo_new --self-test config/config.ini

clean:
	rm -rf $(BUILD_DIR) demo_new

# Header dependency tracking: editing a .h now triggers the right rebuilds.
-include $(CPP_OBJS:.o=.d) $(C_OBJS:.o=.d)
