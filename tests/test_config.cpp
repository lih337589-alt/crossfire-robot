/*
 * 配置层单元测试 —— 独立编译、独立运行，不依赖 OpenCV。
 *
 * 编译： g++ -std=c++17 -I../config test_config.cpp ../config/config.cpp -o test_config
 * 运行： ./test_config
 */
#include "config.h"

#include <cstdio>
#include <cassert>
#include <string>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::fprintf(stderr, "FAIL: %s\n", msg); } \
} while (0)

int main() {
    /* 1. 默认值 */
    {
        Config c;
        CHECK(c.kp == 0.6f, "默认 KP");
        CHECK(c.speed_run == 1670, "默认 SPEED_RUN");
        CHECK(c.blue.h_low == 95 && c.blue.h_high == 132, "默认 BLUE 阈值");
        CHECK(c.img_center() == 320, "img_center = roi_x1 + w/2 = 20+300");
        CHECK(c.roi_w() == 600 && c.roi_h() == 260, "ROI 尺寸");
        CHECK(c.is_race_mode(), "默认比赛模式");
        CHECK(c.field_length_m == 7.0f && c.field_width_m == 4.0f, "默认场地尺寸参数");
        CHECK(c.station_stay_ms >= 3000, "据点停留不低于3秒");
    }

    /* 2. 加载真实 config.ini */
    {
        Config c;
        bool ok = c.load("config.ini");
        CHECK(ok, "加载 config.ini 成功");
        CHECK(c.mode == "race", "读取 MODE");
        CHECK(c.blue_route == "A" && c.red_route == "B", "色卡映射");
        CHECK(c.tts_a.find("路线A") != std::string::npos, "TTS_A 文案");
        CHECK(c.pc_host == "192.168.137.1", "读取 PC_HOST");
        CHECK(c.cam_w == 1280 && c.cam_h == 720, "硬件摄像头处理分辨率为 720p");
        CHECK(c.roi_x1 == 40 && c.roi_y1 == 270 && c.roi_x2 == 1240 && c.roi_y2 == 660, "ROI 从参考分辨率自动缩放");
        CHECK(c.red_hit_y == 570 && c.red_hit_dx == 80, "近区代理阈值随分辨率缩放");
        CHECK(c.validate(), "配置通过完整性校验");
    }

    /* 3. 1080p camera profile scales pixel thresholds from the 640x480 reference. */
    {
        const char* tmp = "/tmp/cfg_test_1080p.ini";
        FILE* f = std::fopen(tmp, "w");
        std::fputs("CAM_W=1920\nCAM_H=1080\nPIXEL_REF_W=640\nPIXEL_REF_H=480\nCAMERA_DEVICE_PATH=/dev/video4\n", f);
        std::fclose(f);
        Config c;
        CHECK(c.load(tmp), "加载 1080p 配置成功");
        CHECK(c.roi_x2 == 1860 && c.roi_y2 == 990, "1080p ROI 缩放");
        CHECK(c.red_hit_y == 855 && c.red_hit_dx == 120, "1080p 近区阈值缩放");
        CHECK(c.min_area_red == 202.5f, "面积阈值按像素面积缩放");
        CHECK(c.camera_device_path == "/dev/video4", "摄像头设备路径配置");
    }

    /* 4. 非法值钳位 */
    {
        Config c;
        c.load_defaults();
        /* 写一个临时配置文件测钳位 */
        const char* tmp = "/tmp/cfg_test_illegal.ini";
        FILE* f = std::fopen(tmp, "w");
        std::fputs("KP=99999\nSPEED_RUN=99999\nROI_X1=-100\nMODE=debug\n", f);
        std::fclose(f);
        bool ok = c.load(tmp);
        CHECK(ok, "加载非法值文件");
        CHECK(c.kp == 100.0f, "KP 钳位到上限 100");
        CHECK(c.speed_run == 2500, "SPEED_RUN 钳位到 2500");
        CHECK(c.roi_x1 == 0, "ROI_X1 钳位到 0");
        CHECK(c.mode == "debug" && !c.is_race_mode(), "MODE=debug 非比赛模式");
    }

    /* 5. 缺失文件回退默认 */
    {
        Config c;
        c.load_defaults();
        c.kp = 0.0f;
        bool ok = c.load("/tmp/not_exist.ini");
        CHECK(!ok, "文件不存在返回 false");
        CHECK(c.kp == 0.0f, "加载失败不破坏已设值");
    }


    /* 6. Reject invalid hue bands and the red-range overlap that could double-count pixels. */
    {
        Config c;
        c.red1.h_high = 175;
        c.red2.h_low = 170;
        CHECK(!c.validate(), "重叠的红色 H 区间被拒绝");
        c = Config{};
        c.blue.h_low = 140;
        c.blue.h_high = 120;
        CHECK(!c.validate(), "反向的蓝色 H 区间被拒绝");
        c = Config{};
        c.station_stay_ms = 2999;
        CHECK(!c.validate(), "直接构造的配置也不能将据点停留降到 3 秒以下");
    }

    std::printf("config 测试结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
