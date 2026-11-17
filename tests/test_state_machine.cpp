/*
 * 状态机 + 事件日志 单元测试（独立编译运行，不依赖 OpenCV/硬件）
 *
 * 编译：g++ -std=c++17 -Wall -Wextra -Idecision test_state_machine.cpp decision/event_log.cpp decision/state_machine.cpp -o test_state_machine
 */
#include "state_machine.h"

#include <cstdio>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::fprintf(stderr, "FAIL: %s\n", msg); } \
} while (0)

/* 完整流程：蓝卡 */
static void test_blue_full_flow() {
    StateMachine sm;
    CHECK(sm.state() == State::IDLE, "初始 IDLE");
    CHECK(sm.on_event(EventType::COLOR_BLUE, 0), "IDLE→蓝卡");
    CHECK(sm.state() == State::ANNOUNCE, "→ANNOUNCE");
    CHECK(sm.on_event(EventType::ANNOUNCE_DONE, 100), "播报完成");
    CHECK(sm.state() == State::READY, "→READY");
    CHECK(sm.on_event(EventType::START, 200), "发车");
    CHECK(sm.state() == State::TO_STATION, "→TO_STATION");
    CHECK(sm.on_event(EventType::ARRIVE_STATION, 1000), "到达据点");
    CHECK(sm.state() == State::STATION_STAY, "→STATION_STAY");
    CHECK(sm.on_event(EventType::STATION_STAY_DONE, 4500), "停留 3.5s 完成");
    CHECK(sm.state() == State::ALIGN, "→ALIGN");
    CHECK(sm.on_event(EventType::ALIGN_DONE, 5000), "回正完成");
    CHECK(sm.state() == State::CROSSING, "→CROSSING");
    CHECK(sm.on_event(EventType::CAMP_SEEN, 8000), "看到营地");
    CHECK(sm.state() == State::TO_CAMP, "→TO_CAMP");
    CHECK(sm.on_event(EventType::CAMP_STOPPED, 9000), "营地停车");
    CHECK(sm.state() == State::FINISHED, "→FINISHED");
    CHECK(sm.is_terminal(), "蓝卡流程终态");
    CHECK(sm.log().count() == 8, "蓝卡流程 8 个事件");
}

/* 完整流程：红卡 */
static void test_red_full_flow() {
    StateMachine sm;
    sm.on_event(EventType::COLOR_RED, 0);
    sm.on_event(EventType::ANNOUNCE_DONE, 100);
    sm.on_event(EventType::START, 200);
    sm.on_event(EventType::ARRIVE_STATION, 1000);
    sm.on_event(EventType::STATION_STAY_DONE, 4000);
    sm.on_event(EventType::ALIGN_DONE, 5000);
    sm.on_event(EventType::CAMP_SEEN, 8000);
    sm.on_event(EventType::CAMP_STOPPED, 9000);
    CHECK(sm.state() == State::FINISHED, "红卡流程完成");
}

/* 碰红锥 → FAILED */
static void test_red_cone_hit() {
    StateMachine sm;
    sm.on_event(EventType::COLOR_BLUE, 0);
    sm.on_event(EventType::ANNOUNCE_DONE, 100);
    sm.on_event(EventType::START, 200);
    sm.on_event(EventType::ARRIVE_STATION, 1000);
    sm.on_event(EventType::STATION_STAY_DONE, 4000);
    sm.on_event(EventType::ALIGN_DONE, 5000);   /* 进入 CROSSING */
    CHECK(sm.state() == State::CROSSING, "穿越中");
    CHECK(sm.on_event(EventType::RED_CONE_HIT, 6000), "碰红锥事件");
    CHECK(sm.state() == State::FAILED, "碰红锥→FAILED");
    CHECK(sm.log().failed(), "事件日志标记失败");
    CHECK(sm.is_terminal(), "失败为终态");
    /* 终态拒绝后续 */
    CHECK(!sm.on_event(EventType::CAMP_SEEN, 7000), "FAILED 拒绝后续事件");
}

/* 碰蓝锥/围栏：罚时，状态不变 */
static void test_penalty() {
    StateMachine sm;
    sm.on_event(EventType::COLOR_BLUE, 0);
    sm.on_event(EventType::ANNOUNCE_DONE, 100);
    sm.on_event(EventType::START, 200);
    sm.on_event(EventType::ARRIVE_STATION, 1000);
    sm.on_event(EventType::STATION_STAY_DONE, 4000);
    sm.on_event(EventType::ALIGN_DONE, 5000);   /* CROSSING */
    CHECK(sm.on_event(EventType::BLUE_CONE_HIT, 6000), "碰蓝锥事件");
    CHECK(sm.on_event(EventType::FENCE_HIT, 6100), "碰围栏事件");
    CHECK(sm.state() == State::CROSSING, "罚时事件不改变状态");
    CHECK(sm.log().total_penalty_s() == 10, "累计罚时 10 秒");
    CHECK(!sm.log().failed(), "罚时不等于失败");
}

/* 超时 → FAILED */
static void test_timeout() {
    StateMachine sm;
    sm.on_event(EventType::COLOR_BLUE, 0);
    sm.on_event(EventType::ANNOUNCE_DONE, 100);
    sm.on_event(EventType::START, 200);
    CHECK(sm.on_event(EventType::TIMEOUT, 60000), "超时事件");
    CHECK(sm.state() == State::FAILED, "超时→FAILED");
}

/* 停留不足 3 秒被拒绝 */
static void test_station_stay_min() {
    StateMachine sm;
    sm.on_event(EventType::COLOR_BLUE, 0);
    sm.on_event(EventType::ANNOUNCE_DONE, 100);
    sm.on_event(EventType::START, 200);
    sm.on_event(EventType::ARRIVE_STATION, 1000);
    CHECK(!sm.on_event(EventType::STATION_STAY_DONE, 2000), "停留 1s 被拒绝");
    CHECK(sm.state() == State::STATION_STAY, "仍在 STATION_STAY");
    CHECK(sm.on_event(EventType::STATION_STAY_DONE, 4100), "停留 3.1s 通过");
    CHECK(sm.state() == State::ALIGN, "→ALIGN");
}

/* 非法迁移：未播报直接发车被拒 */
static void test_illegal_start() {
    StateMachine sm;
    CHECK(!sm.on_event(EventType::START, 0), "IDLE 直接发车被拒");
    sm.on_event(EventType::COLOR_BLUE, 0);
    CHECK(!sm.on_event(EventType::START, 100), "ANNOUNCE 态发车被拒");
    sm.on_event(EventType::ANNOUNCE_DONE, 200);
    CHECK(sm.on_event(EventType::START, 300), "READY 态发车合法");
}

int main() {
    test_blue_full_flow();
    test_red_full_flow();
    test_red_cone_hit();
    test_penalty();
    test_timeout();
    test_station_stay_min();
    test_illegal_start();

    std::printf("状态机测试结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
