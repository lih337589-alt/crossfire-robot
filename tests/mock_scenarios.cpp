#include "../decision/state_machine.h"
#include "../perception/cone_logic.h"
#include <cstdio>

static int pass = 0, fail = 0;
#define CHECK(c, m) do { if (c) ++pass; else { ++fail; std::fprintf(stderr, "FAIL: %s\n", m); } } while (0)

static void start_route(StateMachine& fsm) {
    CHECK(fsm.on_event(EventType::COLOR_BLUE, 100), "card recognition enters announce");
    CHECK(fsm.on_event(EventType::ANNOUNCE_DONE, 200), "announcement acknowledged");
    CHECK(fsm.on_event(EventType::START, 300), "timer start");
}

int main() {
    // Scenario 0: crossing cones disappear mid-run; never issue approach-speed behavior.
    CHECK(decide_cone_loss_action(1, 4) == ConeLossAction::LOW_SPEED_SEARCH,
          "中途丢失锥桶后进入低速搜索");
    CHECK(decide_cone_loss_action(4, 4) == ConeLossAction::STOP_AND_WAIT,
          "持续丢失锥桶后 Mock 控制策略停车等待");
    // Scenario 1: red cone near-contact proxy -> terminal failure.
    {
        StateMachine fsm; start_route(fsm);
        fsm.on_event(EventType::ARRIVE_STATION, 1000);
        CHECK(!fsm.on_event(EventType::STATION_STAY_DONE, 3999), "stay <3s is rejected");
        CHECK(fsm.on_event(EventType::STATION_STAY_DONE, 4000), "stay >=3s is accepted");
        fsm.on_event(EventType::ALIGN_DONE, 5000);
        CHECK(fsm.on_event(EventType::RED_CONE_HIT, 5100), "red cone event accepted");
        CHECK(fsm.state() == State::FAILED && fsm.log().failed(), "red cone forces FAILED");
    }
    // Scenario 2: blue cone and fence incidents add 5 seconds each without terminal failure.
    {
        StateMachine fsm; start_route(fsm);
        fsm.on_event(EventType::ARRIVE_STATION, 1000);
        fsm.on_event(EventType::STATION_STAY_DONE, 4000);
        fsm.on_event(EventType::ALIGN_DONE, 5000);
        CHECK(fsm.on_event(EventType::BLUE_CONE_HIT, 5100), "blue cone event accepted");
        CHECK(fsm.on_event(EventType::FENCE_HIT, 5200), "fence event accepted");
        CHECK(fsm.log().total_penalty_s() == 10, "two penalties total 10s");
        CHECK(fsm.state() == State::CROSSING, "penalty does not fail mission");
    }
    // Scenario 3: camera/sensor failure and timeout fail closed.
    {
        StateMachine fsm; start_route(fsm);
        CHECK(fsm.on_event(EventType::SENSOR_FAILURE, 700), "sensor failure event accepted");
        CHECK(fsm.state() == State::FAILED, "sensor failure stops mission");
    }
    {
        StateMachine fsm; start_route(fsm);
        CHECK(fsm.on_event(EventType::TIMEOUT, 99999), "timeout accepted");
        CHECK(fsm.state() == State::FAILED, "timeout enters FAILED");
    }
    // Scenario 4: announcement failure must not permit launch.
    {
        StateMachine fsm;
        fsm.on_event(EventType::COLOR_RED, 10);
        CHECK(fsm.on_event(EventType::ANNOUNCE_FAILED, 20), "announcement failure accepted");
        CHECK(fsm.state() == State::FAILED, "failed speech inhibits launch");
        CHECK(!fsm.on_event(EventType::START, 30), "no start after failed speech");
    }
    std::printf("Mock 场景结果: %d 通过, %d 失败\n", pass, fail);
    return fail == 0 ? 0 : 1;
}
