#include "state_machine.h"

void StateMachine::reset() {
    st_ = State::IDLE;
    log_.reset();
    station_stay_start_ms_ = 0;
    start_ts_ms_ = 0;
    started_ = false;
    min_station_stay_ms_ = 3000;
}

int64_t StateMachine::elapsed_ms(int64_t now_ms) const {
    if (!started_ || now_ms <= start_ts_ms_) return 0;
    return now_ms - start_ts_ms_;
}

void StateMachine::record(EventType e, int64_t now_ms, int penalty_s) {
    log_.record(e, started_ ? now_ms - start_ts_ms_ : 0, penalty_s);
}

void StateMachine::fail(EventType e, int64_t now_ms) {
    record(e, now_ms);
    st_ = State::FAILED;
}

bool StateMachine::on_event(EventType e, int64_t now_ms) {
    if (is_terminal()) return false;

    switch (e) {
        case EventType::RED_CONE_HIT:
        case EventType::TIMEOUT:
        case EventType::SENSOR_FAILURE:
        case EventType::SAFETY_STOP:
        case EventType::ANNOUNCE_FAILED:
        case EventType::OPERATOR_ABORT:
            fail(e, now_ms);
            return true;
        case EventType::BLUE_CONE_HIT:
        case EventType::FENCE_HIT:
            if (!started_) return false;
            record(e, now_ms, 5);
            return true;
        case EventType::VISION_DEGRADED:
        case EventType::IMU_DEGRADED:
            record(e, now_ms);
            return true;
        default:
            break;
    }

    switch (st_) {
        case State::IDLE:
            if (e == EventType::COLOR_BLUE || e == EventType::COLOR_RED) {
                record(e, now_ms); st_ = State::ANNOUNCE; return true;
            }
            return false;
        case State::ANNOUNCE:
            if (e == EventType::ANNOUNCE_DONE) {
                record(e, now_ms); st_ = State::READY; return true;
            }
            return false;
        case State::READY:
            if (e == EventType::START) {
                started_ = true;
                start_ts_ms_ = now_ms;
                record(e, now_ms);
                st_ = State::TO_STATION;
                return true;
            }
            return false;
        case State::TO_STATION:
            if (e == EventType::ARRIVE_STATION) {
                record(e, now_ms); station_stay_start_ms_ = now_ms; st_ = State::STATION_STAY; return true;
            }
            return false;
        case State::STATION_STAY:
            if (e == EventType::STATION_STAY_DONE) {
                if (now_ms < station_stay_start_ms_ || now_ms - station_stay_start_ms_ < min_station_stay_ms_) return false;
                record(e, now_ms); st_ = State::ALIGN; return true;
            }
            return false;
        case State::ALIGN:
            if (e == EventType::ALIGN_DONE) { record(e, now_ms); st_ = State::CROSSING; return true; }
            return false;
        case State::CROSSING:
            if (e == EventType::CAMP_SEEN) { record(e, now_ms); st_ = State::TO_CAMP; return true; }
            return false;
        case State::TO_CAMP:
            if (e == EventType::CAMP_STOPPED) { record(e, now_ms); st_ = State::FINISHED; return true; }
            return false;
        case State::FINISHED:
        case State::FAILED:
            return false;
    }
    return false;
}

const char* StateMachine::state_name(State s) {
    switch (s) {
        case State::IDLE: return "IDLE";
        case State::ANNOUNCE: return "ANNOUNCE";
        case State::READY: return "READY";
        case State::TO_STATION: return "TO_STATION";
        case State::STATION_STAY: return "STATION_STAY";
        case State::ALIGN: return "ALIGN";
        case State::CROSSING: return "CROSSING";
        case State::TO_CAMP: return "TO_CAMP";
        case State::FINISHED: return "FINISHED";
        case State::FAILED: return "FAILED";
    }
    return "UNKNOWN";
}
