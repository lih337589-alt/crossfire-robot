#include "event_log.h"

void EventLog::reset() { recs_.clear(); }
void EventLog::record(EventType t, int64_t elapsed_ms, int penalty_s) {
    recs_.push_back({t, elapsed_ms < 0 ? 0 : elapsed_ms, penalty_s});
}
bool EventLog::failed() const {
    for (const auto& r : recs_) {
        if (r.type == EventType::RED_CONE_HIT || r.type == EventType::TIMEOUT ||
            r.type == EventType::SENSOR_FAILURE || r.type == EventType::SAFETY_STOP ||
            r.type == EventType::ANNOUNCE_FAILED || r.type == EventType::OPERATOR_ABORT) return true;
    }
    return false;
}
int EventLog::total_penalty_s() const {
    int sum = 0;
    for (const auto& r : recs_) sum += r.penalty_s;
    return sum;
}
int EventLog::count() const { return static_cast<int>(recs_.size()); }
const std::vector<EventRecord>& EventLog::records() const { return recs_; }
const char* EventLog::name(EventType t) {
    switch (t) {
        case EventType::COLOR_BLUE: return "COLOR_BLUE";
        case EventType::COLOR_RED: return "COLOR_RED";
        case EventType::ANNOUNCE_DONE: return "ANNOUNCE_DONE";
        case EventType::ANNOUNCE_FAILED: return "ANNOUNCE_FAILED";
        case EventType::START: return "START";
        case EventType::ARRIVE_STATION: return "ARRIVE_STATION";
        case EventType::STATION_STAY_DONE: return "STATION_STAY_DONE";
        case EventType::ALIGN_DONE: return "ALIGN_DONE";
        case EventType::CAMP_SEEN: return "CAMP_SEEN";
        case EventType::CAMP_STOPPED: return "CAMP_STOPPED";
        case EventType::RED_CONE_HIT: return "RED_CONE_HIT";
        case EventType::BLUE_CONE_HIT: return "BLUE_CONE_HIT";
        case EventType::FENCE_HIT: return "FENCE_HIT";
        case EventType::TIMEOUT: return "TIMEOUT";
        case EventType::SENSOR_FAILURE: return "SENSOR_FAILURE";
        case EventType::SAFETY_STOP: return "SAFETY_STOP";
        case EventType::VISION_DEGRADED: return "VISION_DEGRADED";
        case EventType::IMU_DEGRADED: return "IMU_DEGRADED";
        case EventType::OPERATOR_ABORT: return "OPERATOR_ABORT";
    }
    return "UNKNOWN";
}
