#ifndef CROSSFIRE_EVENT_LOG_H
#define CROSSFIRE_EVENT_LOG_H

#include <cstdint>
#include <vector>

enum class EventType {
    COLOR_BLUE, COLOR_RED,
    ANNOUNCE_DONE, ANNOUNCE_FAILED,
    START, ARRIVE_STATION, STATION_STAY_DONE, ALIGN_DONE,
    CAMP_SEEN, CAMP_STOPPED,
    RED_CONE_HIT, BLUE_CONE_HIT, FENCE_HIT,
    TIMEOUT, SENSOR_FAILURE, SAFETY_STOP,
    VISION_DEGRADED, IMU_DEGRADED, OPERATOR_ABORT
};

struct EventRecord {
    EventType type;
    int64_t ts_ms; // elapsed mission time, measured from START; pre-start events use 0
    int penalty_s;
};

class EventLog {
public:
    void reset();
    void record(EventType t, int64_t elapsed_ms, int penalty_s = 0);
    bool failed() const;
    int total_penalty_s() const;
    int count() const;
    const std::vector<EventRecord>& records() const;
    static const char* name(EventType t);
private:
    std::vector<EventRecord> recs_;
};

#endif
