#ifndef CROSSFIRE_STATE_MACHINE_H
#define CROSSFIRE_STATE_MACHINE_H

#include "event_log.h"

// IDLE -> ANNOUNCE -> READY -> TO_STATION -> STATION_STAY -> ALIGN -> CROSSING -> TO_CAMP -> FINISHED
// Any safety-critical failure transitions to FAILED. No terminal state can be re-entered.
enum class State {
    IDLE, ANNOUNCE, READY, TO_STATION, STATION_STAY, ALIGN,
    CROSSING, TO_CAMP, FINISHED, FAILED
};

class StateMachine {
public:
    State state() const { return st_; }
    const EventLog& log() const { return log_; }
    bool on_event(EventType e, int64_t monotonic_ts_ms);
    void reset();
    bool is_terminal() const { return st_ == State::FINISHED || st_ == State::FAILED; }
    void set_min_station_stay_ms(int64_t ms) { min_station_stay_ms_ = ms < 3000 ? 3000 : ms; }
    int64_t start_ts_ms() const { return start_ts_ms_; }
    int64_t elapsed_ms(int64_t now_ms) const;
    static const char* state_name(State s);
private:
    void record(EventType e, int64_t monotonic_ts_ms, int penalty_s = 0);
    void fail(EventType e, int64_t monotonic_ts_ms);
    State st_ = State::IDLE;
    EventLog log_;
    int64_t station_stay_start_ms_ = 0;
    int64_t min_station_stay_ms_ = 3000;
    int64_t start_ts_ms_ = 0;
    bool started_ = false;
};

#endif
