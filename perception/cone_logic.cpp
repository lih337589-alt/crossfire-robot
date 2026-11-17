#include "cone_logic.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>

size_t ConeRowGrouper::group(const std::vector<Cone>& cones, int tolerance) {
    // Reuse all high-water capacities. Only pointers are sorted/stored on the hot path; Cone
    // objects (including their std::string member) are not copied during grouping.
    for (size_t i = 0; i < active_rows_; ++i) {
        buckets_[i].cones.clear();
        buckets_[i].sum_y = 0;
    }
    active_rows_ = 0;
    sorted_.clear();
    if (sorted_.capacity() < cones.size()) sorted_.reserve(cones.size());
    for (const auto& cone : cones) sorted_.push_back(&cone);
    std::sort(sorted_.begin(), sorted_.end(), [](const Cone* a, const Cone* b) {
        if (a->bottom_y != b->bottom_y) return a->bottom_y > b->bottom_y;
        return a->bottom_x < b->bottom_x;
    });

    tolerance = std::max(1, tolerance);
    for (const Cone* cone_ptr : sorted_) {
        const Cone& cone = *cone_ptr;
        size_t best = active_rows_;
        double best_dist = static_cast<double>(tolerance) + 1.0;
        for (size_t i = 0; i < active_rows_; ++i) {
            const double dist = std::abs(static_cast<double>(cone.bottom_y) - buckets_[i].mean_y());
            if (dist <= tolerance && dist < best_dist) {
                best = i;
                best_dist = dist;
            }
        }
        if (best == active_rows_) {
            if (active_rows_ == buckets_.size()) buckets_.emplace_back();
            RowBucket& bucket = buckets_[active_rows_++];
            bucket.cones.clear();
            bucket.sum_y = 0;
            bucket.cones.push_back(cone_ptr);
            bucket.sum_y = cone.bottom_y;
        } else {
            RowBucket& bucket = buckets_[best];
            bucket.cones.push_back(cone_ptr);
            bucket.sum_y += cone.bottom_y;
        }
    }

    std::sort(buckets_.begin(), buckets_.begin() + static_cast<std::ptrdiff_t>(active_rows_),
              [](const RowBucket& a, const RowBucket& b) { return a.mean_y() > b.mean_y(); });
    for (size_t i = 0; i < active_rows_; ++i) {
        auto& row_cones = buckets_[i].cones;
        std::sort(row_cones.begin(), row_cones.end(),
                  [](const Cone* a, const Cone* b) { return a->bottom_x < b->bottom_x; });
    }
    return active_rows_;
}

const std::vector<const Cone*>& ConeRowGrouper::row(size_t index) const {
    static const std::vector<const Cone*> empty_row;
    return index < active_rows_ ? buckets_[index].cones : empty_row;
}

std::vector<std::vector<Cone>> group_rows(const std::vector<Cone>& cones, int tolerance) {
    ConeRowGrouper grouper;
    grouper.group(cones, tolerance);
    std::vector<std::vector<Cone>> rows;
    rows.reserve(grouper.row_count());
    for (size_t i = 0; i < grouper.row_count(); ++i) {
        const auto& source = grouper.row(i);
        std::vector<Cone> row;
        row.reserve(source.size());
        for (const Cone* cone : source) row.push_back(*cone);
        rows.push_back(std::move(row));
    }
    return rows;
}

bool corridor_has_red_blocker(const std::vector<Cone>& red_cones,
                              int left_x, int right_x, int row_y,
                              int margin_x, int below_tol) {
    const int64_t margin = std::max<int64_t>(0, margin_x);
    const int64_t x_lo = static_cast<int64_t>(left_x) - margin;
    const int64_t x_hi = static_cast<int64_t>(right_x) + margin;
    const int64_t y_limit = static_cast<int64_t>(row_y) + std::max<int64_t>(0, below_tol);
    for (const auto& red : red_cones) {
        if (static_cast<int64_t>(red.bottom_x) >= x_lo &&
            static_cast<int64_t>(red.bottom_x) <= x_hi &&
            static_cast<int64_t>(red.bottom_y) <= y_limit) return true;
    }
    return false;
}

std::vector<Cone> red_blockers(const std::vector<Cone>& red_cones,
                               int left_x, int right_x, int row_y,
                               int margin_x, int below_tol) {
    std::vector<Cone> blockers;
    if (red_cones.empty()) return blockers;
    const int64_t margin = std::max<int64_t>(0, margin_x);
    const int64_t x_lo = static_cast<int64_t>(left_x) - margin;
    const int64_t x_hi = static_cast<int64_t>(right_x) + margin;
    const int64_t y_limit = static_cast<int64_t>(row_y) + std::max<int64_t>(0, below_tol);
    blockers.reserve(red_cones.size());
    for (const auto& red : red_cones) {
        if (static_cast<int64_t>(red.bottom_x) >= x_lo &&
            static_cast<int64_t>(red.bottom_x) <= x_hi &&
            static_cast<int64_t>(red.bottom_y) <= y_limit) blockers.push_back(red);
    }
    return blockers;
}

int choose_gap_center(const std::vector<Gap>& gaps, int image_center_x, const std::string& policy) {
    if (gaps.empty()) return -1;
    const Gap* best = &gaps.front();
    if (policy == "left") {
        for (const auto& g : gaps) if (g.center_x < best->center_x) best = &g;
    } else if (policy == "right") {
        for (const auto& g : gaps) if (g.center_x > best->center_x) best = &g;
    } else {
        const auto distance = [](int a, int b) -> int64_t {
            const int64_t d = static_cast<int64_t>(a) - static_cast<int64_t>(b);
            return d < 0 ? -d : d;
        };
        int64_t best_dist = distance(best->center_x, image_center_x);
        for (const auto& g : gaps) {
            const int64_t d = distance(g.center_x, image_center_x);
            if (d < best_dist || (d == best_dist && g.width > best->width)) {
                best = &g;
                best_dist = d;
            }
        }
    }
    return best->center_x;
}

ConeLossAction decide_cone_loss_action(int consecutive_missing_frames, int stop_after_frames) {
    // Invalid thresholds fail closed. The threshold frame itself requires a stop.
    if (stop_after_frames < 1 || consecutive_missing_frames >= stop_after_frames)
        return ConeLossAction::STOP_AND_WAIT;
    return ConeLossAction::LOW_SPEED_SEARCH;
}


void RedConeTracker::reset() { tracks_.clear(); }

std::vector<Cone> RedConeTracker::update(const std::vector<Cone>& detected,
                                         int confirm_missing_frames,
                                         int match_radius_px,
                                         float confidence_threshold) {
    if (confirm_missing_frames < 1 || match_radius_px < 0 ||
        !std::isfinite(confidence_threshold) || confidence_threshold < 0.0f || confidence_threshold > 1.0f) {
        // Invalid safety configuration must not erase any previously observed obstacle.
        std::vector<Cone> fail_closed;
        fail_closed.reserve(tracks_.size() + detected.size());
        for (const auto& track : tracks_) fail_closed.push_back(track.cone);
        fail_closed.insert(fail_closed.end(), detected.begin(), detected.end());
        return fail_closed;
    }
    std::vector<bool> used(detected.size(), false);
    for (auto& track : tracks_) {
        size_t best = detected.size();
        int64_t best_dist2 = static_cast<int64_t>(match_radius_px) * match_radius_px + 1;
        for (size_t i = 0; i < detected.size(); ++i) {
            if (used[i] || detected[i].confidence < confidence_threshold) continue;
            const int64_t dx = static_cast<int64_t>(track.cone.bottom_x) - detected[i].bottom_x;
            const int64_t dy = static_cast<int64_t>(track.cone.bottom_y) - detected[i].bottom_y;
            // Bound each delta before squaring to avoid signed overflow on malformed coordinates.
            if (dx < -match_radius_px || dx > match_radius_px ||
                dy < -match_radius_px || dy > match_radius_px) continue;
            const int64_t dist2 = dx * dx + dy * dy;
            if (dist2 <= static_cast<int64_t>(match_radius_px) * match_radius_px && dist2 < best_dist2) {
                best = i; best_dist2 = dist2;
            }
        }
        if (best != detected.size()) {
            track.cone = detected[best]; track.missing_frames = 0; used[best] = true;
        } else {
            ++track.missing_frames;
        }
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& t) {
        return t.missing_frames >= confirm_missing_frames;
    }), tracks_.end());
    for (size_t i = 0; i < detected.size(); ++i) {
        if (used[i]) continue;
        // Low-confidence detections are still obstacles, but cannot refresh a trusted track.
        tracks_.push_back(Track{detected[i], 0});
    }
    std::vector<Cone> result;
    result.reserve(tracks_.size());
    for (const auto& track : tracks_) result.push_back(track.cone);
    return result;
}

bool corridor_near_uncertain_edge(int left_x, int right_x, int roi_left_x,
                                  int roi_right_x, int uncertain_edge_px) {
    if (uncertain_edge_px < 0 || roi_right_x <= roi_left_x || right_x < left_x) return true;
    const int64_t margin = uncertain_edge_px;
    return static_cast<int64_t>(left_x) <= static_cast<int64_t>(roi_left_x) + margin ||
           static_cast<int64_t>(right_x) >= static_cast<int64_t>(roi_right_x) - margin;
}
