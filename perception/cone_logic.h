#ifndef CROSSFIRE_CONE_LOGIC_H
#define CROSSFIRE_CONE_LOGIC_H

/*
 * 锥桶决策逻辑（纯逻辑，不依赖 OpenCV，可独立单元测试）。
 * 对应原 demo.cpp 的三个核心决策函数：
 *   group_rows        ← group_blue_rows
 *   red_blockers      ← red_blocks_corridor
 *   choose_gap_center ← choose_gap_center
 */

#include "cone.h"
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string>

/* 按 bottom_y 把锥桶归成"排"（同一排 y 接近，容差 tolerance）；排按远近(mean_y)降序返回 */
std::vector<std::vector<Cone>> group_rows(const std::vector<Cone>& cones, int tolerance);


/* Reusable row grouper for the per-frame control path. It stores pointers into the input cone
 * vector and reuses bucket capacity; input must remain unchanged until the next group() call. */
class ConeRowGrouper {
public:
    size_t group(const std::vector<Cone>& cones, int tolerance);
    size_t row_count() const { return active_rows_; }
    const std::vector<const Cone*>& row(size_t index) const;
private:
    struct RowBucket {
        std::vector<const Cone*> cones;
        int64_t sum_y = 0;
        double mean_y() const {
            return cones.empty() ? 0.0 : static_cast<double>(sum_y) / static_cast<double>(cones.size());
        }
    };
    std::vector<const Cone*> sorted_;
    std::vector<RowBucket> buckets_;
    size_t active_rows_ = 0;
};

/* 判定间隙 [left_x, right_x]（行 ry）内是否有红锥阻挡；below_tol = 比选中排更靠后的容忍像素 */
bool corridor_has_red_blocker(const std::vector<Cone>& red_cones,
                              int left_x, int right_x, int row_y,
                              int margin_x, int below_tol);
std::vector<Cone> red_blockers(const std::vector<Cone>& red_cones,
                               int left_x, int right_x, int row_y,
                               int margin_x, int below_tol);

/* Fail-safe response when crossing-lane cones disappear. */
enum class ConeLossAction { LOW_SPEED_SEARCH, STOP_AND_WAIT };
ConeLossAction decide_cone_loss_action(int consecutive_missing_frames, int stop_after_frames);

class RedConeTracker {
public:
    std::vector<Cone> update(const std::vector<Cone>& detected, int confirm_missing_frames,
                             int match_radius_px, float confidence_threshold);
    void reset();
private:
    struct Track { Cone cone; int missing_frames = 0; };
    std::vector<Track> tracks_;
};
bool corridor_near_uncertain_edge(int left_x, int right_x, int roi_left_x,
                                  int roi_right_x, int uncertain_edge_px);

/* 选缝策略：left=最左 / right=最右 / 其他=离画面中心最近；返回选中缝中心，无缝返回 -1 */
int choose_gap_center(const std::vector<Gap>& gaps, int image_center_x, const std::string& policy);

#endif /* CROSSFIRE_CONE_LOGIC_H */
