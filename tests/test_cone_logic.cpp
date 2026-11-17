/*
 * 锥桶决策逻辑单元测试（纯逻辑，不依赖 OpenCV）
 *
 * 编译：g++ -std=c++17 -Wall -Wextra -Iperception \
 *          tests/test_cone_logic.cpp perception/cone_logic.cpp -o test_cone_logic
 */
#include "cone_logic.h"

#include <cstdio>
#include <climits>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::fprintf(stderr, "FAIL: %s\n", msg); } \
} while (0)

static Cone mk(int bottom_y, int bottom_x) {
    Cone c;
    c.bottom_y = bottom_y;
    c.bottom_x = bottom_x;
    return c;
}

static void test_cone_loss_fail_safe() {
    CHECK(decide_cone_loss_action(1, 4) == ConeLossAction::LOW_SPEED_SEARCH,
          "锥桶刚丢失时进入低速搜索而非接近速度直行");
    CHECK(decide_cone_loss_action(3, 4) == ConeLossAction::LOW_SPEED_SEARCH,
          "确认阈值之前仍仅允许低速搜索");
    CHECK(decide_cone_loss_action(4, 4) == ConeLossAction::STOP_AND_WAIT,
          "达到确认帧数阈值后停车等待");
    CHECK(decide_cone_loss_action(5, 4) == ConeLossAction::STOP_AND_WAIT,
          "持续丢失时保持停车");
    CHECK(decide_cone_loss_action(0, 0) == ConeLossAction::STOP_AND_WAIT,
          "非法阈值采用失效安全停车");
}

static void test_red_tracker_and_uncertainty() {
    RedConeTracker tracker;
    Cone red = mk(180, 200); red.color = "red"; red.confidence = 0.9f;
    auto visible = tracker.update({red}, 3, 45, 0.25f);
    CHECK(visible.size() == 1, "红色地雷首次检测进入障碍集合");
    auto single_miss = tracker.update({}, 3, 45, 0.25f);
    CHECK(single_miss.size() == 1, "单帧漏检不移除已确认红色地雷");
    auto returned = tracker.update({red}, 3, 45, 0.25f);
    CHECK(returned.size() == 1, "单帧漏检后重新检测保持同一障碍");
    tracker.update({}, 3, 45, 0.25f);
    tracker.update({}, 3, 45, 0.25f);
    auto expired = tracker.update({}, 3, 45, 0.25f);
    CHECK(expired.empty(), "连续达到确认帧数后才允许移除历史障碍");

    Cone adjacent_blue = mk(180, 220); adjacent_blue.color = "blue";
    auto adjacent = tracker.update({red, adjacent_blue}, 3, 45, 0.25f);
    CHECK(std::any_of(adjacent.begin(), adjacent.end(), [](const Cone& c) { return c.color == "red"; }),
          "相邻蓝色锥桶不能覆盖红色地雷障碍");
    CHECK(corridor_near_uncertain_edge(25, 100, 20, 620, 12),
          "接近 ROI 边界的截断走廊按不确定处理");
    CHECK(!corridor_near_uncertain_edge(100, 300, 20, 620, 12),
          "远离 ROI 边界的完整走廊不因边界规则直接阻挡");
    Cone low = mk(180, 205); low.color = "red"; low.confidence = 0.1f;
    auto low_conf = tracker.update({low}, 3, 45, 0.25f);
    CHECK(std::any_of(low_conf.begin(), low_conf.end(), [](const Cone& c) { return c.confidence < 0.25f; }),
          "部分遮挡产生的低置信红色候选仍保留为保守障碍");
}

static void test_group_rows() {
    std::vector<Cone> cones = { mk(300, 100), mk(100, 50), mk(105, 60) };
    auto rows = group_rows(cones, 35);

    CHECK(rows.size() == 2, "分成 2 排");
    CHECK(rows[0].size() == 1 && rows[0][0].bottom_y == 300, "近排在前(1个)");
    CHECK(rows[1].size() == 2, "远排 2 个");
    /* 远排按 bottom_x 升序 */
    CHECK(rows[1][0].bottom_x == 50 && rows[1][1].bottom_x == 60, "排内按 x 排序");
}

static void test_red_blockers() {
    std::vector<Cone> reds;
    Cone r1 = mk(150, 200);   /* 间隙内，前方 → 阻挡 */
    Cone r2 = mk(150, 400);   /* 间隙外 → 不阻挡 */
    Cone r3 = mk(300, 200);   /* 后方(below_tol) → 不阻挡 */
    reds.push_back(r1); reds.push_back(r2); reds.push_back(r3);

    auto blockers = red_blockers(reds, 100, 300, 200, 10, 40);
    CHECK(blockers.size() == 1, "只有 1 个阻挡");
    CHECK(blockers[0].bottom_x == 200, "阻挡的是 r1");
    CHECK(corridor_has_red_blocker(reds, 100, 300, 200, 10, 40), "无分配阻挡判定为 true");
    CHECK(!corridor_has_red_blocker(reds, 100, 150, 200, 10, 40), "无分配阻挡判定为 false");
}


static void test_reusable_grouper_and_extremes() {
    ConeRowGrouper grouper;
    std::vector<Cone> cones = {mk(300, 100), mk(100, 50), mk(105, 60)};
    CHECK(grouper.group(cones, 35) == 2, "复用分行器输出 2 排");
    CHECK(grouper.row(0).size() == 1 && grouper.row(0)[0]->bottom_y == 300,
          "复用分行器近排在前");
    CHECK(grouper.row(1).size() == 2 && grouper.row(1)[0]->bottom_x == 50,
          "复用分行器排内按 x 排序");

    std::vector<Cone> one = {mk(10, 20)};
    CHECK(grouper.group(one, 35) == 1, "复用分行器支持行数减少");
    CHECK(grouper.row(0).size() == 1 && grouper.row(1).empty(),
          "复用分行器不暴露上一帧的过期行");
    std::vector<Cone> none;
    CHECK(grouper.group(none, 35) == 0 && grouper.row(0).empty(),
          "复用分行器支持空输入");

    std::vector<Cone> edge = {mk(INT_MAX, INT_MAX)};
    CHECK(corridor_has_red_blocker(edge, INT_MAX - 2, INT_MAX, INT_MAX, 10, 10),
          "极大坐标下阻挡边界不发生有符号溢出");
    CHECK(red_blockers(edge, INT_MAX - 2, INT_MAX, INT_MAX, 10, 10).size() == 1,
          "极大坐标下阻挡列表边界正确");
    std::vector<Gap> extremes = {{INT_MIN, 10}, {INT_MAX, 20}};
    CHECK(choose_gap_center(extremes, 0, "center") == INT_MAX,
          "极端中心距离使用 64 位运算");
}


static std::vector<std::vector<Cone>> reference_group_rows(const std::vector<Cone>& cones, int tolerance) {
    if (cones.empty()) return {};
    struct Bucket { std::vector<Cone> cones; int64_t sum_y = 0; double mean_y() const {
        return cones.empty() ? 0.0 : static_cast<double>(sum_y) / static_cast<double>(cones.size());
    }};
    std::vector<Cone> sorted = cones;
    std::sort(sorted.begin(), sorted.end(), [](const Cone& a, const Cone& b) {
        if (a.bottom_y != b.bottom_y) return a.bottom_y > b.bottom_y;
        return a.bottom_x < b.bottom_x;
    });
    std::vector<Bucket> buckets;
    tolerance = std::max(1, tolerance);
    for (const auto& cone : sorted) {
        int best = -1;
        double best_dist = static_cast<double>(tolerance) + 1.0;
        for (size_t i = 0; i < buckets.size(); ++i) {
            const double dist = std::abs(static_cast<double>(cone.bottom_y) - buckets[i].mean_y());
            if (dist <= tolerance && dist < best_dist) { best = static_cast<int>(i); best_dist = dist; }
        }
        if (best < 0) {
            Bucket b; b.cones.push_back(cone); b.sum_y = cone.bottom_y; buckets.push_back(std::move(b));
        } else {
            buckets[static_cast<size_t>(best)].cones.push_back(cone);
            buckets[static_cast<size_t>(best)].sum_y += cone.bottom_y;
        }
    }
    std::sort(buckets.begin(), buckets.end(), [](const Bucket& a, const Bucket& b) { return a.mean_y() > b.mean_y(); });
    std::vector<std::vector<Cone>> rows;
    rows.reserve(buckets.size());
    for (auto& b : buckets) {
        std::sort(b.cones.begin(), b.cones.end(), [](const Cone& a, const Cone& b) { return a.bottom_x < b.bottom_x; });
        rows.push_back(std::move(b.cones));
    }
    return rows;
}

static void test_grouping_matches_reference() {
    std::mt19937 rng(0xC0FFEEu);
    std::uniform_int_distribution<int> count_dist(0, 30);
    std::uniform_int_distribution<int> x_dist(0, 1279);
    std::uniform_int_distribution<int> y_dist(0, 719);
    std::uniform_int_distribution<int> tol_dist(-5, 80);
    for (int trial = 0; trial < 100; ++trial) {
        std::vector<Cone> cones;
        const int count = count_dist(rng);
        cones.reserve(static_cast<size_t>(count));
        for (int i = 0; i < count; ++i) cones.push_back(mk(y_dist(rng), x_dist(rng)));
        const int tolerance = tol_dist(rng);
        const auto expected = reference_group_rows(cones, tolerance);
        const auto actual = group_rows(cones, tolerance);
        bool same = actual.size() == expected.size();
        for (size_t r = 0; same && r < expected.size(); ++r) {
            same = actual[r].size() == expected[r].size();
            for (size_t c = 0; same && c < expected[r].size(); ++c) {
                same = actual[r][c].bottom_x == expected[r][c].bottom_x &&
                       actual[r][c].bottom_y == expected[r][c].bottom_y;
            }
        }
        CHECK(same, "复用分行器与原分组语义一致（确定性随机样本）");
    }
}

static void test_choose_gap() {
    std::vector<Gap> gaps = { {100, 50}, {300, 60}, {250, 40} };

    CHECK(choose_gap_center(gaps, 200, "center") == 250, "center 策略选最近中心");
    CHECK(choose_gap_center(gaps, 200, "left") == 100, "left 策略选最左");
    CHECK(choose_gap_center(gaps, 200, "right") == 300, "right 策略选最右");

    std::vector<Gap> empty;
    CHECK(choose_gap_center(empty, 200, "center") == -1, "空 gaps 返回 -1");
}

int main() {
    test_cone_loss_fail_safe();
    test_red_tracker_and_uncertainty();
    test_group_rows();
    test_red_blockers();
    test_choose_gap();
    test_reusable_grouper_and_extremes();
    test_grouping_matches_reference();

    std::printf("锥桶逻辑测试结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
