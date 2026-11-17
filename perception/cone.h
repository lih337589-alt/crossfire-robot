#ifndef CROSSFIRE_CONE_H
#define CROSSFIRE_CONE_H

/*
 * 锥桶数据模型（从 OpenCV 类型解耦，便于独立测试核心决策逻辑）。
 * 与原 demo.cpp 的 Cone 对应：bbox/center/bottom 用 int 坐标表示。
 */

#include <string>

struct Cone {
    std::string color;         /* "blue" / "red" */
    int   bbox_x = 0, bbox_y = 0, bbox_w = 0, bbox_h = 0;
    int   center_x = 0, center_y = 0;
    int   bottom_x = 0, bottom_y = 0;   /* 底部着地点（量距离/找缝用） */
    float area = 0.0f;
    float confidence = 1.0f; // shape-quality proxy in [0,1], not a probabilistic detector score
};

struct Gap {
    int center_x = 0;   /* 缝中心 */
    int width = 0;      /* 缝宽 */
};

#endif /* CROSSFIRE_CONE_H */
