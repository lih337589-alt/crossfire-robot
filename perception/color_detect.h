#ifndef CROSSFIRE_COLOR_DETECT_H
#define CROSSFIRE_COLOR_DETECT_H

/*
 * 色卡识别 —— 计算画面中蓝/红像素占比（对应原 computeBlueRatio/computeRedRatio）。
 * 依赖 OpenCV（cvtColor/inRange/countNonZero）。
 */

#include <opencv2/opencv.hpp>
#include "../config/config.h"   /* 复用 ColorThreshold */

struct ColorRatioParams {
    ColorThreshold blue;
    ColorThreshold red1;   /* 红色 H 范围 1（低段） */
    ColorThreshold red2;   /* 红色 H 范围 2（高段） */
};

/* 蓝色像素占比 [0,1] */
float blue_ratio(const cv::Mat& frame, const ColorRatioParams& p);

/* 红色像素占比 [0,1]（两个 H 段 OR，两段不重叠故直接相加） */
float red_ratio(const cv::Mat& frame, const ColorRatioParams& p);

#endif /* CROSSFIRE_COLOR_DETECT_H */
