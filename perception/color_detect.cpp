#include "color_detect.h"

float blue_ratio(const cv::Mat& frame, const ColorRatioParams& p) {
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(p.blue.h_low, p.blue.s_low, p.blue.v_low),
                cv::Scalar(p.blue.h_high, 255, 255), mask);
    int total = frame.rows * frame.cols;
    if (total == 0) return 0.0f;
    return (float)cv::countNonZero(mask) / total;
}

float red_ratio(const cv::Mat& frame, const ColorRatioParams& p) {
    cv::Mat hsv, m1, m2;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(p.red1.h_low, p.red1.s_low, p.red1.v_low),
                cv::Scalar(p.red1.h_high, 255, 255), m1);
    cv::inRange(hsv,
                cv::Scalar(p.red2.h_low, p.red2.s_low, p.red2.v_low),
                cv::Scalar(p.red2.h_high, 255, 255), m2);
    int total = frame.rows * frame.cols;
    if (total == 0) return 0.0f;
    /* 红色两个 H 段（0~10 与 170~180）不重叠，直接相加等价于按位或 */
    int red = cv::countNonZero(m1) + cv::countNonZero(m2);
    return (float)red / total;
}
