#include "camp_detect.h"

#include <algorithm>

CampResult CampDetector::detect(const cv::Mat& mask, const CampDetectParams& p) {
    CampResult result;
    if (mask.empty()) return result;
    cv::findContours(mask, contours_, hierarchy_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const double roi_area = static_cast<double>(mask.rows) * mask.cols;
    if (roi_area <= 0.0) return result;
    double best_area = roi_area * std::max(0.0f, p.min_ratio);
    cv::Point best(-1, -1);
    for (const auto& contour : contours_) {
        const double area = cv::contourArea(contour);
        if (area <= best_area) continue;
        const cv::Moments m = cv::moments(contour);
        if (m.m00 == 0.0) continue;
        best = cv::Point(static_cast<int>(m.m10 / m.m00) + p.roi_x1,
                         static_cast<int>(m.m01 / m.m00) + p.roi_y1);
        best_area = area;
    }
    if (best.x >= 0) {
        result.found = true;
        result.center_x = best.x; result.center_y = best.y;
        result.area = static_cast<float>(best_area);
        result.ratio = static_cast<float>(best_area / roi_area);
    }
    return result;
}
CampResult detect_camp(const cv::Mat& mask, const CampDetectParams& p) {
    CampDetector detector;
    return detector.detect(mask, p);
}
