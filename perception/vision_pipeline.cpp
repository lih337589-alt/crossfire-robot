#include "vision_pipeline.h"

#include <algorithm>

void VisionPipeline::configure(const VisionPipelineParams& params) {
    p_ = params;
    // Keep the kernel bounded even when the class is used without Config::validate().
    int k = std::clamp(p_.morphology_kernel_size, 1, 31);
    if ((k % 2) == 0) ++k;
    p_.morphology_iterations = std::clamp(p_.morphology_iterations, 0, 5);
    p_.dark_threshold_v = std::clamp(p_.dark_threshold_v, 0, 255);
    kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    configured_ = true;
}

bool VisionPipeline::process(const cv::Mat& bgr, bool clean_masks) {
    if (!configured_ || bgr.empty() || bgr.channels() != 3 || bgr.depth() != CV_8U) return false;
    cv::cvtColor(bgr, hsv_, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_, cv::Scalar(p_.colors.blue.h_low, p_.colors.blue.s_low, p_.colors.blue.v_low),
                cv::Scalar(p_.colors.blue.h_high, 255, 255), blue_mask_);
    cv::inRange(hsv_, cv::Scalar(p_.colors.red1.h_low, p_.colors.red1.s_low, p_.colors.red1.v_low),
                cv::Scalar(p_.colors.red1.h_high, 255, 255), red1_mask_);
    cv::inRange(hsv_, cv::Scalar(p_.colors.red2.h_low, p_.colors.red2.s_low, p_.colors.red2.v_low),
                cv::Scalar(p_.colors.red2.h_high, 255, 255), red2_mask_);
    cv::bitwise_or(red1_mask_, red2_mask_, red_mask_);
    cv::inRange(hsv_, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, p_.dark_threshold_v), dark_mask_);

    if (clean_masks && p_.morphology_iterations > 0) {
        cv::morphologyEx(blue_mask_, blue_mask_, cv::MORPH_OPEN, kernel_, cv::Point(-1, -1), p_.morphology_iterations);
        cv::morphologyEx(blue_mask_, blue_mask_, cv::MORPH_CLOSE, kernel_, cv::Point(-1, -1), p_.morphology_iterations);
        cv::morphologyEx(red_mask_, red_mask_, cv::MORPH_OPEN, kernel_, cv::Point(-1, -1), p_.morphology_iterations);
        cv::morphologyEx(red_mask_, red_mask_, cv::MORPH_CLOSE, kernel_, cv::Point(-1, -1), p_.morphology_iterations);
    }
    const cv::Scalar m = cv::mean(bgr);
    brightness_ = (static_cast<float>(m[0]) + static_cast<float>(m[1]) + static_cast<float>(m[2])) / 3.0f;
    return true;
}

float VisionPipeline::blue_ratio() const {
    const int total = blue_mask_.rows * blue_mask_.cols;
    return total > 0 ? static_cast<float>(cv::countNonZero(blue_mask_)) / total : 0.0f;
}
float VisionPipeline::red_ratio() const {
    // red_mask_ 已是 red1|red2 的按位或：直接数一次，省去第二次全图扫描。
    // 即使两段色相重叠结果也正确（不再依赖“区间不重叠”前提）。
    const int total = red_mask_.rows * red_mask_.cols;
    if (total <= 0) return 0.0f;
    return static_cast<float>(cv::countNonZero(red_mask_)) / total;
}

bool VisionPipeline::fence_contact_proxy(const cv::Rect& roi, int edge_strip_px,
                                         int bottom_band_px, float threshold, float opposite_edge_max_ratio) const {
    if (dark_mask_.empty() || roi.width <= 0 || roi.height <= 0) return false;
    const cv::Rect bounds(0, 0, dark_mask_.cols, dark_mask_.rows);
    const cv::Rect safe = roi & bounds;
    if (safe.width < 2 || safe.height < 2) return false;
    const int strip = std::clamp(edge_strip_px, 1, safe.width / 2);
    const int band = std::clamp(bottom_band_px, 1, safe.height);
    const cv::Rect left(safe.x, safe.y + safe.height - band, strip, band);
    const cv::Rect right(safe.x + safe.width - strip, safe.y + safe.height - band, strip, band);
    const double denom = static_cast<double>(strip) * band;
    const double l = cv::countNonZero(dark_mask_(left)) / denom;
    const double r = cv::countNonZero(dark_mask_(right)) / denom;
    // Asymmetry helps avoid treating a uniformly dark floor as a side fence.
    return (l >= threshold && r < opposite_edge_max_ratio) ||
           (r >= threshold && l < opposite_edge_max_ratio);
}
