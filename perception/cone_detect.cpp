#include "cone_detect.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

int ConeDetector::section_width(const cv::Mat& roi, int y_start, int y_end) const {
    const int y1 = std::max(0, y_start);
    const int y2 = std::min(roi.rows, y_end);
    if (y1 >= y2) return 0;
    int min_x = roi.cols, max_x = -1;
    for (int y = y1; y < y2; ++y) {
        const unsigned char* row = roi.ptr<unsigned char>(y);
        for (int x = 0; x < roi.cols; ++x) {
            if (row[x] != 0) {
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
            }
        }
    }
    return max_x < min_x ? 0 : max_x - min_x + 1;
}

const std::vector<Cone>& ConeDetector::detect(const cv::Mat& mask, const char* color, float min_area) {
    cones_.clear();
    // findContours requires an 8-bit, single-channel image. Reject invalid input instead of
    // allowing OpenCV to throw from the real-time control loop.
    if (mask.empty() || mask.channels() != 1 || mask.depth() != CV_8U ||
        !std::isfinite(min_area) || min_area < 0.0f) {
        return cones_;
    }

    cv::findContours(mask, contours_, hierarchy_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // contourArea() walks contour points. Compute it once and reuse the value in the filtering
    // pass; the vector retains capacity between frames.
    contour_areas_.clear();
    if (contour_areas_.capacity() < contours_.size()) contour_areas_.reserve(contours_.size());
    double max_blob = 0.0;
    for (const auto& contour : contours_) {
        const double area = contour.empty() ? 0.0 : cv::contourArea(contour);
        contour_areas_.push_back(area);
        max_blob = std::max(max_blob, area);
    }

    // The supplied mask is already the ROI. Use its actual dimensions rather than duplicated
    // configuration metadata so a caller cannot accidentally use a stale ROI size.
    const double roi_area = static_cast<double>(mask.cols) * static_cast<double>(mask.rows);
    if (p_.suppress_large_blob && roi_area > 0.0 &&
        max_blob >= roi_area * std::max(0.0f, p_.stop_area_ratio)) {
        return cones_;
    }

    if (cones_.capacity() < contours_.size()) cones_.reserve(contours_.size());
    for (size_t i = 0; i < contours_.size(); ++i) {
        const auto& contour = contours_[i];
        const double contour_area = contour_areas_[i];
        const float area = static_cast<float>(contour_area); // preserve the original float filter semantics
        if (contour.empty() || area < min_area) continue;

        const cv::Rect bbox = cv::boundingRect(contour);
        const int w = bbox.width;
        const int h = bbox.height;
        if (w < p_.min_cont_w || h < p_.min_cont_h) continue;
        if (static_cast<int64_t>(bbox.y) + output_offset_y_ < p_.top_exclude_global_y) continue;
        if (bbox.x <= p_.edge_margin_px ||
            static_cast<int64_t>(bbox.x) + w >= static_cast<int64_t>(mask.cols) - p_.edge_margin_px) continue;

        const float aspect = static_cast<float>(w) / static_cast<float>(h);
        if (aspect < p_.aspect_min || aspect > p_.aspect_max) continue;
        const double bbox_area = static_cast<double>(w) * static_cast<double>(h);
        if (bbox_area <= 0.0) continue;
        const float fill = area / static_cast<float>(bbox_area);
        if (fill < p_.fill_min || fill > p_.fill_max) continue;
        const float aspect_mid = (p_.aspect_min + p_.aspect_max) * 0.5f;
        const float aspect_half = std::max(0.001f, (p_.aspect_max - p_.aspect_min) * 0.5f);
        const float fill_mid = (p_.fill_min + p_.fill_max) * 0.5f;
        const float fill_half = std::max(0.001f, (p_.fill_max - p_.fill_min) * 0.5f);
        const float aspect_score = std::max(0.0f, 1.0f - std::abs(aspect - aspect_mid) / aspect_half);
        const float fill_score = std::max(0.0f, 1.0f - std::abs(fill - fill_mid) / fill_half);
        const float shape_confidence = std::min(aspect_score, fill_score);

        if (h >= p_.shape_min_height) {
            // ROI is a Mat header/view; no pixel copy is made here.
            const cv::Mat roi = mask(bbox);
            const int top_w = section_width(roi, 0, h / 3);
            const int mid_w = section_width(roi, h / 3, (2 * h) / 3);
            const int bot_w = section_width(roi, (2 * h) / 3, h);
            if (bot_w < std::max(p_.bottom_min_width,
                                 static_cast<int>(top_w * p_.bottom_width_ratio))) continue;
            if (mid_w < std::max(p_.mid_min_width,
                                 static_cast<int>(top_w * p_.mid_width_ratio))) continue;
        }

        const cv::Moments m = cv::moments(contour);
        const int cx = m.m00 == 0.0 ? bbox.x + bbox.width / 2
                                    : static_cast<int>(m.m10 / m.m00);
        const int cy = m.m00 == 0.0 ? bbox.y + bbox.height / 2
                                    : static_cast<int>(m.m01 / m.m00);
        // boundingRect is inclusive for integer contour points: bottom-most point is y+h-1.
        // Reuse the already-computed box instead of scanning all contour points again.
        const int max_y = bbox.y + bbox.height - 1;
        if (max_y < 0) continue;

        const int divisor = std::max(1, p_.bottom_band_height_divisor);
        const int band = std::max(p_.bottom_band_min_px,
                                  std::min(p_.bottom_band_max_px, h / divisor));
        int64_t x_sum = 0;
        int64_t x_count = 0;
        for (const auto& pt : contour) {
            if (pt.y >= max_y - band) {
                x_sum += static_cast<int64_t>(pt.x);
                ++x_count;
            }
        }
        const int bottom_x = x_count == 0 ? bbox.x + bbox.width / 2
            : static_cast<int>(std::lround(static_cast<double>(x_sum) /
                                           static_cast<double>(x_count)));

        Cone c;
        c.color = color ? color : "unknown";
        c.bbox_x = bbox.x + output_offset_x_;
        c.bbox_y = bbox.y + output_offset_y_;
        c.bbox_w = w;
        c.bbox_h = h;
        c.center_x = cx + output_offset_x_;
        c.center_y = cy + output_offset_y_;
        c.bottom_x = bottom_x + output_offset_x_;
        c.bottom_y = max_y + output_offset_y_;
        c.area = area;
        c.confidence = shape_confidence;
        cones_.push_back(std::move(c));
    }
    std::sort(cones_.begin(), cones_.end(),
              [](const Cone& a, const Cone& b) { return a.bottom_y > b.bottom_y; });
    return cones_;
}

std::vector<Cone> find_cones(const cv::Mat& mask, const std::string& color,
                             float min_area, const ConeDetectParams& p) {
    ConeDetector detector(p);
    const auto& result = detector.detect(mask, color.c_str(), min_area);
    return result;
}
