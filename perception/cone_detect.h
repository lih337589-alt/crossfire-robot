#ifndef CROSSFIRE_CONE_DETECT_H
#define CROSSFIRE_CONE_DETECT_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "cone.h"

struct ConeDetectParams {
    int min_cont_w = 7, min_cont_h = 6;
    float aspect_min = 0.2f, aspect_max = 1.8f;
    float fill_min = 0.08f, fill_max = 0.82f;
    float stop_area_ratio = 0.24f;
    bool suppress_large_blob = true; // blue camp mask suppression; red safety detector can disable this
    int roi_x1 = 20, roi_y1 = 180;
    int roi_w = 600, roi_h = 260;
    int top_exclude_global_y = 230;
    int edge_margin_px = 1, shape_min_height = 28;
    int bottom_band_min_px = 3, bottom_band_max_px = 14, bottom_band_height_divisor = 5;
    int bottom_min_width = 7, mid_min_width = 5;
    float bottom_width_ratio = 1.05f, mid_width_ratio = 0.60f;
};

// Reuse contour/hierarchy/output capacities across frames. Returned reference remains valid
// until the next detect() call on this same detector instance.
class ConeDetector {
public:
    explicit ConeDetector(const ConeDetectParams& params = {}) : p_(params) {}
    void set_params(const ConeDetectParams& params) { p_ = params; }
    void set_output_offset(int x, int y) { output_offset_x_ = x; output_offset_y_ = y; }
    const std::vector<Cone>& detect(const cv::Mat& mask, const char* color, float min_area);
private:
    int section_width(const cv::Mat& roi, int y_start, int y_end) const;
    ConeDetectParams p_;
    int output_offset_x_ = 0, output_offset_y_ = 0;
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;
    std::vector<double> contour_areas_; // cached once per contour; capacity reused across frames
    std::vector<Cone> cones_;
};

// Compatibility helper; real-time code should keep a ConeDetector instance and reuse it.
std::vector<Cone> find_cones(const cv::Mat& mask, const std::string& color,
                             float min_area, const ConeDetectParams& p);

#endif
