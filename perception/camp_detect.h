#ifndef CROSSFIRE_CAMP_DETECT_H
#define CROSSFIRE_CAMP_DETECT_H

#include <opencv2/opencv.hpp>
#include <vector>

struct CampDetectParams {
    float min_ratio = 0.06f;
    int roi_x1 = 20, roi_y1 = 180;
};
struct CampResult {
    bool found = false;
    int center_x = -1, center_y = -1;
    float area = 0.0f, ratio = 0.0f;
};

class CampDetector {
public:
    CampResult detect(const cv::Mat& mask, const CampDetectParams& params);
private:
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;
};
CampResult detect_camp(const cv::Mat& mask, const CampDetectParams& params);

#endif
