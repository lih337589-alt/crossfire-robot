#ifndef CROSSFIRE_TEST_OPENCV_STUB_HPP
#define CROSSFIRE_TEST_OPENCV_STUB_HPP
// Test-only OpenCV declaration shim. It supports HAL/core syntax checks only and is NOT a
// substitute for OpenCV; never link the production binary against this file.
#include <vector>
#include <string>
namespace cv {
struct Point { int x = 0, y = 0; Point() = default; Point(int xx, int yy) : x(xx), y(yy) {} };
struct Size { int width = 0, height = 0; Size() = default; Size(int w, int h) : width(w), height(h) {} };
struct Scalar {
    double v[4] = {0,0,0,0};
    Scalar() = default;
    Scalar(double a, double b = 0, double c = 0, double d = 0) : v{a,b,c,d} {}
    double operator[](int i) const { return v[i]; }
};
struct Rect {
    int x = 0, y = 0, width = 0, height = 0;
    Rect() = default;
    Rect(int xx, int yy, int w, int h) : x(xx), y(yy), width(w), height(h) {}
};
inline Rect operator&(const Rect& a, const Rect& b) {
    const int x1 = a.x > b.x ? a.x : b.x, y1 = a.y > b.y ? a.y : b.y;
    const int x2 = (a.x+a.width) < (b.x+b.width) ? (a.x+a.width) : (b.x+b.width);
    const int y2 = (a.y+a.height) < (b.y+b.height) ? (a.y+a.height) : (b.y+b.height);
    return Rect(x1, y1, x2 > x1 ? x2-x1 : 0, y2 > y1 ? y2-y1 : 0);
}
struct Vec4i { int val[4] = {0,0,0,0}; };
struct Moments { double m00 = 0, m10 = 0, m01 = 0; };
class Mat {
public:
    int rows = 0, cols = 0;
    Mat() = default;
    Mat(int r, int c, int type) : rows(r), cols(c), channels_(type == 16 ? 3 : 1) {}
    Mat(int r, int c, int type, const Scalar&) : rows(r), cols(c), channels_(type == 16 ? 3 : 1) {}
    bool empty() const { return rows <= 0 || cols <= 0; }
    int channels() const { return channels_; }
    int type() const { return channels_ == 3 ? 16 : 0; }
    int depth() const { return 0; }
    void release() { rows = cols = 0; channels_ = 1; }
    template<class T> T* ptr(int) { return nullptr; }
    template<class T> const T* ptr(int) const { return nullptr; }
    Mat operator()(const Rect& r) const { return Mat(r.height, r.width, channels_ == 3 ? 16 : 0); }
private:
    int channels_ = 1;
};
class VideoCapture {
public:
    bool open(int, int) { return true; }
    bool open(const std::string&, int) { return true; }
    bool isOpened() const { return true; }
    bool set(int, double) { return true; }
    bool grab() { return true; }
    bool retrieve(Mat&) { return true; }
    void release() {}
};
constexpr int CAP_V4L2 = 0, CAP_PROP_FRAME_WIDTH = 1, CAP_PROP_FRAME_HEIGHT = 2,
              CAP_PROP_FPS = 3, CAP_PROP_BUFFERSIZE = 4;
constexpr int CV_8U = 0;
constexpr int COLOR_BGR2HSV = 1, MORPH_ELLIPSE = 2, MORPH_OPEN = 3, MORPH_CLOSE = 4,
              RETR_EXTERNAL = 5, CHAIN_APPROX_SIMPLE = 6, INTER_AREA = 7, INTER_LINEAR = 8;
inline void resize(const Mat& src, Mat& dst, const Size& size, double = 0.0, double = 0.0, int = INTER_AREA) { dst = Mat(size.height, size.width, 0); (void)src; }
inline void cvtColor(const Mat&, Mat&, int) {}
inline void inRange(const Mat&, const Scalar&, const Scalar&, Mat&) {}
inline void bitwise_or(const Mat&, const Mat&, Mat&) {}
inline Mat getStructuringElement(int, const Size&) { return Mat(1,1,0); }
inline void morphologyEx(const Mat&, Mat&, int, const Mat&, const Point& = Point(-1,-1), int = 1) {}
inline Scalar mean(const Mat&) { return Scalar(); }
inline int countNonZero(const Mat&) { return 0; }
inline void findContours(const Mat&, std::vector<std::vector<Point>>&, std::vector<Vec4i>&, int, int) {}
inline double contourArea(const std::vector<Point>&) { return 0.0; }
inline Rect boundingRect(const std::vector<Point>&) { return Rect(); }
inline Moments moments(const std::vector<Point>&) { return Moments(); }
inline void fillConvexPoly(Mat&, const std::vector<Point>&, const Scalar&) {}
}
#define CV_8U 0
#define CV_8UC1 0
#define CV_8UC3 16
#endif
