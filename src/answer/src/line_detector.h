//
// Created by sfx233 on 25-3-23.
//

#ifndef LINE_DETECTOR_H
#define LINE_DETECTOR_H
#include"image.h"
class LineDetector {
public:
    LineDetector() : window_size(5) {}
    // 检测直线
    cv::Vec4i detectLines(cv::Mat black_mask) {
        std::vector<cv::Vec4i> lines;

        // 使用 HoughLinesP 检测直线
        cv::HoughLinesP(black_mask, lines, 1, CV_PI / 180, 50, 200, 80);
        // 如果检测到直线，更新滑动窗口
        if (!lines.empty()) {
            updateWindow(lines[0]); // 只处理第一条直线
        }
        // 返回平滑后的直线数据
        return smoothLines();
    }
private:
    int window_size; // 滑动窗口大小
    std::deque<cv::Vec4i> line_window; // 滑动窗口，存储最近的直线数据
    // 更新滑动窗口
    void updateWindow(const cv::Vec4i& line) {
        if (line_window.size() >= window_size) {
            line_window.pop_front(); // 移除最旧的数据
        }
        line_window.push_back(line); // 添加最新的数据
    }
    // 对直线数据进行平滑处理
    cv::Vec4i smoothLines() {


        cv::Vec4i smoothed_line(0, 0, 0, 0);
        if (line_window.empty()) {
            return smoothed_line; // 如果没有数据，返回空
        }
        for (const auto& line : line_window) { // 计算滑动窗口中直线数据的平均值
            smoothed_line += line;
        }
        smoothed_line /= static_cast<int>(line_window.size());
        return smoothed_line;
    }
};//直线检测（带有平滑，缓存操作）
#endif //LINE_DETECTOR_H