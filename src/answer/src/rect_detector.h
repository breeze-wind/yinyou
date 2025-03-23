//
// Created by sfx233 on 25-3-23.
//
#ifndef RECT_DETECTOR_H
#define RECT_DETECTOR_H
#include"image.h"
class RectDetector {
public:
    RectDetector() : window_size(5) {}

    // 检测矩形
    cv::RotatedRect detectRects(const std::vector<cv::RotatedRect>& rects) {
        if (!rects.empty()) {
            updateWindow(rects[0]); // 只处理第一个矩形
        }
        return smoothRects();
    }

private:
    int window_size; // 滑动窗口大小
    std::deque<cv::RotatedRect> rect_window; // 滑动窗口，存储最近的矩形数据

    // 更新滑动窗口
    void updateWindow(const cv::RotatedRect& rect) {
        if (rect_window.size() >= window_size) {
            rect_window.pop_front(); // 移除最旧的数据
        }
        rect_window.push_back(rect); // 添加最新的数据
    }

    // 对矩形数据进行平滑处理
    cv::RotatedRect smoothRects() {
        cv::RotatedRect smoothed_rect;
        if (rect_window.empty()) {
            return smoothed_rect; // 如果没有数据，返回空
        }

        // 计算滑动窗口中矩形数据的平均值
        cv::Point2f center_sum(0, 0);
        cv::Size2f size_sum(0, 0);
        float angle_sum = 0;

        for (const auto& rect : rect_window) {
            center_sum += rect.center;
            size_sum += rect.size;
            angle_sum += rect.angle;
        }

        smoothed_rect.center = center_sum / static_cast<float>(rect_window.size());
        smoothed_rect.size = size_sum / static_cast<float>(rect_window.size());
        smoothed_rect.angle = angle_sum / static_cast<float>(rect_window.size());

        return smoothed_rect;
    }
};
#endif