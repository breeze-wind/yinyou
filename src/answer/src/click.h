//
// Created by sfx233 on 25-3-23.
//

#ifndef CLICK_H
#define CLICK_H
#include "image.h"
class click {
public:
    click() : min_distance_squared(16.0), min_time_interval(0.3) {}

    bool operator()(cv::Point2f p) {
        try {
            // 计算当前 p 和 last_p 的欧氏距离平方
            double dx = p.x - last_p.x;
            double dy = p.y - last_p.y;
            double distance_squared = dx * dx + dy * dy;

            // 获取当前时间
            auto now = std::chrono::steady_clock::now();

            // 计算时间差
            double time_diff = std::chrono::duration<double>(now - last_publish_time).count();

            // 双重检测：距离和时间间隔
            if (distance_squared < min_distance_squared || time_diff < min_time_interval) {
                return false; // 如果任一条件不满足，不发送消息
            }

            // 更新 last_p 和 last_publish_time
            last_p = p;
            last_publish_time = now;

            // 发布消息
            geometry_msgs::msg::Point32 point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0;
            publisher->publish(point);

            return true;
        } catch (...) {
            return false;
        }
    }

    void Publisher(rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr pub) {
        publisher = pub;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr publisher;
    double min_distance_squared; // 最小距离的平方（避免开方计算）
    double min_time_interval;    // 最小时间间隔（秒）
    cv::Point2f last_p;          // 上一次的 p 值
    std::chrono::steady_clock::time_point last_publish_time; // 上一次发送消息的时间
};

#endif //CLICK_H