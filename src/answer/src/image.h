//
// Created by sfx233 on 25-3-23.
//

#ifndef SHOW_H
#define SHOW_H
#include<string.h>
#include<functional>
#include<algorithm>
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<geometry_msgs/msg/point32.hpp>
#include"cv_bridge/cv_bridge.h"
#include"opencv2/opencv.hpp"
#include<memory>
#include"rect_detector.h"
#include"click.h"
#include"line_detector.h"
double distanceBetweenPoints(const cv::Point2f &p1, const cv::Point2f &p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}//点对点距离
double distanceToLine(const cv::Point &point, const cv::Vec4f &line) {
    double x0 = point.x, y0 = point.y;
    double x1 = line[0], y1 = line[1];
    double x2 = line[2], y2 = line[3];

    double A = y2 - y1;
    double B = x1 - x2;
    double C = x2 * y1 - x1 * y2;

    return std::abs(A * x0 + B * y0 + C) / std::sqrt(A * A + B * B);
}
//点到线距离
#endif //SHOW_H