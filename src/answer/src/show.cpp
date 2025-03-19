#include<string.h>
#include<functional>
#include<algorithm>
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<geometry_msgs/msg/point32.hpp>
#include"cv_bridge/cv_bridge.h"
#include"opencv2/opencv.hpp"
#include<memory>
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


class image_precss : public rclcpp::Node {
public:
    image_precss() : rclcpp::Node("image_subscriber") {
        subscription_ = create_subscription<sensor_msgs::msg::Image>("raw_image", 10,
                                                                     std::bind(&image_precss::image_callback, this,
                                                                               std::placeholders::_1));
        msg_pub = this->create_publisher<geometry_msgs::msg::Point32>("click_position", 10);
        my_click.Publisher(msg_pub);
    }

    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr msg_pub;
    click my_click;
    cv::Mat blue_mask;
    cv::Mat black_mask;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    RectDetector rect_detector; // 新增 RectDetector 对象

    bool line_processing(const cv::Mat& black_mask, cv::Vec4i& processed_line) {
        LineDetector line_detector;
        processed_line = line_detector.detectLines(black_mask);
        return true;
    }

    void image_init(const sensor_msgs::msg::Image::SharedPtr msg, cv::Mat &frame, cv::Mat &blue_mask, cv::Mat &black_mask) {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        cv::Mat imag2;
        cv::Scalar low, high;
        low = cv::Scalar(250, 190, 10);
        high = cv::Scalar(256, 200, 20);
        cv::inRange(frame, low, high, blue_mask);
        cv::inRange(frame, low, high, imag2);

        cv::bitwise_or(blue_mask, imag2, blue_mask); // 合并，现在 blue_mask 是处理过的蓝色框
        low = cv::Scalar(240, 240, 240);
        high = cv::Scalar(255, 255, 255);
        cv::inRange(frame, low, high, black_mask); // 处理音符部分
    }

    void print(std::vector<cv::RotatedRect> rects, cv::Vec4i processed_line, cv::Mat show) {
        // 绘制检测到的矩形
        for (const auto& rect : rects) {
            cv::Point2f vertices[4];
            rect.points(vertices); // 获取矩形的四个顶点
            // 绘制矩形
            for (int i = 0; i < 4; i++) {
                cv::line(show, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2); // 绿色矩形
            }
            // 打印矩形中心点信息
            std::cout << "Rect Center: (" << rect.center.x << ", " << rect.center.y << ")" << std::endl;
        }
        // 绘制检测到的直线
        cv::line(show, cv::Point(processed_line[0], processed_line[1]), cv::Point(processed_line[2], processed_line[3]), cv::Scalar(255, 0, 0), 2);
        // 显示图像
        cv::imshow("Detected Rectangles", show);
        cv::waitKey(1);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame;

            image_init(msg, frame, blue_mask, black_mask);
            cv::Mat blue_pre = blue_mask.clone();
            std::vector<cv::RotatedRect> rects;
            cv::Vec4i processed_line;
            cv::Mat show = frame.clone();

            if (!line_processing(black_mask, processed_line)) {
                std::cerr << "line_process_error";
                return;
            }

            while (true) {
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(blue_pre, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                if (contours.empty()) break;

                cv::RotatedRect rect = cv::minAreaRect(contours[0]);
                rects.emplace_back(rect);

                cv::Mat mask = cv::Mat::zeros(blue_pre.size(), CV_8UC1);
                cv::Point2f vertices[4];
                rect.points(vertices); // 存端点
                std::vector<cv::Point> poly;
                for (int i = 0; i < 4; i++) {
                    poly.push_back(vertices[i]);
                }
                cv::fillConvexPoly(mask, poly, cv::Scalar(255));
                blue_pre.setTo(0, mask); // 将矩形区域设置为背景色

            //     std::thread t([blue_pre, frame] {
            //         // cv::imshow("image", frame); cv::imshow("image3", blue_pre); cv::waitKey(1);
            //     });
            //     t.detach();
            }

            for (const auto& rect : rects) {
                cv::RotatedRect smoothed_rect = rect_detector.detectRects(rects); // 平滑处理矩形
                double dis = distanceToLine(smoothed_rect.center, processed_line);

                if (dis < 60 && dis > 0) { // 参数与延迟有关
                    if (my_click(smoothed_rect.center)) {
                        std::cout << "position send" << std::endl;
                        return;
                    }
                }

               std::cout << "dis: " << dis << " x:" << smoothed_rect.center.x << " y:" << smoothed_rect.center.y << std::endl;
            }


          //  print(rects, processed_line, show);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_INFO(this->get_logger(), "error%s", e.what());
        }
    }
};//节点类



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<image_precss> node=std::make_shared<image_precss>();
    rclcpp::spin(node);
    rclcpp::shutdown();


    cv::destroyAllWindows();
}
