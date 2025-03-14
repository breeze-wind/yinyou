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


class click {
public:
    click()=default;
    bool operator()(cv::Point2f p) {
        try {
            // 计算当前 p 和 last_p 的欧氏距离平方
            double dx = p.x - last_p.x;
            double dy = p.y - last_p.y;
            double distance_squared = dx * dx + dy * dy;

            // 如果距离小于阈值，不进行 publish
            if (distance_squared < min_distance_squared) {
                return false;
            }

            // 更新 last_p
            last_p = p;

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
        publisher=pub;
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr publisher; // 发布器
    double min_distance_squared=10.0; // 最小距离的平方（避免开方计算）
    cv::Point2f last_p; // 上一次的 p 值
};

// 计算点到直线的距离
double distanceToLine(const cv::Point &point, const cv::Vec4f &line) {
    double x0 = point.x, y0 = point.y;
    double x1 = line[0], y1 = line[1];
    double x2 = line[2], y2 = line[3];

    double A = y2 - y1;
    double B = x1 - x2;
    double C = x2 * y1 - x1 * y2;

    return std::abs(A * x0 + B * y0 + C) / std::sqrt(A * A + B * B);
}


class image_precss : public rclcpp::Node {
public:
    image_precss() : rclcpp::Node("image_subscriber") {
        subscription_ = create_subscription<sensor_msgs::msg::Image>("raw_image", 10,
                                                                     std::bind(&image_precss::image_callback, this,
                                                                               std::placeholders::_1));
        msg_pub=this->create_publisher<geometry_msgs::msg::Point32>("click_position",10);
         my_click.Publisher(msg_pub);
    }
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr msg_pub;
   click my_click;
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    bool line_processing(const cv::Mat& frame, const cv::Mat& black_mask, cv::Mat &show, cv::Point2f &line_start, cv::Point2f &line_end) {
        std::vector<cv::Vec4i> lines;
        show = frame.clone();
        int count = 0;
        cv::HoughLinesP(black_mask, lines, 1.0,CV_PI / 180, 500);
        std::vector<cv::Point2f> points;
        for (const auto &line: lines) {
            points.emplace_back(line[0], line[1]); // 起点
            points.emplace_back(line[2], line[3]); // 终点
        }if (points.size() < 2) {
            std::cerr << "Error: Not enough points to fit a line." << std::endl;
            return false;
        }

        cv::Vec4f line_params; // 存储拟合直线的参数 (vx, vy, x0, y0)
        cv::fitLine(points, line_params, cv::DIST_L2, 0, 0.01, 0.01);
        float vx = line_params[0];
        float vy = line_params[1];
        float x0 = line_params[2];
        float y0 = line_params[3];

        line_start = cv::Point2f(x0 - vx * 1000, y0 - vy * 1000);
        line_end = cv::Point2f(x0 + vx * 1000, y0 + vy * 1000);
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

        cv::bitwise_or(blue_mask, imag2, blue_mask); //合并，现在imag1是处理过的蓝色框
        // cv::inRange(frame, low, high, imag2);
        // cv::bitwise_or(blue_mask, imag2, blue_mask);
        low = cv::Scalar(240, 240, 240);
        high = cv::Scalar(255, 255, 255);
        cv::inRange(frame, low, high, black_mask); //处理音符部分
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame;
            cv::Mat blue_mask;
            cv::Mat black_mask;
            image_init(msg, frame, blue_mask, black_mask);
            ////////
            cv::Mat blue_pre=blue_mask.clone();
            std::vector<cv::RotatedRect> rects;
            cv::Point2f line_start;
            cv::Point2f line_end;//判定线的首尾坐标
            cv::Mat show;
            if (!line_processing(frame, black_mask, show, line_start, line_end)) {std::cerr<<"line_process_error";return;}
            while (true) {
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(blue_pre, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                // 如果没有检测到轮廓，退出循环
                if (contours.empty()) break;
                cv::RotatedRect rect = cv::minAreaRect(contours[0]);
                 rects.emplace_back(rect);

                cv::Mat mask = cv::Mat::zeros(blue_pre.size(), CV_8UC1);
                cv::Point2f vertices[4];
                rect.points(vertices);//存端点
                std::vector<cv::Point> poly;
                for (int i = 0; i < 4; i++) {
                    poly.push_back(vertices[i]);
                }
                cv::fillConvexPoly(mask, poly, cv::Scalar(255));
                blue_pre.setTo(0, mask); // 将矩形区域设置为背景色
                // std::thread t( [blue_pre,frame] {
                //     cv::imshow("image", frame);cv::imshow("image3", blue_pre);cv::waitKey(1);
                // });
               //t.detach();
            }
            for (const auto &rect: rects) {
               double dis= distanceToLine(rect.center,cv::Vec4i(line_start.x,line_start.y,line_end.x,line_end.y));
                if (dis<50&&dis>0) {//参数与延迟有关

                if (my_click(rect.center)) {
                    std::cout<<"position send"<<std::endl;
                    return ;
                }

                }
                std::cout<<"dis: "<<dis<<" x:"<<rect.center.x<<" y:"<<rect.center.y<<std::endl;
            }


            cv::line(show, line_start, line_end, cv::Scalar(255, 0, 0), 2);
           // std::cout << show.type() << std::endl;
           //  cv::imshow("image1", blue_mask);
           //  cv::waitKey(1);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_INFO(this->get_logger(), "error%s", e.what());
        }
    };
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<image_precss>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
}
