#include"image.h"



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

                if (dis < 90 && dis > 0) { // 参数与延迟有关
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