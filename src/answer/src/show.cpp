#include<string.h>
#include<functional>
#include<algorithm>
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include"cv_bridge/cv_bridge.h"
#include"opencv2/opencv.hpp"
#include<memory>

class picture_show : public rclcpp::Node {
public:
    picture_show() : rclcpp::Node("image_subscriber") {
        subscription_ = create_subscription<sensor_msgs::msg::Image>("raw_image", 10,
                                                                     std::bind(&picture_show::image_callback, this,
                                                                               std::placeholders::_1));
    }
        private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
       void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

          try {
              cv::Mat frame=cv_bridge::toCvCopy(msg,"bgr8")->image;
              cv::imshow("image",frame);
              cv::waitKey(1);
              static int count = 0;
              if (++count%30 == 0) {
                  cv::imwrite("/home/sfx233/test"+std::to_string(count)+".jpg",frame);
              }
          }catch (const cv_bridge::Exception& e){
                  RCLCPP_INFO(this->get_logger(),"cv转化失败%s",e.what());
              }
          };
       };


int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<picture_show>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
}