#include<string.h>
#include<functional>
#include<algorithm>
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include"cv_bridge/cv_bridge.h"
#include"opencv2/opencv.hpp"
#include<memory>

class image_precss : public rclcpp::Node {
public:
    image_precss() : rclcpp::Node("image_subscriber") {
        subscription_ = create_subscription<sensor_msgs::msg::Image>("raw_image", 10,
                                                                     std::bind(&image_precss::image_callback, this,
                                                                               std::placeholders::_1));
    }
        private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
       void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

          try {
              cv::Mat frame=cv_bridge::toCvCopy(msg,"bgr8")->image;
              cv::imshow("image",frame);
              cv::Mat imag1, imag2,imag3;
              cv::Scalar low, high;
              low = cv::Scalar(250,190,10);
              high = cv::Scalar(256,200,20);
              cv::inRange(frame,low,high,imag1);
              cv::inRange(frame,high,low,imag2);
              cv::bitwise_or(imag1,imag2,imag1);//合并，现在imag1是处理过的蓝色框

              low=cv::Scalar(250,250,250);
              high=cv::Scalar(255,255,255);
              cv::inRange(frame,low,high,imag3);
              cv::bitwise_or(imag3,imag1,imag1);
              cv::imshow("image1",imag1);
              cv::waitKey(1);


          }catch (const cv_bridge::Exception& e){
                  RCLCPP_INFO(this->get_logger(),"cv转化失败%s",e.what());
              }
          };
       };


int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<image_precss>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
}