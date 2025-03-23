//
// Created by sfx233 on 25-3-23.
//
#include"image_process.cpp"
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<image_precss> node=std::make_shared<image_precss>();
    rclcpp::spin(node);
    rclcpp::shutdown();


    cv::destroyAllWindows();
}