cmake_minimum_required(VERSION 3.22)
project(answer)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()
# 添加可执行文件
ament_auto_add_executable(
        show
        src/show.cpp
)

# 链接ROS库
target_link_libraries(show
        ${rclcpp_LIBRARIES}
        ${OpenCV_LIBRARIES}  # 链接生成的消息类型

)

ament_auto_package()