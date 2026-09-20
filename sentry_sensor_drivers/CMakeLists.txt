cmake_minimum_required(VERSION 3.8)
project(sentry_sensor_drivers)

find_package(ament_cmake REQUIRED)

# TODO: 添加节点 / 库目标（当前为无源码骨架，仅保留构建入口）
# add_executable(<node_name> src/<node_name>.cpp)
# ament_target_dependencies(<node_name> rclcpp ...)
# install(TARGETS <node_name> DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
