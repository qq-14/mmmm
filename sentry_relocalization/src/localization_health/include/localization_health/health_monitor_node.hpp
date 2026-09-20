#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace localization_health {

class HealthMonitorNode : public rclcpp::Node
{
public:
  enum class HealthState
  {
    NORMAL,
    DEGRADED,
    GLOBAL_LOST
  };

  explicit HealthMonitorNode(const rclcpp::NodeOptions & options);

private:
  void declareParameters();
  void lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void relocStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void checkTimerCallback();
  void triggerRelocalization();
  void publishHealth(double lio_rate, double lio_age, double lio_speed, double lio_yaw_rate,
    bool zero_velocity, bool tf_ok, double map_odom_jump, bool reloc_fresh);

  static std::string stateToString(HealthState state);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  static double wrappedYawDiff(double lhs, double rhs);

  // 参数
  std::string lio_odom_topic_{"aft_mapped_to_init"};
  std::string reloc_status_topic_{"relocalization_status"};
  std::string wheel_odom_topic_{""};
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  std::string relocalize_service_{"/gicp_recall"};
  double check_period_{0.2};
  double lio_min_rate_{5.0};
  double lio_timeout_{0.5};
  double lio_max_jump_trans_{1.0};
  double lio_max_jump_yaw_deg_{30.0};
  double reloc_status_timeout_{3.0};
  double reloc_min_overlap_{0.4};
  double reloc_max_normalized_score_{0.2};
  double map_odom_max_jump_trans_{0.5};
  double map_odom_max_jump_yaw_deg_{10.0};
  double zero_velocity_thresh_{0.05};
  int degraded_enter_count_{2};
  int normal_enter_count_{5};
  int lost_enter_count_{3};
  double relocalize_retry_period_{10.0};
  bool enable_relocalize_trigger_{true};

  // 状态机
  HealthState state_{HealthState::NORMAL};
  int good_count_{0};
  int bad_count_{0};
  int lost_count_{0};
  rclcpp::Time last_relocalize_call_;

  // LIO 里程计
  std::mutex lio_mtx_;
  bool lio_seen_{false};
  rclcpp::Time last_lio_stamp_;
  double last_lio_x_{0.0};
  double last_lio_y_{0.0};
  double last_lio_yaw_{0.0};
  double lio_speed_{0.0};
  double lio_yaw_rate_{0.0};
  bool lio_jump_{false};
  std::deque<rclcpp::Time> lio_stamps_;

  // 重定位状态
  std::mutex reloc_mtx_;
  bool reloc_status_received_{false};
  rclcpp::Time reloc_status_stamp_;
  std::string reloc_state_{"unknown"};
  bool reloc_localized_{false};
  double reloc_overlap_{0.0};
  double reloc_score_{0.0};
  double reloc_inlier_{0.0};
  double reloc_planar_ratio_{0.0};

  // map->odom 跳变检测
  bool has_map_odom_{false};
  double last_map_odom_x_{0.0};
  double last_map_odom_y_{0.0};
  double last_map_odom_yaw_{0.0};

  // 轮式里程计（可选）
  bool wheel_seen_{false};
  double wheel_speed_{0.0};

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr degraded_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lost_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr reloc_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr relocalize_client_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace localization_health
