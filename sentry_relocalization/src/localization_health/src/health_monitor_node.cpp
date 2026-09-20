#include "localization_health/health_monitor_node.hpp"

#include <cmath>

#include "tf2/exceptions.h"

namespace localization_health {

HealthMonitorNode::HealthMonitorNode(const rclcpp::NodeOptions & options)
: Node("localization_health_monitor", options)
{
  declareParameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  health_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("localization_health", 10);
  degraded_pub_ = this->create_publisher<std_msgs::msg::Bool>("localization_degraded", 10);
  lost_pub_ = this->create_publisher<std_msgs::msg::Bool>("localization_lost", 10);

  lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    lio_odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&HealthMonitorNode::lioOdomCallback, this, std::placeholders::_1));
  reloc_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    reloc_status_topic_, 10,
    std::bind(&HealthMonitorNode::relocStatusCallback, this, std::placeholders::_1));
  if (!wheel_odom_topic_.empty()) {
    wheel_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HealthMonitorNode::wheelOdomCallback, this, std::placeholders::_1));
  }

  relocalize_client_ = this->create_client<std_srvs::srv::Trigger>(relocalize_service_);
  check_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(check_period_),
    std::bind(&HealthMonitorNode::checkTimerCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "[health] started: lio=%s reloc_status=%s service=%s check=%.2fs",
    lio_odom_topic_.c_str(), reloc_status_topic_.c_str(), relocalize_service_.c_str(),
    check_period_);
}

void HealthMonitorNode::declareParameters()
{
  lio_odom_topic_ = this->declare_parameter<std::string>("lio_odom_topic", "aft_mapped_to_init");
  reloc_status_topic_ =
    this->declare_parameter<std::string>("reloc_status_topic", "relocalization_status");
  wheel_odom_topic_ = this->declare_parameter<std::string>("wheel_odom_topic", "");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
  relocalize_service_ = this->declare_parameter<std::string>("relocalize_service", "/gicp_recall");
  check_period_ = this->declare_parameter<double>("check_period", 0.2);
  lio_min_rate_ = this->declare_parameter<double>("lio_min_rate", 5.0);
  lio_timeout_ = this->declare_parameter<double>("lio_timeout", 0.5);
  lio_max_jump_trans_ = this->declare_parameter<double>("lio_max_jump_trans", 1.0);
  lio_max_jump_yaw_deg_ = this->declare_parameter<double>("lio_max_jump_yaw_deg", 30.0);
  reloc_status_timeout_ = this->declare_parameter<double>("reloc_status_timeout", 3.0);
  reloc_min_overlap_ = this->declare_parameter<double>("reloc_min_overlap", 0.4);
  reloc_max_normalized_score_ =
    this->declare_parameter<double>("reloc_max_normalized_score", 0.2);
  map_odom_max_jump_trans_ = this->declare_parameter<double>("map_odom_max_jump_trans", 0.5);
  map_odom_max_jump_yaw_deg_ =
    this->declare_parameter<double>("map_odom_max_jump_yaw_deg", 10.0);
  zero_velocity_thresh_ = this->declare_parameter<double>("zero_velocity_thresh", 0.05);
  degraded_enter_count_ = this->declare_parameter<int>("degraded_enter_count", 2);
  normal_enter_count_ = this->declare_parameter<int>("normal_enter_count", 5);
  lost_enter_count_ = this->declare_parameter<int>("lost_enter_count", 3);
  relocalize_retry_period_ = this->declare_parameter<double>("relocalize_retry_period", 10.0);
  enable_relocalize_trigger_ = this->declare_parameter<bool>("enable_relocalize_trigger", true);
}

std::string HealthMonitorNode::stateToString(HealthState state)
{
  switch (state) {
    case HealthState::NORMAL:
      return "NORMAL";
    case HealthState::DEGRADED:
      return "DEGRADED";
    case HealthState::GLOBAL_LOST:
      return "GLOBAL_LOST";
  }
  return "UNKNOWN";
}

double HealthMonitorNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double HealthMonitorNode::wrappedYawDiff(double lhs, double rhs)
{
  return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

void HealthMonitorNode::lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  const double yaw = yawFromQuaternion(msg->pose.pose.orientation);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    return;
  }

  std::lock_guard<std::mutex> lock(lio_mtx_);
  if (lio_seen_) {
    const double dt = (stamp - last_lio_stamp_).seconds();
    if (dt > 1e-3 && dt < 1.0) {
      const double dx = x - last_lio_x_;
      const double dy = y - last_lio_y_;
      lio_speed_ = std::hypot(dx, dy) / dt;
      lio_yaw_rate_ = std::abs(wrappedYawDiff(yaw, last_lio_yaw_)) / dt;
      if (std::hypot(dx, dy) > lio_max_jump_trans_ ||
          std::abs(wrappedYawDiff(yaw, last_lio_yaw_)) >
            lio_max_jump_yaw_deg_ * M_PI / 180.0) {
        lio_jump_ = true;
      }
    }
  }
  last_lio_stamp_ = stamp;
  last_lio_x_ = x;
  last_lio_y_ = y;
  last_lio_yaw_ = yaw;
  lio_seen_ = true;

  lio_stamps_.push_back(stamp);
  while (!lio_stamps_.empty() && (stamp - lio_stamps_.front()).seconds() > 1.0) {
    lio_stamps_.pop_front();
  }
}

void HealthMonitorNode::relocStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(reloc_mtx_);
  reloc_status_received_ = true;
  reloc_status_stamp_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  for (const auto & status : msg->status) {
    if (status.name != "relocalization") {
      continue;
    }
    reloc_state_ = status.message;
    reloc_localized_ = false;
    for (const auto & kv : status.values) {
      try {
        if (kv.key == "localized") {
          reloc_localized_ = (kv.value == "true");
        } else if (kv.key == "overlap_ratio") {
          reloc_overlap_ = std::stod(kv.value);
        } else if (kv.key == "normalized_score") {
          reloc_score_ = std::stod(kv.value);
        } else if (kv.key == "inlier_ratio") {
          reloc_inlier_ = std::stod(kv.value);
        } else if (kv.key == "planar_eigen_ratio") {
          reloc_planar_ratio_ = std::stod(kv.value);
        }
      } catch (const std::exception &) {
        // 忽略无法解析的字段
      }
    }
  }
}

void HealthMonitorNode::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(lio_mtx_);
  wheel_seen_ = true;
  wheel_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
}

void HealthMonitorNode::triggerRelocalization()
{
  const auto now = this->now();
  if (last_relocalize_call_.nanoseconds() != 0 &&
      (now - last_relocalize_call_).seconds() < relocalize_retry_period_) {
    return;
  }
  last_relocalize_call_ = now;
  if (!relocalize_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "[health] relocalization service not available");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  relocalize_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "[health] relocalization triggered: %s",
        response->success ? "accepted" : "rejected");
    });
  RCLCPP_WARN(this->get_logger(), "[health] GLOBAL_LOST -> trigger relocalization");
}

void HealthMonitorNode::checkTimerCallback()
{
  const auto now = this->now();

  // ---- LIO 健康 ----
  double lio_rate = 0.0;
  double lio_age = 1e9;
  double lio_speed = 0.0;
  double lio_yaw_rate = 0.0;
  bool lio_jump = false;
  bool lio_seen = false;
  {
    std::lock_guard<std::mutex> lock(lio_mtx_);
    lio_seen = lio_seen_;
    lio_rate = static_cast<double>(lio_stamps_.size());
    if (lio_seen_) {
      lio_age = (now - last_lio_stamp_).seconds();
    }
    lio_speed = lio_speed_;
    lio_yaw_rate = lio_yaw_rate_;
    lio_jump = lio_jump_;
    lio_jump_ = false;  // 单次事件
  }
  const bool lio_ok = lio_seen && lio_age <= lio_timeout_ && lio_rate >= lio_min_rate_ && !lio_jump;
  const bool zero_velocity = lio_seen && lio_speed < zero_velocity_thresh_;

  // ---- TF 健康 ----
  bool tf_ok = false;
  double map_odom_jump = 0.0;
  try {
    const auto base_tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    (void)base_tf;
    const auto map_odom = tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);
    tf_ok = true;
    const double x = map_odom.transform.translation.x;
    const double y = map_odom.transform.translation.y;
    const double yaw = yawFromQuaternion(map_odom.transform.rotation);
    if (has_map_odom_) {
      const double dxy = std::hypot(x - last_map_odom_x_, y - last_map_odom_y_);
      const double dyaw = std::abs(wrappedYawDiff(yaw, last_map_odom_yaw_));
      map_odom_jump = std::max(dxy, dyaw);
      if (dxy > map_odom_max_jump_trans_ ||
          dyaw > map_odom_max_jump_yaw_deg_ * M_PI / 180.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[health] map->odom jump detected: dxy=%.3f m dyaw=%.1f deg", dxy, dyaw * 180.0 / M_PI);
        tf_ok = false;
      }
    }
    last_map_odom_x_ = x;
    last_map_odom_y_ = y;
    last_map_odom_yaw_ = yaw;
    has_map_odom_ = true;
  } catch (const tf2::TransformException &) {
    tf_ok = false;
  }

  // ---- 重定位健康 ----
  bool reloc_fresh = false;
  bool reloc_ok = true;
  bool reloc_localized = false;
  double reloc_overlap = 0.0;
  double reloc_score = 0.0;
  {
    std::lock_guard<std::mutex> lock(reloc_mtx_);
    if (reloc_status_received_) {
      reloc_fresh = (now - reloc_status_stamp_).seconds() <= reloc_status_timeout_;
      reloc_localized = reloc_localized_;
      reloc_overlap = reloc_overlap_;
      reloc_score = reloc_score_;
      if (reloc_fresh) {
        reloc_ok = reloc_localized ||
                   (reloc_overlap >= reloc_min_overlap_ && reloc_score <= reloc_max_normalized_score_);
      }
    }
  }

  // ---- 状态判定 ----
  const bool lost_condition = !tf_ok || !lio_seen || lio_age > 2.0 * lio_timeout_;
  const bool degraded_condition = !lio_ok || (reloc_fresh && !reloc_ok);

  if (lost_condition) {
    ++lost_count_;
    bad_count_ = 0;
    good_count_ = 0;
    if (lost_count_ >= lost_enter_count_ && state_ != HealthState::GLOBAL_LOST) {
      state_ = HealthState::GLOBAL_LOST;
      RCLCPP_ERROR(this->get_logger(), "[health] state -> GLOBAL_LOST");
    }
  } else if (degraded_condition) {
    ++bad_count_;
    lost_count_ = 0;
    good_count_ = 0;
    if (bad_count_ >= degraded_enter_count_ && state_ == HealthState::NORMAL) {
      state_ = HealthState::DEGRADED;
      RCLCPP_WARN(this->get_logger(), "[health] state -> DEGRADED");
    }
  } else {
    ++good_count_;
    bad_count_ = 0;
    lost_count_ = 0;
    if (good_count_ >= normal_enter_count_ && state_ != HealthState::NORMAL) {
      state_ = HealthState::NORMAL;
      RCLCPP_INFO(this->get_logger(), "[health] state -> NORMAL");
    }
  }

  if (state_ == HealthState::GLOBAL_LOST && enable_relocalize_trigger_) {
    triggerRelocalization();
  }

  publishHealth(lio_rate, lio_age, lio_speed, lio_yaw_rate, zero_velocity, tf_ok, map_odom_jump,
    reloc_fresh);
}

void HealthMonitorNode::publishHealth(double lio_rate, double lio_age, double lio_speed,
  double lio_yaw_rate, bool zero_velocity, bool tf_ok, double map_odom_jump, bool reloc_fresh)
{
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "localization_health";
  status.hardware_id = "localization";
  if (state_ == HealthState::NORMAL) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  } else if (state_ == HealthState::DEGRADED) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  status.message = stateToString(state_);

  auto add = [&status](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };
  add("state", stateToString(state_));
  add("lio_rate", std::to_string(lio_rate));
  add("lio_age", std::to_string(lio_age));
  add("lio_speed", std::to_string(lio_speed));
  add("lio_yaw_rate", std::to_string(lio_yaw_rate));
  add("zero_velocity", zero_velocity ? "true" : "false");
  add("tf_ok", tf_ok ? "true" : "false");
  add("map_odom_jump", std::to_string(map_odom_jump));
  add("reloc_fresh", reloc_fresh ? "true" : "false");
  {
    std::lock_guard<std::mutex> lock(reloc_mtx_);
    add("reloc_state", reloc_state_);
    add("reloc_localized", reloc_localized_ ? "true" : "false");
    add("reloc_overlap", std::to_string(reloc_overlap_));
    add("reloc_normalized_score", std::to_string(reloc_score_));
    add("reloc_inlier_ratio", std::to_string(reloc_inlier_));
    add("reloc_planar_eigen_ratio", std::to_string(reloc_planar_ratio_));
  }
  {
    std::lock_guard<std::mutex> lock(lio_mtx_);
    add("wheel_seen", wheel_seen_ ? "true" : "false");
    add("wheel_speed", std::to_string(wheel_speed_));
  }

  msg.status.push_back(status);
  health_pub_->publish(msg);

  std_msgs::msg::Bool degraded;
  degraded.data = (state_ == HealthState::DEGRADED);
  degraded_pub_->publish(degraded);
  std_msgs::msg::Bool lost;
  lost.data = (state_ == HealthState::GLOBAL_LOST);
  lost_pub_->publish(lost);
}

}  // namespace localization_health
