#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "px4_msgs/msg/landing_target_pose.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"

class Px4FrameConverterNode : public rclcpp::Node
{
public:
  Px4FrameConverterNode()
  : Node("px4_frame_converter_node")
  {
    aruco_pose_topic_ = declare_parameter<std::string>(
      "aruco_pose_topic", "/drone_aruco/pose");
    vehicle_attitude_topic_ = declare_parameter<std::string>(
      "vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
    vehicle_local_position_topic_ = declare_parameter<std::string>(
      "vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
    error_frd_topic_ = declare_parameter<std::string>(
      "error_frd_topic", "/aruco_px4/error_frd");
    error_ned_topic_ = declare_parameter<std::string>(
      "error_ned_topic", "/aruco_px4/error_ned");
    landing_target_topic_ = declare_parameter<std::string>(
      "landing_target_topic", "/fmu/in/landing_target_pose");

    input_pose_is_flu_ = declare_parameter<bool>("input_pose_is_flu", true);
    target_timeout_ = declare_parameter<double>("target_timeout", 0.3);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    cov_xy_ = declare_parameter<double>("cov_xy", 0.01);
    min_valid_altitude_ = declare_parameter<double>("min_valid_altitude", 1.0);

    const auto px4_out_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    error_frd_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      error_frd_topic_, 10);
    error_ned_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      error_ned_topic_, 10);

    landing_target_pub_ = create_publisher<px4_msgs::msg::LandingTargetPose>(
      landing_target_topic_, 10);

    aruco_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      aruco_pose_topic_,
      10,
      std::bind(&Px4FrameConverterNode::arucoPoseCallback, this, std::placeholders::_1));

    vehicle_attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      vehicle_attitude_topic_,
      px4_out_qos,
      std::bind(&Px4FrameConverterNode::vehicleAttitudeCallback, this, std::placeholders::_1));

    vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      vehicle_local_position_topic_,
      px4_out_qos,
      std::bind(&Px4FrameConverterNode::vehicleLocalPositionCallback, this, std::placeholders::_1));

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    landing_target_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&Px4FrameConverterNode::publishLandingTarget, this));

    RCLCPP_INFO(get_logger(), "PX4 frame converter started (Precision Landing Mode)");
    RCLCPP_INFO(
      get_logger(),
      "Publishing LandingTargetPose to %s for commander mode auto:precland",
      landing_target_topic_.c_str());
  }

private:
  void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg)
  {
    attitude_q_w_ = msg->q[0];
    attitude_q_x_ = msg->q[1];
    attitude_q_y_ = msg->q[2];
    attitude_q_z_ = msg->q[3];
    vehicle_attitude_received_ = true;
  }

  void vehicleLocalPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg)
  {
    vehicle_x_ = msg->x;
    vehicle_y_ = msg->y;
    vehicle_z_ = msg->z;
    vehicle_local_position_valid_ = msg->xy_valid && msg->z_valid;
    vehicle_local_position_received_ = true;
  }

  void arucoPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    const double input_x = msg->pose.position.x;
    const double input_y = msg->pose.position.y;
    const double input_z = msg->pose.position.z;

    const double frd_x = input_x;
    const double frd_y = input_pose_is_flu_ ? -input_y : input_y;
    const double frd_z = input_pose_is_flu_ ? -input_z : input_z;

    geometry_msgs::msg::Vector3Stamped error_frd;
    error_frd.header = msg->header;
    error_frd.header.frame_id = "base_link_frd";
    error_frd.vector.x = frd_x;
    error_frd.vector.y = frd_y;
    error_frd.vector.z = frd_z;
    error_frd_pub_->publish(error_frd);

    if (!vehicle_attitude_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Waiting for PX4 vehicle attitude on %s",
        vehicle_attitude_topic_.c_str());
      return;
    }

    double ned_x = 0.0;
    double ned_y = 0.0;
    double ned_z = 0.0;
    rotateBodyFrdToNed(frd_x, frd_y, frd_z, ned_x, ned_y, ned_z);

    geometry_msgs::msg::Vector3Stamped error_ned;
    error_ned.header = msg->header;
    error_ned.header.frame_id = "map_ned";
    error_ned.vector.x = ned_x;
    error_ned.vector.y = ned_y;
    error_ned.vector.z = ned_z;
    error_ned_pub_->publish(error_ned);

    latest_ned_x_ = ned_x;
    latest_ned_y_ = ned_y;
    latest_ned_z_ = ned_z;
    latest_target_time_ = get_clock()->now();
    target_received_ = true;

    if (!vehicle_local_position_received_ || !vehicle_local_position_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Waiting for valid PX4 vehicle local position on %s",
        vehicle_local_position_topic_.c_str());
    }
  }

  void publishLandingTarget()
  {
    px4_msgs::msg::LandingTargetPose landing_target;

    landing_target.timestamp = get_clock()->now().nanoseconds() / 1000;
    landing_target.is_static = true;
    landing_target.rel_vel_valid = false;

    const bool fresh_target =
      target_received_ &&
      (get_clock()->now() - latest_target_time_).seconds() <= target_timeout_;
    const double altitude_above_local_origin = -vehicle_z_;
    const bool altitude_allows_target =
      vehicle_local_position_received_ &&
      vehicle_local_position_valid_ &&
      altitude_above_local_origin > min_valid_altitude_;
    const bool usable_target =
      fresh_target && altitude_allows_target;

    landing_target.rel_pos_valid = usable_target;
    landing_target.abs_pos_valid = usable_target;  // PX4 does not use absolute position for precision landing

    if (usable_target) {
      landing_target.x_rel = static_cast<float>(latest_ned_x_);
      landing_target.y_rel = static_cast<float>(latest_ned_y_);
      landing_target.z_rel = static_cast<float>(latest_ned_z_);
      landing_target.x_abs = static_cast<float>(vehicle_x_ + latest_ned_x_);
      landing_target.y_abs = static_cast<float>(vehicle_y_ + latest_ned_y_);
      landing_target.z_abs = static_cast<float>(vehicle_z_ + latest_ned_z_);
    } else {
      logInvalidTargetState(fresh_target);
    }

    landing_target.vx_rel = 0.0F;
    landing_target.vy_rel = 0.0F;
    landing_target.cov_x_rel = static_cast<float>(cov_xy_);
    landing_target.cov_y_rel = static_cast<float>(cov_xy_);
    landing_target.cov_vx_rel = 0.0F;
    landing_target.cov_vy_rel = 0.0F;

    landing_target_pub_->publish(landing_target);
  }

  void logInvalidTargetState(bool fresh_target)
  {
    const double target_age =
      target_received_ ? (get_clock()->now() - latest_target_time_).seconds() : -1.0;

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Landing target invalid: target_received=%s fresh=%s age=%.3f attitude=%s local_pos_received=%s local_pos_valid=%s altitude=%.3f min_valid_altitude=%.3f",
      target_received_ ? "true" : "false",
      fresh_target ? "true" : "false",
      target_age,
      vehicle_attitude_received_ ? "true" : "false",
      vehicle_local_position_received_ ? "true" : "false",
      vehicle_local_position_valid_ ? "true" : "false",
      -vehicle_z_,
      min_valid_altitude_);
  }

  void rotateBodyFrdToNed(
    double body_x, double body_y, double body_z,
    double & ned_x, double & ned_y, double & ned_z) const
  {
    const double w = attitude_q_w_;
    const double x = attitude_q_x_;
    const double y = attitude_q_y_;
    const double z = attitude_q_z_;

    ned_x =
      (1.0 - 2.0 * (y * y + z * z)) * body_x +
      (2.0 * (x * y - z * w)) * body_y +
      (2.0 * (x * z + y * w)) * body_z;
    ned_y =
      (2.0 * (x * y + z * w)) * body_x +
      (1.0 - 2.0 * (x * x + z * z)) * body_y +
      (2.0 * (y * z - x * w)) * body_z;
    ned_z =
      (2.0 * (x * z - y * w)) * body_x +
      (2.0 * (y * z + x * w)) * body_y +
      (1.0 - 2.0 * (x * x + y * y)) * body_z;
  }

  std::string aruco_pose_topic_;
  std::string vehicle_attitude_topic_;
  std::string vehicle_local_position_topic_;
  std::string error_frd_topic_;
  std::string error_ned_topic_;
  std::string landing_target_topic_;

  bool input_pose_is_flu_{true};
  double target_timeout_{0.3};
  double publish_rate_hz_{30.0};
  double cov_xy_{0.01};
  double min_valid_altitude_{1.0};

  bool vehicle_attitude_received_{false};
  bool vehicle_local_position_received_{false};
  bool vehicle_local_position_valid_{false};
  bool target_received_{false};
  double attitude_q_w_{1.0};
  double attitude_q_x_{0.0};
  double attitude_q_y_{0.0};
  double attitude_q_z_{0.0};
  double latest_ned_x_{0.0};
  double latest_ned_y_{0.0};
  double latest_ned_z_{0.0};
  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  double vehicle_z_{0.0};
  rclcpp::Time latest_target_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_frd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_ned_pub_;
  rclcpp::Publisher<px4_msgs::msg::LandingTargetPose>::SharedPtr landing_target_pub_;
  rclcpp::TimerBase::SharedPtr landing_target_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4FrameConverterNode>());
  rclcpp::shutdown();
  return 0;
}
