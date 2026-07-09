#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono_literals;

enum class State
{
  IDLE,
  TAKEOFF,
  APPROACH,
  LAND,
  LANDED
};

class DroneLandingController : public rclcpp::Node
{
public:
  DroneLandingController()
  : Node("drone_landing_controller")
  {
    takeoff_height_ = declare_parameter<double>("takeoff_height", 2.0);
    approach_threshold_xy_ = declare_parameter<double>("approach_threshold_xy", 0.1);
    descend_height_ = declare_parameter<double>("descend_height", 0.5);
    land_threshold_z_ = declare_parameter<double>("land_threshold_z", 0.15);
    descend_speed_ = declare_parameter<double>("descend_speed", 0.3);
    approach_speed_ = declare_parameter<double>("approach_speed", 0.5);
    aruco_timeout_ = declare_parameter<double>("aruco_timeout", 0.5);
    aruco_frame_is_flu_ = declare_parameter<bool>("aruco_frame_is_flu", true);
    pid_kp_xy_ = declare_parameter<double>("pid_kp_xy", 0.6);
    pid_ki_xy_ = declare_parameter<double>("pid_ki_xy", 0.0);
    pid_kd_xy_ = declare_parameter<double>("pid_kd_xy", 0.08);
    max_xy_speed_ = declare_parameter<double>("max_xy_speed", 0.6);
    land_align_time_ = declare_parameter<double>("land_align_time", 1.0);
    
    // [SỬA Ở ĐÂY] Đặt mặc định là true để Node này tự quản lý Takeoff, tránh lỗi ngắt OFFBOARD của PX4
    takeoff_before_landing_ = declare_parameter<bool>("takeoff_before_landing", true);

    std::string px4_ns = "/fmu/";
    const auto px4_out_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(px4_ns + "in/offboard_control_mode", 10);
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(px4_ns + "in/trajectory_setpoint", 10);
    vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(px4_ns + "in/vehicle_command", 10);

    aruco_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone_aruco/pose", 10, std::bind(&DroneLandingController::arucoPoseCallback, this, std::placeholders::_1));
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/start_landing", 10, std::bind(&DroneLandingController::startCallback, this, std::placeholders::_1));
    vehicle_local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      px4_ns + "out/vehicle_local_position", px4_out_qos, std::bind(&DroneLandingController::vehicleLocalPositionCallback, this, std::placeholders::_1));
    vehicle_attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      px4_ns + "out/vehicle_attitude", px4_out_qos, std::bind(&DroneLandingController::vehicleAttitudeCallback, this, std::placeholders::_1));
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      px4_ns + "out/vehicle_status", px4_out_qos, std::bind(&DroneLandingController::vehicleStatusCallback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(100ms, std::bind(&DroneLandingController::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Drone landing controller started (Takeoff Managed Mode)");
  }

private:
  void arucoPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    aruco_pose_ = *msg; aruco_pose_received_ = true; last_aruco_time_ = get_clock()->now();
  }
  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
    vehicle_x_ = msg->x; vehicle_y_ = msg->y; vehicle_z_ = msg->z; vehicle_local_pos_received_ = true;
  }
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
    nav_state_ = msg->nav_state; armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED); vehicle_status_received_ = true;
  }
  void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg) {
    attitude_q_w_ = msg->q[0]; attitude_q_x_ = msg->q[1]; attitude_q_y_ = msg->q[2]; attitude_q_z_ = msg->q[3]; vehicle_attitude_received_ = true;
  }

  void controlLoop() {
    if (state_ == State::TAKEOFF || state_ == State::APPROACH) publishOffboardControlMode();
    switch (state_) {
      case State::IDLE: handleIdle(); break;
      case State::TAKEOFF: handleTakeoff(); break;
      case State::APPROACH: handleApproach(); break;
      case State::LAND: handleLand(); break;
      case State::LANDED: handleLanded(); break;
    }
  }

  void handleIdle() {
    if (!vehicle_status_received_ || !vehicle_local_pos_received_ || !vehicle_attitude_received_) return;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "[IDLE] Ready. Run: ros2 topic pub /start_landing std_msgs/msg/Bool '{data: true}' -1");
  }

  void startCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    if (msg->data && state_ == State::IDLE) {
      resetPid();
      resetTakeoff();
      takeoff_x_ = vehicle_x_;
      takeoff_y_ = vehicle_y_;
      state_ = (takeoff_before_landing_ || !armed_) ? State::TAKEOFF : State::APPROACH;
    }
  }

void handleTakeoff() {
    publishTrajectorySetpoint(takeoff_x_, takeoff_y_, -takeoff_height_);

    if (offboard_setpoint_counter_ < 10) {
      ++offboard_setpoint_counter_;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[TAKEOFF] Streaming initial setpoints before OFFBOARD: %d/10",
        offboard_setpoint_counter_);
      return;
    }

    const bool offboard_active =
      nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    if (!offboard_active) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[TAKEOFF] Requesting OFFBOARD mode");
    }

    if (!armed_) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[TAKEOFF] Sending ARM command");
    }

    if (!offboard_active || !armed_) {
      return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[TAKEOFF] z=%.2f target=%.2f nav=%u armed=%s",
      vehicle_z_, -takeoff_height_, nav_state_, armed_ ? "yes" : "no");

    if (vehicle_z_ <= -takeoff_height_ + 0.3) {
      RCLCPP_INFO(get_logger(), "[TAKEOFF] Reached takeoff height. Starting APPROACH...");
      resetPid(); 
      state_ = State::APPROACH;
    }
  }

  void handleApproach() {
    if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
      publishVelocitySetpoint(0.0, 0.0, 0.0);
      return;
    }
    if (!hasFreshArucoPose()) {
      resetPid(); publishVelocitySetpoint(0.0, 0.0, 0.0); return;
    }

    const double dt = 0.1;
    const double body_error_x = aruco_pose_.pose.position.x;
    const double body_error_y = aruco_pose_.pose.position.y;
    const double dist_xy = std::hypot(body_error_x, body_error_y);

    const double body_cmd_x = updatePidAxis(body_error_x, pid_integral_x_, previous_error_x_, dt);
    const double body_cmd_y = updatePidAxis(body_error_y, pid_integral_y_, previous_error_y_, dt);

    double body_frd_x = body_cmd_x;
    double body_frd_y = aruco_frame_is_flu_ ? -body_cmd_y : body_cmd_y;
    const double speed = std::hypot(body_frd_x, body_frd_y);
    if (speed > max_xy_speed_) {
      const double scale = max_xy_speed_ / speed;
      body_frd_x *= scale; body_frd_y *= scale;
    }

    double ned_vx = 0.0, ned_vy = 0.0, ned_vz = 0.0;
    rotateBodyToNed(body_frd_x, body_frd_y, 0.0, ned_vx, ned_vy, ned_vz);

    // [SỬA Ở ĐÂY] Thêm Logic hạ dần độ cao (Navigation Z-axis)
    // Trong hệ NED, Z âm nghĩa là ở trên cao, Z dương là hướng xuống đất.
    // Nếu drone đang bay cao hơn mốc descend_height_, vận tốc Z sẽ được bơm thêm giá trị descend_speed_ (bay xuống).
    double current_altitude = -vehicle_z_; 
    if (current_altitude > descend_height_ + 0.1) {
        ned_vz = descend_speed_; 
    } else if (current_altitude < descend_height_ - 0.1) {
        ned_vz = -descend_speed_; // Kéo lên lại nếu lỡ rớt quá đà
    } else {
        ned_vz = 0.0; // Đã đạt độ cao an toàn để tracking
    }

    publishVelocitySetpoint(ned_vx, ned_vy, ned_vz);

    if (dist_xy < approach_threshold_xy_) {
      aligned_time_ += dt;
      if (aligned_time_ >= land_align_time_) {
        publishVelocitySetpoint(0.0, 0.0, 0.0);
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        state_ = State::LAND;
      }
    } else {
      aligned_time_ = 0.0;
    }
  }

  void handleLand() {
    if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }
    if (std::abs(vehicle_z_) < 0.1) state_ = State::LANDED;
  }

  void handleLanded() {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
  }

  void publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.position = state_ == State::TAKEOFF;
    msg.velocity = state_ == State::APPROACH;
    msg.acceleration = false; msg.attitude = false; msg.body_rate = false;
    offboard_mode_pub_->publish(msg);
  }

void publishTrajectorySetpoint(float x, float y, float z) {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.position = {x, y, z}; 
    msg.velocity = {NAN, NAN, NAN}; 
    msg.acceleration = {NAN, NAN, NAN}; 
    
    // [SỬA Ở ĐÂY] Không được để NAN. PX4 bắt buộc phải có góc Yaw hợp lệ
    msg.yaw = 0.0f; 
    
    trajectory_pub_->publish(msg);
  }

  void publishVelocitySetpoint(float vx, float vy, float vz) {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.position = {NAN, NAN, NAN}; 
    msg.velocity = {vx, vy, vz}; 
    msg.acceleration = {NAN, NAN, NAN}; 
    
    // [SỬA Ở ĐÂY] Khóa góc Yaw ở 0 độ (hướng Bắc) để drone không bị xoay ngang
    msg.yaw = 0.0f; 
    
    trajectory_pub_->publish(msg);
  }

  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.command = command; msg.param1 = param1; msg.param2 = param2;
    msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1; msg.from_external = true;
    vehicle_cmd_pub_->publish(msg);
  }

  double updatePidAxis(double error, double & integral, double & previous_error, double dt) const {
    integral = clamp(integral + error * dt, -1.0, 1.0);
    const double derivative = (error - previous_error) / dt;
    previous_error = error;
    return pid_kp_xy_ * error + pid_ki_xy_ * integral + pid_kd_xy_ * derivative;
  }

  void rotateBodyToNed(double body_x, double body_y, double body_z, double & ned_x, double & ned_y, double & ned_z) const {
    const double w = attitude_q_w_, x = attitude_q_x_, y = attitude_q_y_, z = attitude_q_z_;
    ned_x = (1.0 - 2.0 * (y * y + z * z)) * body_x + (2.0 * (x * y - z * w)) * body_y + (2.0 * (x * z + y * w)) * body_z;
    ned_y = (2.0 * (x * y + z * w)) * body_x + (1.0 - 2.0 * (x * x + z * z)) * body_y + (2.0 * (y * z - x * w)) * body_z;
    ned_z = (2.0 * (x * z - y * w)) * body_x + (2.0 * (y * z + x * w)) * body_y + (1.0 - 2.0 * (x * x + y * y)) * body_z;
  }

  void resetPid() { pid_integral_x_ = 0.0; pid_integral_y_ = 0.0; previous_error_x_ = 0.0; previous_error_y_ = 0.0; aligned_time_ = 0.0; }
  void resetTakeoff() { offboard_setpoint_counter_ = 0; }
  double clamp(double value, double min_value, double max_value) const { return std::max(min_value, std::min(value, max_value)); }
  bool hasFreshArucoPose() const {
    if (!aruco_pose_received_) return false;
    return (get_clock()->now() - last_aruco_time_).seconds() <= aruco_timeout_;
  }

  State state_{State::IDLE};
  bool armed_{false}; uint8_t nav_state_{0};
  double vehicle_x_{0.0}, vehicle_y_{0.0}, vehicle_z_{0.0};
  double takeoff_x_{0.0}, takeoff_y_{0.0};
  int offboard_setpoint_counter_{0};
  bool vehicle_local_pos_received_{false}, vehicle_status_received_{false}, vehicle_attitude_received_{false};
  double attitude_q_w_{1.0}, attitude_q_x_{0.0}, attitude_q_y_{0.0}, attitude_q_z_{0.0};

  geometry_msgs::msg::PoseStamped aruco_pose_;
  bool aruco_pose_received_{false};
  rclcpp::Time last_aruco_time_{0, 0, RCL_ROS_TIME};

  double takeoff_height_, approach_threshold_xy_, descend_height_, land_threshold_z_, descend_speed_, approach_speed_, aruco_timeout_;
  bool aruco_frame_is_flu_, takeoff_before_landing_;
  double pid_kp_xy_, pid_ki_xy_, pid_kd_xy_, max_xy_speed_, land_align_time_;
  double pid_integral_x_{0.0}, pid_integral_y_{0.0}, previous_error_x_{0.0}, previous_error_y_{0.0}, aligned_time_{0.0};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneLandingController>());
  rclcpp::shutdown();
  return 0;
}
