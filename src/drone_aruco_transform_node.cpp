#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class DroneArucoTransformNode : public rclcpp::Node
{
public:
  DroneArucoTransformNode()
  : Node("drone_aruco_transform_node")
  {
    camera_frame_ = declare_parameter<std::string>("camera_frame", "cameradown_link");
    drone_frame_ = declare_parameter<std::string>("drone_frame", "base_link");
    aruco_frame_prefix_ = declare_parameter<std::string>("aruco_frame_prefix", "aruco_");
    publish_annotated_ = declare_parameter<bool>("publish_annotated", true);
    static_camera_transform_ = declare_parameter<bool>("static_camera_transform", true);

    // [SỬA Ở ĐÂY] Căn chỉnh lại hệ trục tọa độ giữa Drone (FLU) và Camera (Optical)
    // Roll = 180 độ (PI) để trục Z (tia nhìn của camera) cắm thẳng xuống đất.
    // Yaw = -90 độ (-PI/2) để trục X của ảnh khớp với bên phải của drone, Y khớp với phía sau.
    camera_offset_x_ = declare_parameter<double>("camera_offset_x", 0.0);
    camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.0);
    camera_offset_z_ = declare_parameter<double>("camera_offset_z", -0.02);
    camera_offset_roll_ = declare_parameter<double>("camera_offset_roll", M_PI);
    camera_offset_pitch_ = declare_parameter<double>("camera_offset_pitch", 0.0);
    camera_offset_yaw_ = declare_parameter<double>("camera_offset_yaw", -M_PI / 2.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("drone_aruco/poses", 10);
    single_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("drone_aruco/pose", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("drone_aruco/markers", 10);

    aruco_poses_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "aruco/poses", 10,
      std::bind(&DroneArucoTransformNode::arucoPosesCallback, this, std::placeholders::_1));

    if (static_camera_transform_) {
      publishCameraTransform();
    }

    RCLCPP_INFO(get_logger(), "Drone-Aruco transform node started");
  }

private:
  void arucoPosesCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
  {
    if (msg->poses.empty()) return;

    const std::string pose_camera_frame =
      msg->header.frame_id.empty() ? camera_frame_ : msg->header.frame_id;

    if (pose_camera_frame != camera_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Aruco pose frame '%s' differs from configured camera_frame '%s'",
        pose_camera_frame.c_str(),
        camera_frame_.c_str());
    }

    geometry_msgs::msg::PoseArray drone_poses;
    drone_poses.header.stamp = msg->header.stamp;
    drone_poses.header.frame_id = drone_frame_;

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    visualization_msgs::msg::MarkerArray marker_array;

    appendDeleteAllMarker(marker_array, drone_poses.header);

    for (std::size_t i = 0; i < msg->poses.size(); ++i) {
      const auto & camera_aruco_pose = msg->poses[i];

      geometry_msgs::msg::TransformStamped drone_to_camera;
      try {
        drone_to_camera = tf_buffer_->lookupTransform(
          drone_frame_, pose_camera_frame, msg->header.stamp, std::chrono::milliseconds(100));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          3000,
          "Could not transform %s -> %s at stamp %.3f: %s",
          drone_frame_.c_str(),
          pose_camera_frame.c_str(),
          rclcpp::Time(msg->header.stamp).seconds(),
          ex.what());
        continue;
      }

      tf2::Transform t_camera_aruco;
      t_camera_aruco.setOrigin(tf2::Vector3(
        camera_aruco_pose.position.x, camera_aruco_pose.position.y, camera_aruco_pose.position.z));
      t_camera_aruco.setRotation(tf2::Quaternion(
        camera_aruco_pose.orientation.x, camera_aruco_pose.orientation.y,
        camera_aruco_pose.orientation.z, camera_aruco_pose.orientation.w));

      tf2::Transform t_drone_camera;
      t_drone_camera.setOrigin(tf2::Vector3(
        drone_to_camera.transform.translation.x, drone_to_camera.transform.translation.y, drone_to_camera.transform.translation.z));
      t_drone_camera.setRotation(tf2::Quaternion(
        drone_to_camera.transform.rotation.x, drone_to_camera.transform.rotation.y,
        drone_to_camera.transform.rotation.z, drone_to_camera.transform.rotation.w));

      tf2::Transform t_drone_aruco = t_drone_camera * t_camera_aruco;
      const auto & origin = t_drone_aruco.getOrigin();
      const auto & rot = t_drone_aruco.getRotation();

      geometry_msgs::msg::Pose drone_pose;
      drone_pose.position.x = origin.x();
      drone_pose.position.y = origin.y();
      drone_pose.position.z = origin.z();
      drone_pose.orientation.x = rot.x();
      drone_pose.orientation.y = rot.y();
      drone_pose.orientation.z = rot.z();
      drone_pose.orientation.w = rot.w();

      drone_poses.poses.push_back(drone_pose);

      if (i == msg->poses.size() - 1) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = drone_poses.header;
        pose_stamped.pose = drone_pose;
        single_pose_pub_->publish(pose_stamped);
      }

      int marker_id = static_cast<int>(i);
      geometry_msgs::msg::TransformStamped drone_aruco_tf;
      drone_aruco_tf.header = drone_poses.header;
      drone_aruco_tf.child_frame_id = aruco_frame_prefix_ + std::to_string(marker_id);
      drone_aruco_tf.transform.translation.x = origin.x();
      drone_aruco_tf.transform.translation.y = origin.y();
      drone_aruco_tf.transform.translation.z = origin.z();
      drone_aruco_tf.transform.rotation = drone_pose.orientation;
      transforms.push_back(drone_aruco_tf);

      appendRvizMarkers(marker_array, drone_poses.header, drone_pose, marker_id);
    }

    poses_pub_->publish(drone_poses);
    markers_pub_->publish(marker_array);
    if (!transforms.empty()) tf_broadcaster_->sendTransform(transforms);
  }

  void publishCameraTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = get_clock()->now();
    transform.header.frame_id = drone_frame_;
    transform.child_frame_id = camera_frame_;

    transform.transform.translation.x = camera_offset_x_;
    transform.transform.translation.y = camera_offset_y_;
    transform.transform.translation.z = camera_offset_z_;

    tf2::Quaternion q;
    q.setRPY(camera_offset_roll_, camera_offset_pitch_, camera_offset_yaw_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    static_tf_broadcaster_->sendTransform(transform);
  }

  void appendDeleteAllMarker(visualization_msgs::msg::MarkerArray & array, const std_msgs::msg::Header & header) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(marker);
  }

  void appendRvizMarkers(
    visualization_msgs::msg::MarkerArray & array, const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Pose & pose, int marker_id) const
  {
    visualization_msgs::msg::Marker drone_marker;
    drone_marker.header = header;
    drone_marker.ns = "drone_aruco";
    drone_marker.id = marker_id * 10;
    drone_marker.type = visualization_msgs::msg::Marker::SPHERE;
    drone_marker.action = visualization_msgs::msg::Marker::ADD;
    drone_marker.pose = pose;
    drone_marker.scale.x = 0.05; drone_marker.scale.y = 0.05; drone_marker.scale.z = 0.05;
    drone_marker.color.r = 1.0F; drone_marker.color.g = 0.2F; drone_marker.color.b = 0.2F; drone_marker.color.a = 0.9F;
    array.markers.push_back(drone_marker);

    visualization_msgs::msg::Marker label;
    label.header = header;
    label.ns = "drone_aruco_label";
    label.id = marker_id * 10 + 1;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position = pose.position;
    label.pose.position.z += 0.1;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.04;
    label.color.r = 1.0F; label.color.g = 1.0F; label.color.b = 1.0F; label.color.a = 1.0F;

    double dist = std::sqrt(pose.position.x * pose.position.x + pose.position.y * pose.position.y + pose.position.z * pose.position.z);
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << "Drone->ArUco " << marker_id << " d=" << dist << "m";
    label.text = ss.str();
    array.markers.push_back(label);
  }

  std::string camera_frame_, drone_frame_, aruco_frame_prefix_;
  bool publish_annotated_, static_camera_transform_;
  double camera_offset_x_, camera_offset_y_, camera_offset_z_;
  double camera_offset_roll_, camera_offset_pitch_, camera_offset_yaw_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_poses_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr single_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneArucoTransformNode>());
  rclcpp::shutdown();
  return 0;
}
