#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ArucoPoseNode : public rclcpp::Node
{
public:
  ArucoPoseNode()
  : Node("aruco_pose_node")
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/camera/image_raw");

    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera_info");

    marker_size_ = declare_parameter<double>(
      "marker_size", 0.5);

    const int dictionary_id = declare_parameter<int>(
      "dictionary_id", cv::aruco::DICT_4X4_50);

    publish_annotated_ = declare_parameter<bool>(
      "publish_annotated", true);

    camera_frame_override_ = declare_parameter<std::string>(
      "camera_frame", "");

    child_frame_prefix_ = declare_parameter<std::string>(
      "child_frame_prefix", "camera_aruco_");

    publish_fallback_camera_info_ = declare_parameter<bool>(
      "publish_fallback_camera_info", true);

    fallback_camera_width_ = declare_parameter<int>(
      "fallback_camera_width", 1280);

    fallback_camera_height_ = declare_parameter<int>(
      "fallback_camera_height", 720);

    fallback_camera_horizontal_fov_ = declare_parameter<double>(
      "fallback_camera_horizontal_fov", 1.2);

    if (marker_size_ <= 0.0) {
      throw std::invalid_argument("marker_size must be greater than zero");
    }

    if (fallback_camera_width_ <= 0 || fallback_camera_height_ <= 0) {
      throw std::invalid_argument("fallback camera dimensions must be greater than zero");
    }

    if (fallback_camera_horizontal_fov_ <= 0.0 || fallback_camera_horizontal_fov_ >= M_PI) {
      throw std::invalid_argument("fallback_camera_horizontal_fov must be in (0, pi)");
    }

    try {
      dictionary_ = cv::aruco::getPredefinedDictionary(
        static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary_id));
    } catch (const cv::Exception & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid ArUco dictionary ID %d: %s",
        dictionary_id,
        e.what());
      throw;
    }

    detector_parameters_ = cv::aruco::DetectorParameters::create();

    // Improve corner accuracy before pose estimation.
    detector_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "aruco/poses", 10);

    ids_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "aruco/pose_ids", 10);

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "aruco/markers", 10);

    if (publish_annotated_) {
      annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "aruco/pose_image", rclcpp::SensorDataQoS());
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ArucoPoseNode::cameraInfoCallback,
        this,
        std::placeholders::_1));

    if (publish_fallback_camera_info_) {
      fallback_camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        rclcpp::QoS(1).reliable().transient_local());

      fallback_camera_info_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ArucoPoseNode::publishFallbackCameraInfo, this));
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ArucoPoseNode::imageCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ArUco pose node started");
    RCLCPP_INFO(get_logger(), "Image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "CameraInfo topic: %s",
      camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Marker size: %.6f m", marker_size_);
    RCLCPP_INFO(
      get_logger(),
      "Outputs: /aruco/poses, /aruco/pose_ids, /aruco/markers and TF");

    if (publish_fallback_camera_info_) {
      RCLCPP_INFO(
        get_logger(),
        "Fallback CameraInfo enabled: %dx%d, horizontal_fov=%.3f rad",
        fallback_camera_width_,
        fallback_camera_height_,
        fallback_camera_horizontal_fov_);
    }
  }

private:
  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "CameraInfo is invalid: fx=%.6f, fy=%.6f",
        msg->k[0],
        msg->k[4]);
      return;
    }

    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        camera_matrix_.at<double>(row, col) = msg->k[row * 3 + col];
      }
    }

    if (msg->d.empty()) {
      distortion_coefficients_ = cv::Mat::zeros(5, 1, CV_64F);
    } else {
      distortion_coefficients_ = cv::Mat(
        static_cast<int>(msg->d.size()), 1, CV_64F);

      for (std::size_t i = 0; i < msg->d.size(); ++i) {
        distortion_coefficients_.at<double>(static_cast<int>(i), 0) = msg->d[i];
      }
    }

    calibration_width_ = static_cast<int>(msg->width);
    calibration_height_ = static_cast<int>(msg->height);
    camera_info_frame_ = msg->header.frame_id;

    if (!camera_info_ready_) {
      RCLCPP_INFO(
        get_logger(),
        "Received CameraInfo: %dx%d, frame='%s', fx=%.3f, fy=%.3f",
        calibration_width_,
        calibration_height_,
        camera_info_frame_.c_str(),
        camera_matrix_.at<double>(0, 0),
        camera_matrix_.at<double>(1, 1));
    }

    camera_info_ready_ = true;
  }

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!msg->header.frame_id.empty()) {
      last_image_frame_ = msg->header.frame_id;
    }

    if (!camera_info_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Waiting for valid CameraInfo on %s",
        camera_info_topic_.c_str());
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "cv_bridge failed: %s",
        e.what());
      return;
    }

    if (cv_ptr->image.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received empty image");
      return;
    }

    cv::Mat gray;

    if (!convertToGray(cv_ptr->image, msg->encoding, gray)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Unsupported image encoding: %s, channels: %d",
        msg->encoding.c_str(),
        cv_ptr->image.channels());
      return;
    }

    cv::Mat active_camera_matrix = camera_matrix_.clone();

    if (
      calibration_width_ > 0 && calibration_height_ > 0 &&
      (gray.cols != calibration_width_ || gray.rows != calibration_height_))
    {
      const double scale_x =
        static_cast<double>(gray.cols) / static_cast<double>(calibration_width_);
      const double scale_y =
        static_cast<double>(gray.rows) / static_cast<double>(calibration_height_);

      active_camera_matrix.at<double>(0, 0) *= scale_x;
      active_camera_matrix.at<double>(0, 2) *= scale_x;
      active_camera_matrix.at<double>(1, 1) *= scale_y;
      active_camera_matrix.at<double>(1, 2) *= scale_y;

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Image size %dx%d differs from CameraInfo %dx%d; intrinsic matrix was scaled",
        gray.cols,
        gray.rows,
        calibration_width_,
        calibration_height_);
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    try {
      cv::aruco::detectMarkers(
        gray,
        dictionary_,
        corners,
        ids,
        detector_parameters_,
        rejected);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "ArUco detection failed: %s",
        e.what());
      return;
    }

    const std::string camera_frame = resolveCameraFrame(*msg);

    geometry_msgs::msg::PoseArray poses_message;
    poses_message.header = msg->header;
    poses_message.header.frame_id = camera_frame;

    std_msgs::msg::Int32MultiArray ids_message;

    visualization_msgs::msg::MarkerArray marker_array;
    appendDeleteAllMarker(marker_array, poses_message.header);

    cv::Mat annotated;

    if (publish_annotated_) {
      if (!convertToBgr(cv_ptr->image, msg->encoding, annotated)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Could not create annotated image");
      } else if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(annotated, corners, ids);
      }
    }

    const std::vector<cv::Point3f> object_points = markerObjectPoints();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    for (std::size_t i = 0; i < ids.size(); ++i) {
      cv::Vec3d rotation_vector;
      cv::Vec3d translation_vector;

      bool solved = false;

      try {
        solved = cv::solvePnP(
          object_points,
          corners[i],
          active_camera_matrix,
          distortion_coefficients_,
          rotation_vector,
          translation_vector,
          false,
          cv::SOLVEPNP_IPPE_SQUARE);
      } catch (const cv::Exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "solvePnP failed for marker %d: %s",
          ids[i],
          e.what());
        continue;
      }

      if (!solved || translation_vector[2] <= 0.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "No valid pose solution for marker %d",
          ids[i]);
        continue;
      }

      const geometry_msgs::msg::Pose pose = poseFromOpenCv(
        rotation_vector,
        translation_vector);

      poses_message.poses.push_back(pose);
      ids_message.data.push_back(ids[i]);

      transforms.push_back(makeTransform(
        poses_message.header,
        pose,
        ids[i]));

      appendRvizMarkers(
        marker_array,
        poses_message.header,
        pose,
        ids[i]);

      if (!annotated.empty()) {
        cv::drawFrameAxes(
          annotated,
          active_camera_matrix,
          distortion_coefficients_,
          rotation_vector,
          translation_vector,
          static_cast<float>(marker_size_ * 0.75),
          2);

        drawPoseText(
          annotated,
          corners[i],
          ids[i],
          translation_vector);
      }
    }

    poses_pub_->publish(poses_message);
    ids_pub_->publish(ids_message);
    markers_pub_->publish(marker_array);

    if (!transforms.empty()) {
      tf_broadcaster_->sendTransform(transforms);
    }

    if (publish_annotated_ && !annotated.empty()) {
      cv_bridge::CvImage output;
      output.header = msg->header;
      output.header.frame_id = camera_frame;
      output.encoding = sensor_msgs::image_encodings::BGR8;
      output.image = annotated;
      annotated_pub_->publish(*output.toImageMsg());
    }

    if (!ids_message.data.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Published poses for ArUco IDs: %s",
        idsToString(ids_message.data).c_str());
    }
  }

  std::vector<cv::Point3f> markerObjectPoints() const
  {
    const float half = static_cast<float>(marker_size_ * 0.5);

    // Required corner order for SOLVEPNP_IPPE_SQUARE:
    // top-left, top-right, bottom-right, bottom-left.
    return {
      cv::Point3f(-half,  half, 0.0F),
      cv::Point3f( half,  half, 0.0F),
      cv::Point3f( half, -half, 0.0F),
      cv::Point3f(-half, -half, 0.0F)
    };
  }

  geometry_msgs::msg::Pose poseFromOpenCv(
    const cv::Vec3d & rotation_vector,
    const cv::Vec3d & translation_vector) const
  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);

    tf2::Matrix3x3 tf_rotation(
      rotation_matrix.at<double>(0, 0),
      rotation_matrix.at<double>(0, 1),
      rotation_matrix.at<double>(0, 2),
      rotation_matrix.at<double>(1, 0),
      rotation_matrix.at<double>(1, 1),
      rotation_matrix.at<double>(1, 2),
      rotation_matrix.at<double>(2, 0),
      rotation_matrix.at<double>(2, 1),
      rotation_matrix.at<double>(2, 2));

    tf2::Quaternion quaternion;
    tf_rotation.getRotation(quaternion);
    quaternion.normalize();

    geometry_msgs::msg::Pose pose;
    pose.position.x = translation_vector[0];
    pose.position.y = translation_vector[1];
    pose.position.z = translation_vector[2];
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
  }

  geometry_msgs::msg::TransformStamped makeTransform(
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Pose & pose,
    int marker_id) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = header;
    transform.child_frame_id = child_frame_prefix_ + std::to_string(marker_id);

    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;

    return transform;
  }

  void publishFallbackCameraInfo()
  {
    if (camera_info_ready_ || !publish_fallback_camera_info_) {
      return;
    }

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.stamp = now();
    camera_info.header.frame_id = fallbackCameraFrame();
    camera_info.width = static_cast<uint32_t>(fallback_camera_width_);
    camera_info.height = static_cast<uint32_t>(fallback_camera_height_);
    camera_info.distortion_model = "plumb_bob";
    camera_info.d.assign(5, 0.0);

    const double fx = static_cast<double>(fallback_camera_width_) /
      (2.0 * std::tan(fallback_camera_horizontal_fov_ * 0.5));
    const double fy = fx;
    const double cx = static_cast<double>(fallback_camera_width_) * 0.5;
    const double cy = static_cast<double>(fallback_camera_height_) * 0.5;

    camera_info.k = {
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0
    };

    camera_info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    camera_info.p = {
      fx, 0.0, cx, 0.0,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    fallback_camera_info_pub_->publish(camera_info);

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Published fallback CameraInfo on %s because no valid calibration was received",
      camera_info_topic_.c_str());
  }

  std::string fallbackCameraFrame() const
  {
    if (!camera_frame_override_.empty()) {
      return camera_frame_override_;
    }

    if (!last_image_frame_.empty()) {
      return last_image_frame_;
    }

    return "x500_downward_camera_0/cameradown_link/down_cam";
  }

  void appendDeleteAllMarker(
    visualization_msgs::msg::MarkerArray & array,
    const std_msgs::msg::Header & header) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(marker);
  }

  void appendRvizMarkers(
    visualization_msgs::msg::MarkerArray & array,
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Pose & pose,
    int marker_id) const
  {
    visualization_msgs::msg::Marker plate;
    plate.header = header;
    plate.ns = "aruco_pose";
    plate.id = marker_id * 10;
    plate.type = visualization_msgs::msg::Marker::CUBE;
    plate.action = visualization_msgs::msg::Marker::ADD;
    plate.pose = pose;
    plate.scale.x = marker_size_;
    plate.scale.y = marker_size_;
    plate.scale.z = std::max(0.001, marker_size_ * 0.05);
    plate.color.r = 0.1F;
    plate.color.g = 0.9F;
    plate.color.b = 0.2F;
    plate.color.a = 0.65F;
    array.markers.push_back(plate);

    visualization_msgs::msg::Marker label;
    label.header = header;
    label.ns = "aruco_label";
    label.id = marker_id * 10 + 1;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position = pose.position;
    label.pose.position.y -= marker_size_;
    label.pose.orientation.w = 1.0;
    label.scale.z = std::max(0.015, marker_size_ * 0.8);
    label.color.r = 1.0F;
    label.color.g = 1.0F;
    label.color.b = 1.0F;
    label.color.a = 1.0F;
    label.text = "ArUco " + std::to_string(marker_id);
    array.markers.push_back(label);
  }

  void drawPoseText(
    cv::Mat & image,
    const std::vector<cv::Point2f> & marker_corners,
    int marker_id,
    const cv::Vec3d & translation_vector) const
  {
    if (marker_corners.empty()) {
      return;
    }

    const double distance = cv::norm(translation_vector);

    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(3);
    stream << "ID " << marker_id
           << " x=" << translation_vector[0]
           << " y=" << translation_vector[1]
           << " z=" << translation_vector[2]
           << " d=" << distance << " m";

    const cv::Point origin(
      static_cast<int>(marker_corners.front().x),
      std::max(20, static_cast<int>(marker_corners.front().y) - 10));

    cv::putText(
      image,
      stream.str(),
      origin,
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      cv::Scalar(0, 255, 255),
      1,
      cv::LINE_AA);
  }

  std::string resolveCameraFrame(
    const sensor_msgs::msg::Image & image_message) const
  {
    if (!camera_frame_override_.empty()) {
      return camera_frame_override_;
    }

    if (!image_message.header.frame_id.empty()) {
      return image_message.header.frame_id;
    }

    if (!camera_info_frame_.empty()) {
      return camera_info_frame_;
    }

    return "camera_optical_frame";
  }

  bool convertToGray(
    const cv::Mat & image,
    const std::string & encoding,
    cv::Mat & gray) const
  {
    try {
      if (image.channels() == 1) {
        if (image.depth() == CV_8U) {
          gray = image;
        } else {
          cv::normalize(image, gray, 0, 255, cv::NORM_MINMAX, CV_8U);
        }
        return true;
      }

      if (encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
        return true;
      }

      if (encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::cvtColor(image, gray, cv::COLOR_RGBA2GRAY);
        return true;
      }

      if (encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        return true;
      }

      if (
        encoding == sensor_msgs::image_encodings::BGR8 ||
        image.channels() == 3)
      {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        return true;
      }

      if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        return true;
      }
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to convert image to grayscale: %s",
        e.what());
    }

    return false;
  }

  bool convertToBgr(
    const cv::Mat & image,
    const std::string & encoding,
    cv::Mat & bgr) const
  {
    try {
      if (image.channels() == 1) {
        cv::Mat image_8bit;

        if (image.depth() == CV_8U) {
          image_8bit = image;
        } else {
          cv::normalize(image, image_8bit, 0, 255, cv::NORM_MINMAX, CV_8U);
        }

        cv::cvtColor(image_8bit, bgr, cv::COLOR_GRAY2BGR);
        return true;
      }

      if (encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
        return true;
      }

      if (encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::cvtColor(image, bgr, cv::COLOR_RGBA2BGR);
        return true;
      }

      if (encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
        return true;
      }

      if (image.channels() == 3) {
        bgr = image.clone();
        return true;
      }

      if (image.channels() == 4) {
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
        return true;
      }
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to convert image to BGR: %s",
        e.what());
    }

    return false;
  }

  static std::string idsToString(const std::vector<int32_t> & ids)
  {
    std::ostringstream stream;

    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (i > 0) {
        stream << ", ";
      }
      stream << ids[i];
    }

    return stream.str();
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_override_;
  std::string camera_info_frame_;
  std::string last_image_frame_;
  std::string child_frame_prefix_;

  double marker_size_{0.5};
  double fallback_camera_horizontal_fov_{1.2};
  bool publish_annotated_{true};
  bool publish_fallback_camera_info_{true};
  bool camera_info_ready_{false};

  int calibration_width_{0};
  int calibration_height_{0};
  int fallback_camera_width_{1280};
  int fallback_camera_height_{720};

  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr fallback_camera_info_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ids_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;

  rclcpp::TimerBase::SharedPtr fallback_camera_info_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<ArucoPoseNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("aruco_pose_node"),
      "Node stopped with exception: %s",
      e.what());
  }

  rclcpp::shutdown();
  return 0;
}
