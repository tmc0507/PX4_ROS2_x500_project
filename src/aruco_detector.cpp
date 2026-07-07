#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector")
  {
    const std::string image_topic =
      declare_parameter<std::string>(
        "image_topic",
        "/camera/image_raw");

    const int dictionary_id =
      declare_parameter<int>(
        "dictionary_id",
        cv::aruco::DICT_4X4_50);

    publish_annotated_ =
      declare_parameter<bool>(
        "publish_annotated",
        true);

    try {
      dictionary_ =
        cv::aruco::getPredefinedDictionary(
        static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(
          dictionary_id));
    } catch (const cv::Exception & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid ArUco dictionary ID %d: %s",
        dictionary_id,
        e.what());

      throw;
    }

    detector_parameters_ =
      cv::aruco::DetectorParameters::create();

    ids_pub_ =
      create_publisher<std_msgs::msg::Int32MultiArray>(
        "aruco/ids",
        10);

    if (publish_annotated_) {
      annotated_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
          "aruco/image_marked",
          rclcpp::SensorDataQoS());
    }

    image_sub_ =
      create_subscription<sensor_msgs::msg::Image>(
        image_topic,
        rclcpp::SensorDataQoS(),
        std::bind(
          &ArucoDetectorNode::imageCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "ArUco detector started");

    RCLCPP_INFO(
      get_logger(),
      "Input image topic: %s",
      image_topic.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Marker ID topic: %s",
      ids_pub_->get_topic_name());

    if (publish_annotated_) {
      RCLCPP_INFO(
        get_logger(),
        "Annotated image topic: %s",
        annotated_pub_->get_topic_name());
    }
  }

private:
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
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

    const cv::Mat & image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received empty image");

      return;
    }

    cv::Mat gray;

    if (!convertToGray(
        image,
        msg->encoding,
        gray))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Unsupported image encoding: %s, channels: %d",
        msg->encoding.c_str(),
        image.channels());

      return;
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

    publishIds(ids);

    if (!ids.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Detected ArUco IDs: %s",
        idsToString(ids).c_str());
    }

    if (publish_annotated_) {
      publishAnnotatedImage(
        msg,
        image,
        corners,
        ids);
    }
  }

  bool convertToGray(
    const cv::Mat & image,
    const std::string & encoding,
    cv::Mat & gray)
  {
    try {
      if (image.channels() == 1) {
        if (image.depth() == CV_8U) {
          gray = image;
        } else {
          cv::normalize(
            image,
            gray,
            0,
            255,
            cv::NORM_MINMAX,
            CV_8U);
        }

        return true;
      }

      if (encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(
          image,
          gray,
          cv::COLOR_RGB2GRAY);

        return true;
      }

      if (encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::cvtColor(
          image,
          gray,
          cv::COLOR_RGBA2GRAY);

        return true;
      }

      if (encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::cvtColor(
          image,
          gray,
          cv::COLOR_BGRA2GRAY);

        return true;
      }

      if (
        encoding == sensor_msgs::image_encodings::BGR8 ||
        image.channels() == 3)
      {
        cv::cvtColor(
          image,
          gray,
          cv::COLOR_BGR2GRAY);

        return true;
      }

      if (image.channels() == 4) {
        cv::cvtColor(
          image,
          gray,
          cv::COLOR_BGRA2GRAY);

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

  void publishIds(const std::vector<int> & ids)
  {
    std_msgs::msg::Int32MultiArray ids_msg;
    ids_msg.data = ids;

    ids_pub_->publish(ids_msg);
  }

  void publishAnnotatedImage(
    const sensor_msgs::msg::Image::ConstSharedPtr msg,
    const cv::Mat & image,
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids)
  {
    cv::Mat annotated;

    try {
      if (image.channels() == 1) {
        cv::Mat image_8bit;

        if (image.depth() == CV_8U) {
          image_8bit = image;
        } else {
          cv::normalize(
            image,
            image_8bit,
            0,
            255,
            cv::NORM_MINMAX,
            CV_8U);
        }

        cv::cvtColor(
          image_8bit,
          annotated,
          cv::COLOR_GRAY2BGR);
      } else if (
        msg->encoding ==
        sensor_msgs::image_encodings::RGB8)
      {
        cv::cvtColor(
          image,
          annotated,
          cv::COLOR_RGB2BGR);
      } else if (
        msg->encoding ==
        sensor_msgs::image_encodings::RGBA8)
      {
        cv::cvtColor(
          image,
          annotated,
          cv::COLOR_RGBA2BGR);
      } else if (
        msg->encoding ==
        sensor_msgs::image_encodings::BGRA8)
      {
        cv::cvtColor(
          image,
          annotated,
          cv::COLOR_BGRA2BGR);
      } else if (image.channels() == 3) {
        annotated = image.clone();
      } else if (image.channels() == 4) {
        cv::cvtColor(
          image,
          annotated,
          cv::COLOR_BGRA2BGR);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Cannot create annotated image");

        return;
      }

      if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(
          annotated,
          corners,
          ids);
      }

      drawDetectionInformation(
        annotated,
        ids);

      cv_bridge::CvImage output;

      output.header = msg->header;
      output.encoding =
        sensor_msgs::image_encodings::BGR8;

      output.image = annotated;

      annotated_pub_->publish(
        *output.toImageMsg());
    } catch (const cv::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to publish annotated image: %s",
        e.what());
    }
  }

  void drawDetectionInformation(
    cv::Mat & image,
    const std::vector<int> & ids)
  {
    const std::string marker_count =
      "Markers: " + std::to_string(ids.size());

    const std::string ids_text =
      ids.empty()
      ? "IDs: none"
      : "IDs: " + idsToString(ids);

    cv::putText(
      image,
      marker_count,
      cv::Point(15, 30),
      cv::FONT_HERSHEY_SIMPLEX,
      0.7,
      cv::Scalar(0, 255, 0),
      2,
      cv::LINE_AA);

    cv::putText(
      image,
      ids_text,
      cv::Point(15, 60),
      cv::FONT_HERSHEY_SIMPLEX,
      0.7,
      ids.empty()
        ? cv::Scalar(0, 0, 255)
        : cv::Scalar(0, 255, 0),
      2,
      cv::LINE_AA);
  }

  static std::string idsToString(
    const std::vector<int> & ids)
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

  bool publish_annotated_{true};

  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  cv::Ptr<cv::aruco::DetectorParameters>
  detector_parameters_;

  rclcpp::Subscription<
    sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Int32MultiArray>::SharedPtr ids_pub_;

  rclcpp::Publisher<
    sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<ArucoDetectorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("aruco_detector"),
      "Node stopped with exception: %s",
      e.what());
  }

  rclcpp::shutdown();

  return 0;
}