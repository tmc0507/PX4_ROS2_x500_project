// =============================================================================
// ARUCO DETECTOR NODE — Phiên bản tái sử dụng
// =============================================================================
//
// KIẾN TRÚC:
//   ┌─────────────────────────────────────────────────────────┐
//   │  DetectionNode (khung ROS — KHÔNG THAY ĐỔI)           │
//   │  ├── Nhận ảnh từ camera                                │
//   │  ├── Gọi IDetector.detect()                            │
//   │  ├── Gọi IDetector.draw()                              │
//   │  └── Publish kết quả ra ROS topics                     │
//   └─────────────────────────────────────────────────────────┘
//                         │
//                         ▼
//   ┌─────────────────────────────────────────────────────────┐
//   │  IDetector (interface — THAY ĐỔI khi chuyển bài toán)  │
//   │  ├── ArucODetector      → detect marker ArUco          │
//   │  ├── QRCodeDetector     → detect QR code               │
//   │  ├── YOLODetector       → detect object bằng YOLO      │
//   │  └── FaceDetector       → detect khuôn mặt             │
//   └─────────────────────────────────────────────────────────┘
//
// HƯỚNG DẪN THAY ĐỔI:
//   1. Tạo class mới kế thừa IDetector
//   2. Implement 3 method: init(), detect(), draw()
//   3. Trong main(), thay ArucODetector() bằng detector mới
//   4. KHÔNG CẦN SỬA DetectionNode
//
// =============================================================================

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

// =============================================================================
// PHẦN 1: INTERFACE CHO DETECTOR
// =============================================================================
// [THAY ĐỔI] Tạo interface mới nếu bài toán khác hoàn toàn
// [GIỮ NGUYÊN] Nếu bài toán detect tương tự (có bounding box, class ID)
// =============================================================================

/// Kết quả detect — dùng std::variant để hỗ trợ nhiều kiểu kết quả
using DetectionResult = std::variant<
  std::monostate,                          // Không detect được gì
  std::vector<int>,                        // ArUco: danh sách IDs
  std::vector<std::pair<int, float>>,      // Object: (class_id, confidence)
  std::vector<std::vector<cv::Point2f>>    // Segmentation: polygon points
>;

/// Interface cho mọi detector
class IDetector {
public:
  virtual ~IDetector() = default;

  /// Khởi tạo detector từ ROS node (đọc params, load model)
  virtual void init(rclcpp::Node::SharedPtr node) = 0;

  /// Chạy detection trên ảnh, trả về kết quả
  virtual DetectionResult detect(const cv::Mat& image) = 0;

  /// Vẽ kết quả lên ảnh để hiển thị
  virtual void draw(
    cv::Mat& image,
    const DetectionResult& result) = 0;

  /// Trả về tên loại detector (để log)
  virtual std::string name() const = 0;
};

// =============================================================================
// PHẦN 2: ARUCO DETECTOR — Implement cụ thể
// =============================================================================
// [THAY ĐỔI] Class này thay bằng detector mới khi chuyển bài toán
// =============================================================================

class ArucODetector : public IDetector {
public:
  void init(rclcpp::Node::SharedPtr node) override {
    // [THAY ĐỔI #1] Đọc parameters đặc thù ArUco
    int dict_id = node->declare_parameter<int>(
      "dictionary_id", cv::aruco::DICT_4X4_50);

    // [THAY ĐỔI #2] Khởi tạo model/algorithm đặc thù
    try {
      dictionary_ = cv::aruco::getPredefinedDictionary(
        static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dict_id));
    } catch (const cv::Exception& e) {
      RCLCPP_FATAL(
        node->get_logger(),
        "Invalid ArUco dictionary ID %d: %s",
        dict_id, e.what());
      throw;
    }

    detector_params_ = cv::aruco::DetectorParameters::create();
  }

  /// [THAY ĐỔI #3] Thuật toán detection chính
  DetectionResult detect(const cv::Mat& image) override {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::aruco::detectMarkers(
      image, dictionary_, corners, ids,
      detector_params_, rejected);

    // Trả về IDs nếu tìm thấy marker
    if (!ids.empty()) {
      return ids;  // std::vector<int>
    }
    return std::monostate{};  // Không detect được gì
  }

  /// [THAY ĐỔI #4] Logic vẽ kết quả lên ảnh
  void draw(cv::Mat& image, const DetectionResult& result) override {
    const auto* ids = std::get_if<std::vector<int>>(&result);
    if (!ids || ids->empty()) {
      drawText(image, "No markers detected", cv::Scalar(0, 0, 255));
      return;
    }

    // Hiển thị số lượng marker
    drawText(image,
      "Markers: " + std::to_string(ids->size()),
      cv::Scalar(0, 255, 0));

    // Hiển thị danh sách IDs
    std::ostringstream ss;
    for (size_t i = 0; i < ids->size(); ++i) {
      if (i > 0) ss << ", ";
      ss << (*ids)[i];
    }
    drawText(image,
      "IDs: " + ss.str(),
      cv::Scalar(0, 255, 0), cv::Point(15, 60));
  }

  std::string name() const override { return "ArUco"; }

private:
  void drawText(
    cv::Mat& image,
    const std::string& text,
    const cv::Scalar& color,
    const cv::Point& pos = cv::Point(15, 30))
  {
    cv::putText(
      image, text, pos,
      cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv::LINE_AA);
  }

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

// =============================================================================
// PHẦN 3: VÍ DỤ DETECTOR KHÁC — QR Code
// =============================================================================
// [THAY ĐỔI] Uncomment khi muốn dùng QR code detection
// =============================================================================

/*
#include <opencv2/core.hpp>

class QRCodeDetector : public IDetector {
public:
  void init(rclcpp::Node::SharedPtr node) override {
    detector_ = cv::QRCodeDetector::create();
  }

  DetectionResult detect(const cv::Mat& image) override {
    std::vector<cv::Point2f> points;
    cv::Mat straight_qr;

    if (detector_->detectAndDecode(image, points, straight_qr)) {
      // Trả về polygon points của QR code
      return std::vector<std::vector<cv::Point2f>>{points};
    }
    return std::monostate{};
  }

  void draw(cv::Mat& image, const DetectionResult& result) override {
    const auto* polygons =
      std::get_if<std::vector<std::vector<cv::Point2f>>>(&result);
    if (!polygons || polygons->empty()) return;

    for (const auto& pts : *polygons) {
      cv::polylines(image, pts, true,
        cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
  }

  std::string name() const override { return "QRCode"; }

private:
  cv::Ptr<cv::QRCodeDetector> detector_;
};
*/

// =============================================================================
// PHẦN 4: VÍ DỤ DETECTOR KHÁC — YOLO Object Detection
// =============================================================================
// [THAY ĐỔI] Uncomment khi muốn dùng YOLO
// =============================================================================

/*
#include <opencv2/dnn.hpp>

class YOLODetector : public IDetector {
public:
  void init(rclcpp::Node::SharedPtr node) override {
    std::string model_path = node->declare_parameter<std::string>(
      "model_path", "yolov8n.onnx");

    net_ = cv::dnn::readNetFromONNX(model_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    conf_threshold_ = node->declare_parameter<float>("conf_threshold", 0.5f);
  }

  DetectionResult detect(const cv::Mat& image) override {
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1./255.,
      cv::Size(640, 640), cv::Scalar(), true, false);
    net_.setInput(blob);

    cv::Mat output = net_.forward();
    // ... parse output, extract bboxes, class_ids, confidences

    std::vector<std::pair<int, float>> detections;
    // detections.push_back({class_id, confidence});
    return detections;
  }

  void draw(cv::Mat& image, const DetectionResult& result) override {
    const auto* dets =
      std::get_if<std::vector<std::pair<int, float>>>(&result);
    if (!dets || dets->empty()) return;

    for (const auto& [cls, conf] : *dets) {
      // cv::rectangle(image, bbox, color);
      // cv::putText(image, class_name + conf, ...);
    }
  }

  std::string name() const override { return "YOLO"; }

private:
  cv::dnn::Net net_;
  float conf_threshold_;
};
*/

// =============================================================================
// PHẦN 5: NODE CHUNG — KHÔNG THAY ĐỔI
// =============================================================================
// [GIỮ NGUYÊN] Phần này hoạt động với mọi loại detector
// Chỉ cần thay detector trong main() là node tự chạy đúng
// =============================================================================

class DetectionNode : public rclcpp::Node {
public:
  explicit DetectionNode(std::unique_ptr<IDetector> detector)
  : Node("detection_node"), detector_(std::move(detector))
  {
    // ---- Đọc parameters chung (giữ nguyên cho mọi bài toán) ----
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/camera/image_raw");

    publish_annotated_ = declare_parameter<bool>(
      "publish_annotated", true);

    // ---- Khởi tạo detector đặc thù ----
    detector_->init(this->shared_from_this());

    // ---- Tạo ROS publishers (giữ nguyên) ----
    result_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "detection/result", 10);

    if (publish_annotated_) {
      annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "detection/image_marked",
        rclcpp::SensorDataQoS());
    }

    // ---- Tạo ROS subscriber (giữ nguyên) ----
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DetectionNode::imageCallback, this, std::placeholders::_1));

    // ---- Log khởi động (giữ nguyên) ----
    RCLCPP_INFO(get_logger(), "%s detector started",
      detector_->name().c_str());
    RCLCPP_INFO(get_logger(), "Input topic: %s",
      image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Result topic: %s",
      result_pub_->get_topic_name().c_str());
  }

private:
  // =========================================================================
  // imageCallback — KHÔNG THAY ĐỔI
  // Luồng xử lý: ROS Image → OpenCV Mat → Preprocess → Detect → Draw → Pub
  // =========================================================================
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    // Bước 1: Chuyển ROS message sang OpenCV Mat (giữ nguyên)
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN(get_logger(), "cv_bridge failed: %s", e.what());
      return;
    }

    const cv::Mat& image = cv_ptr->image;
    if (image.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Empty image");
      return;
    }

    // Bước 2: Preprocess — chuyển sang grayscale (giữ nguyên)
    cv::Mat gray;
    if (!convertToGray(image, msg->encoding, gray)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported encoding: %s", msg->encoding.c_str());
      return;
    }

    // Bước 3: Chạy detection — GỌI INTERFACE
    // [THAY ĐỔI] Detector được inject qua constructor,
    //             không cần sửa code ở đây
    DetectionResult result;
    try {
      result = detector_->detect(gray);
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s detection failed: %s",
        detector_->name().c_str(), e.what());
      return;
    }

    // Bước 4: Publish kết quả (giữ nguyên)
    publishResult(result);

    // Bước 5: Vẽ và publish ảnh annotated
    if (publish_annotated_) {
      publishAnnotatedImage(msg, image, result);
    }
  }

  // =========================================================================
  // convertToGray — GIỮ NGUYÊN 100%
  // Hỗ trợ mọi định dạng ảnh RGB, BGR, RGBA, BGRA, grayscale
  // =========================================================================
  bool convertToGray(
    const cv::Mat& image,
    const std::string& encoding,
    cv::Mat& gray)
  {
    try {
      // Ảnh đã là 1 kênh → dùng trực tiếp hoặc normalize
      if (image.channels() == 1) {
        if (image.depth() == CV_8U) {
          gray = image;
        } else {
          cv::normalize(image, gray, 0, 255,
            cv::NORM_MINMAX, CV_8U);
        }
        return true;
      }

      // RGB → Gray
      if (encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
        return true;
      }

      // RGBA → Gray
      if (encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::cvtColor(image, gray, cv::COLOR_RGBA2GRAY);
        return true;
      }

      // BGRA → Gray
      if (encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        return true;
      }

      // BGR hoặc 3 kênh không rõ → Gray
      if (encoding == sensor_msgs::image_encodings::BGR8 ||
          image.channels() == 3)
      {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        return true;
      }

      // 4 kênh không rõ → Gray
      if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        return true;
      }
    } catch (const cv::Exception& e) {
      RCLCPP_WARN(get_logger(),
        "Grayscale conversion failed: %s", e.what());
    }
    return false;
  }

  // =========================================================================
  // publishResult — GIỮ NGUYÊN (chỉ đổi message type nếu cần)
  // =========================================================================
  void publishResult(const DetectionResult& result) {
    std_msgs::msg::Int32MultiArray msg;

    // Trích xuất IDs từ variant (nếu là ArUco)
    if (const auto* ids = std::get_if<std::vector<int>>(&result)) {
      msg.data = *ids;
    }

    result_pub_->publish(msg);
  }

  // =========================================================================
  // publishAnnotatedImage — GIỮ NGUYÊN, gọi detector_->draw()
  // =========================================================================
  void publishAnnotatedImage(
    const sensor_msgs::msg::Image::ConstSharedPtr msg,
    const cv::Mat& image,
    const DetectionResult& result)
  {
    try {
      cv::Mat annotated;

      // Chuyển sang BGR để vẽ màu đúng (giữ nguyên)
      if (!convertToBGR(image, msg->encoding, annotated)) {
        return;
      }

      // Gọi draw() của detector — [THAY ĐỔI qua interface]
      detector_->draw(annotated, result);

      // Convert sang ROS message và publish (giữ nguyên)
      cv_bridge::CvImage output;
      output.header = msg->header;
      output.encoding = sensor_msgs::image_encodings::BGR8;
      output.image = annotated;
      annotated_pub_->publish(*output.toImageMsg());

    } catch (const cv::Exception& e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Annotated image failed: %s", e.what());
    }
  }

  // =========================================================================
  // convertToBGR — GIỮ NGUYÊN 100%
  // Chuyển mọi định dạng ảnh sang BGR để vẽ màu đúng
  // =========================================================================
  bool convertToBGR(
    const cv::Mat& image,
    const std::string& encoding,
    cv::Mat& bgr)
  {
    try {
      if (image.channels() == 1) {
        cv::Mat img8;
        if (image.depth() == CV_8U) {
          img8 = image;
        } else {
          cv::normalize(image, img8, 0, 255,
            cv::NORM_MINMAX, CV_8U);
        }
        cv::cvtColor(img8, bgr, cv::COLOR_GRAY2BGR);
      } else if (encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
      } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::cvtColor(image, bgr, cv::COLOR_RGBA2BGR);
      } else if (encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
      } else if (image.channels() == 3) {
        bgr = image.clone();
      } else if (image.channels() == 4) {
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
      } else {
        return false;
      }
      return true;
    } catch (const cv::Exception& e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "BGR conversion failed: %s", e.what());
    }
    return false;
  }

  // ---- Biến thành viên — GIỮ NGUYÊN ----
  std::string image_topic_;
  bool publish_annotated_{true};
  std::unique_ptr<IDetector> detector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr result_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
};

// =============================================================================
// MAIN — CHỈ CẦN THAY DETECTOR TẠI ĐÂY
// =============================================================================
// [THAY ĐỔI] Đổi `ArucODetector()` thành detector mới
// Ví dụ: std::make_unique<QRCodeDetector>()
//         std::make_unique<YOLODetector>()
// =============================================================================

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    // ---- DÒNG CẦN THAY ĐỔI KHI CHUYỂN BÀI TOÁN ----
    auto detector = std::make_unique<ArucODetector>();

    rclcpp::spin(std::make_shared<DetectionNode>(std::move(detector)));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("detection"),
      "Node stopped: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
