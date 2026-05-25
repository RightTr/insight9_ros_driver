#include "ros_utils.h"
#include "Insight_9_receive.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>

using namespace insight9_ros;

class Insight9RosDriver {
public:
#ifdef USE_ROS1
  Insight9RosDriver() : nh_(), pnh_("~"), has_vio_pose_(false) {
    init();
  }
#elif defined(USE_ROS2)
  explicit Insight9RosDriver(const NodePtr &node) : node_(node), has_vio_pose_(false) {
    init();
  }
#endif

  ~Insight9RosDriver() {
    shutdownSdk();
  }

private:
  struct CachedVio {
    PoseStampedMsg pose;
    ClockTime received_time;
  };

  void init() {
    loadParams();
    initPublishers();
    initSdk();
    startTimers();
  }

  void loadParams() {
#ifdef USE_ROS1
    load_rosparam(pnh_, "imu_frame_id", imu_frame_id_, std::string("camera_imu"));
    load_rosparam(pnh_, "infra1_frame_id", infra1_frame_id_, std::string("camera_infra1"));
    load_rosparam(pnh_, "infra2_frame_id", infra2_frame_id_, std::string("camera_infra2"));
    load_rosparam(pnh_, "color_frame_id", color_frame_id_, std::string("camera_color"));
    load_rosparam(pnh_, "depth_frame_id", depth_frame_id_, std::string("camera_depth"));
    load_rosparam(pnh_, "vio_frame_id", vio_frame_id_, std::string("insght_vio"));
    load_rosparam(pnh_, "vio_path_frame_id", vio_path_frame_id_, std::string("insght_vio"));
#elif defined(USE_ROS2)
    load_rosparam(node_, "imu_frame_id", imu_frame_id_, std::string("camera_imu"));
    load_rosparam(node_, "infra1_frame_id", infra1_frame_id_, std::string("camera_infra1"));
    load_rosparam(node_, "infra2_frame_id", infra2_frame_id_, std::string("camera_infra2"));
    load_rosparam(node_, "color_frame_id", color_frame_id_, std::string("camera_color"));
    load_rosparam(node_, "depth_frame_id", depth_frame_id_, std::string("camera_depth"));
    load_rosparam(node_, "vio_frame_id", vio_frame_id_, std::string("insght_vio"));
    load_rosparam(node_, "vio_path_frame_id", vio_path_frame_id_, std::string("insght_vio"));
#endif
  }

  void initPublishers() {
#ifdef USE_ROS1
    imu_pub_ = create_publisher<ImuMsg>(nh_, "/camera/camera/imu", 100);
    infra1_pub_ = create_publisher<ImageMsg>(nh_, "/camera/camera/infra1/image_rect_raw", 10);
    infra2_pub_ = create_publisher<ImageMsg>(nh_, "/camera/camera/infra2/image_rect_raw", 10);
    color_pub_ = create_publisher<CompressedImageMsg>(nh_, "/camera/camera/color/image_rect_raw/compressed", 10);
    depth_pub_ = create_publisher<ImageMsg>(nh_, "/camera/camera/depth/image_rect_raw", 10);
    infra1_info_pub_ = create_publisher<CameraInfoMsg>(nh_, "/camera/camera/infra1/camera_info", 1, true);
    infra2_info_pub_ = create_publisher<CameraInfoMsg>(nh_, "/camera/camera/infra2/camera_info", 1, true);
    color_info_pub_ = create_publisher<CameraInfoMsg>(nh_, "/camera/camera/color/camera_info", 1, true);
    vio_20_pub_ = create_publisher<PoseStampedMsg>(nh_, "/insight/vio_20hz", 10);
    vio_100_pub_ = create_publisher<PoseStampedMsg>(nh_, "/insight/vio_100hz", 10);
    vio_status_pub_ = create_publisher<StringMsg>(nh_, "/insight/vio_status", 1, true);
    vio_path_pub_ = create_publisher<PathMsg>(nh_, "/insight/vio_path", 1, true);
#elif defined(USE_ROS2)
    imu_pub_ = create_publisher<ImuMsg>(node_, "/camera/camera/imu", 100);
    infra1_pub_ = create_publisher<ImageMsg>(node_, "/camera/camera/infra1/image_rect_raw", 10);
    infra2_pub_ = create_publisher<ImageMsg>(node_, "/camera/camera/infra2/image_rect_raw", 10);
    color_pub_ = create_publisher<CompressedImageMsg>(node_, "/camera/camera/color/image_rect_raw/compressed", 10);
    depth_pub_ = create_publisher<ImageMsg>(node_, "/camera/camera/depth/image_rect_raw", 10);
    infra1_info_pub_ = create_publisher<CameraInfoMsg>(node_, "/camera/camera/infra1/camera_info", 1, true);
    infra2_info_pub_ = create_publisher<CameraInfoMsg>(node_, "/camera/camera/infra2/camera_info", 1, true);
    color_info_pub_ = create_publisher<CameraInfoMsg>(node_, "/camera/camera/color/camera_info", 1, true);
    vio_20_pub_ = create_publisher<PoseStampedMsg>(node_, "/insight/vio_20hz", 10);
    vio_100_pub_ = create_publisher<PoseStampedMsg>(node_, "/insight/vio_100hz", 10);
    vio_status_pub_ = create_publisher<StringMsg>(node_, "/insight/vio_status", 1, true);
    vio_path_pub_ = create_publisher<PathMsg>(node_, "/insight/vio_path", 1, true);
#endif
  }

  void initSdk() {
    if (insight9_receive_init() != 0) {
      throw std::runtime_error("insight9_receive_init failed");
    }
    insight9_receive_register_image_callback(&Insight9RosDriver::imageCbThunk, this);
    insight9_receive_register_imu_callback(&Insight9RosDriver::imuCbThunk, this);
    insight9_receive_register_vio_callback(&Insight9RosDriver::vioCbThunk, this);
    if (insight9_receive_start() != 0) {
      insight9_receive_cleanup();
      throw std::runtime_error("insight9_receive_start failed");
    }
  }

  void shutdownSdk() {
    try {
      insight9_receive_stop();
      insight9_receive_cleanup();
    } catch (...) {
    }
  }

  void startTimers() {
#ifdef USE_ROS1
    vio_20_timer_ = nh_.createTimer(ros::Duration(0.05), &Insight9RosDriver::onVio20Timer, this);
    vio_100_timer_ = nh_.createTimer(ros::Duration(0.01), &Insight9RosDriver::onVio100Timer, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0), &Insight9RosDriver::onStatusTimer, this);
#elif defined(USE_ROS2)
    vio_20_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Insight9RosDriver::onVio20Timer, this));
    vio_100_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Insight9RosDriver::onVio100Timer, this));
    status_timer_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&Insight9RosDriver::onStatusTimer, this));
#endif
  }

  static void imageCbThunk(int cam_id, uint8_t *data, size_t size,
                           int width, int height, unsigned int format,
                           uint64_t timestamp, uint64_t right_timestamp,
                           void *userdata) {
    auto *self = static_cast<Insight9RosDriver *>(userdata);
    if (self) {
      self->handleImage(cam_id, data, size, width, height, format, timestamp, right_timestamp);
    }
  }

  static void imuCbThunk(float ax, float ay, float az,
                         float gx, float gy, float gz,
                         uint64_t timestamp, void *userdata) {
    auto *self = static_cast<Insight9RosDriver *>(userdata);
    if (self) {
      self->handleImu(ax, ay, az, gx, gy, gz, timestamp);
    }
  }

  static void vioCbThunk(float px, float py, float pz,
                         float qx, float qy, float qz, float qw,
                         uint64_t timestamp, void *userdata) {
    auto *self = static_cast<Insight9RosDriver *>(userdata);
    if (self) {
      self->handleVio(px, py, pz, qx, qy, qz, qw, timestamp);
    }
  }

  static CameraInfoMsg makeCameraInfo(const std::string &frame_id, uint32_t width, uint32_t height) {
    CameraInfoMsg info;
    info.header.frame_id = frame_id;
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    info.D.clear();
    for (size_t i = 0; i < info.K.size(); ++i) {
      info.K[i] = 0.0;
    }
    for (size_t i = 0; i < info.R.size(); ++i) {
      info.R[i] = 0.0;
    }
    for (size_t i = 0; i < info.P.size(); ++i) {
      info.P[i] = 0.0;
    }
    info.K[0] = 1.0;
    info.K[4] = 1.0;
    info.K[8] = 1.0;
    info.R[0] = 1.0;
    info.R[4] = 1.0;
    info.R[8] = 1.0;
    info.P[0] = 1.0;
    info.P[5] = 1.0;
    info.P[10] = 1.0;
    return info;
  }

  Stamp stampFromSdk(uint64_t ts_us) const {
    return ts_us ? stamp_from_us(ts_us) : currentStamp();
  }

  ClockTime timeFromSdk(uint64_t ts_us) const {
    return ts_us ? time_from_us(ts_us) : currentTime();
  }

  Stamp currentStamp() const {
#ifdef USE_ROS1
    return now_stamp();
#elif defined(USE_ROS2)
    return now_stamp(node_);
#endif
  }

  ClockTime currentTime() const {
#ifdef USE_ROS1
    return now_time();
#elif defined(USE_ROS2)
    return now_time(node_);
#endif
  }

  void handleImage(int cam_id, uint8_t *data, size_t size,
                   int width, int height, unsigned int format,
                   uint64_t timestamp, uint64_t right_timestamp) {
    (void)format;
    const Stamp stamp = stampFromSdk(timestamp);

    if (cam_id == 0) {
      CompressedImageMsg msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = color_frame_id_;
      msg.format = "jpeg";
      msg.data.assign(data, data + size);
      publish(color_pub_, msg);

      CameraInfoMsg info = color_info_;
      info.header.stamp = stamp;
      info.header.frame_id = color_frame_id_;
      publish(color_info_pub_, info);
      return;
    }

    if (cam_id == 1) {
      const int mono_height = (height > 0 ? (height - 1) / 2 : 0);
      if (mono_height <= 0) {
        return;
      }
      const size_t mono_bytes = static_cast<size_t>(width) * static_cast<size_t>(mono_height);
      if (size < mono_bytes * 2) {
        return;
      }

      ImageMsg left_msg;
      left_msg.header.stamp = stamp;
      left_msg.header.frame_id = infra1_frame_id_;
      left_msg.height = mono_height;
      left_msg.width = width;
      left_msg.encoding = "mono8";
      left_msg.is_bigendian = false;
      left_msg.step = width;
      left_msg.data.assign(data, data + mono_bytes);
      publish(infra1_pub_, left_msg);

      ImageMsg right_msg;
      right_msg.header.stamp = stampFromSdk(right_timestamp);
      right_msg.header.frame_id = infra2_frame_id_;
      right_msg.height = mono_height;
      right_msg.width = width;
      right_msg.encoding = "mono8";
      right_msg.is_bigendian = false;
      right_msg.step = width;
      right_msg.data.assign(data + mono_bytes, data + mono_bytes * 2);
      publish(infra2_pub_, right_msg);

      CameraInfoMsg left_info = infra1_info_;
      left_info.header.stamp = left_msg.header.stamp;
      left_info.header.frame_id = infra1_frame_id_;
      left_info.width = width;
      left_info.height = mono_height;
      publish(infra1_info_pub_, left_info);

      CameraInfoMsg right_info = infra2_info_;
      right_info.header.stamp = right_msg.header.stamp;
      right_info.header.frame_id = infra2_frame_id_;
      right_info.width = width;
      right_info.height = mono_height;
      publish(infra2_info_pub_, right_info);
      return;
    }

    if (cam_id == 2) {
      const int depth_height = (height > 1 ? height - 2 : 0);
      if (depth_height <= 0) {
        return;
      }
      const size_t depth_bytes = static_cast<size_t>(width) * static_cast<size_t>(depth_height) * 2U;
      if (size < depth_bytes) {
        return;
      }

      ImageMsg msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = depth_frame_id_;
      msg.height = depth_height;
      msg.width = width;
      msg.encoding = "16UC1";
      msg.is_bigendian = false;
      msg.step = width * 2;
      msg.data.assign(data, data + depth_bytes);
      publish(depth_pub_, msg);
    }
  }

  void handleImu(float ax, float ay, float az,
                 float gx, float gy, float gz,
                 uint64_t timestamp) {
    ImuMsg msg;
    msg.header.stamp = stampFromSdk(timestamp);
    msg.header.frame_id = imu_frame_id_;
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;
    msg.orientation_covariance[0] = -1.0;
    publish(imu_pub_, msg);
  }

  void handleVio(float px, float py, float pz,
                 float qx, float qy, float qz, float qw,
                 uint64_t timestamp) {
    PoseStampedMsg pose;
    pose.header.stamp = stampFromSdk(timestamp);
    pose.header.frame_id = vio_frame_id_;
    pose.pose.position.x = px;
    pose.pose.position.y = py;
    pose.pose.position.z = pz;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      cached_vio_.pose = pose;
      cached_vio_.received_time = currentTime();
      has_vio_pose_ = true;

      path_.header.frame_id = vio_path_frame_id_;
      path_.header.stamp = pose.header.stamp;
      path_.poses.push_back(pose);
      publish(vio_path_pub_, path_);
    }
  }

  void publishCachedVio(const Publisher<PoseStampedMsg> &pub) {
    PoseStampedMsg pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_vio_pose_) {
        return;
      }
      pose = cached_vio_.pose;
    }
    publish(pub, pose);
  }

  void publishVioStatus() {
    StringMsg msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_vio_pose_) {
        msg.data = "waiting_for_vio";
      } else {
        const auto age = currentTime() - cached_vio_.received_time;
        msg.data = (duration_seconds(age) > 1.0) ? "stale" : "tracking";
      }
    }
    publish(vio_status_pub_, msg);
  }

#ifdef USE_ROS1
  void onVio20Timer(const ros::TimerEvent &) { publishCachedVio(vio_20_pub_); }
  void onVio100Timer(const ros::TimerEvent &) { publishCachedVio(vio_100_pub_); }
  void onStatusTimer(const ros::TimerEvent &) { publishVioStatus(); }
#elif defined(USE_ROS2)
  void onVio20Timer() { publishCachedVio(vio_20_pub_); }
  void onVio100Timer() { publishCachedVio(vio_100_pub_); }
  void onStatusTimer() { publishVioStatus(); }
#endif

#ifdef USE_ROS1
  Node nh_;
  Node pnh_;
#elif defined(USE_ROS2)
  NodePtr node_;
#endif

  Publisher<ImuMsg> imu_pub_;
  Publisher<ImageMsg> infra1_pub_;
  Publisher<ImageMsg> infra2_pub_;
  Publisher<CompressedImageMsg> color_pub_;
  Publisher<ImageMsg> depth_pub_;
  Publisher<CameraInfoMsg> infra1_info_pub_;
  Publisher<CameraInfoMsg> infra2_info_pub_;
  Publisher<CameraInfoMsg> color_info_pub_;
  Publisher<PoseStampedMsg> vio_20_pub_;
  Publisher<PoseStampedMsg> vio_100_pub_;
  Publisher<StringMsg> vio_status_pub_;
  Publisher<PathMsg> vio_path_pub_;
  Timer vio_20_timer_;
  Timer vio_100_timer_;
  Timer status_timer_;

  std::string imu_frame_id_;
  std::string infra1_frame_id_;
  std::string infra2_frame_id_;
  std::string color_frame_id_;
  std::string depth_frame_id_;
  std::string vio_frame_id_;
  std::string vio_path_frame_id_;

  CameraInfoMsg infra1_info_ = makeCameraInfo("camera_infra1", 544, 640);
  CameraInfoMsg infra2_info_ = makeCameraInfo("camera_infra2", 544, 640);
  CameraInfoMsg color_info_ = makeCameraInfo("camera_color", 1088, 1920);

  std::mutex mutex_;
  CachedVio cached_vio_;
  PathMsg path_;
  bool has_vio_pose_;
};

int main(int argc, char **argv) {
#ifdef USE_ROS1
  ros::init(argc, argv, "insight9_ros_driver");
  try {
    Insight9RosDriver driver;
    ros::spin();
  } catch (const std::exception &e) {
    ROS_ERROR("%s", e.what());
    return 1;
  }
  return 0;
#elif defined(USE_ROS2)
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("insight9_ros_driver");
    Insight9RosDriver driver(node);
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("inght_ros_driver"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
#endif
}
