#ifndef INGHT_ROS_DRIVER_ROS_UTILS_H
#define INGHT_ROS_DRIVER_ROS_UTILS_H

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#ifdef USE_ROS1
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

namespace insight9_ros {
using Node = ros::NodeHandle;
using NodePtr = std::shared_ptr<ros::NodeHandle>;
template <typename MsgT>
using Publisher = ros::Publisher;
using Timer = ros::Timer;
using ClockTime = ros::Time;
using Stamp = ros::Time;
using Duration = ros::Duration;
using ImageMsg = sensor_msgs::Image;
using CompressedImageMsg = sensor_msgs::CompressedImage;
using CameraInfoMsg = sensor_msgs::CameraInfo;
using ImuMsg = sensor_msgs::Imu;
using PoseStampedMsg = geometry_msgs::PoseStamped;
using PathMsg = nav_msgs::Path;
using StringMsg = std_msgs::String;

inline bool ok() { return ros::ok(); }
inline void shutdown() { ros::shutdown(); }
inline ClockTime now_time() { return ros::Time::now(); }
inline Stamp now_stamp() { return ros::Time::now(); }
inline ClockTime time_from_us(uint64_t us) { return ros::Time().fromNSec(us * 1000ULL); }
inline Stamp stamp_from_us(uint64_t us) { return ros::Time().fromNSec(us * 1000ULL); }
inline double duration_seconds(const Duration &duration) { return duration.toSec(); }

template <typename MsgT>
inline Publisher<MsgT> create_publisher(Node &node, const std::string &topic, uint32_t queue_size, bool latch = false) {
    return node.advertise<MsgT>(topic, queue_size, latch);
}

template <typename MsgT>
inline void publish(const Publisher<MsgT> &pub, const MsgT &msg) {
    pub.publish(msg);
}

template <typename T>
inline void load_rosparam(Node &node, const std::string &name, T &value, const T &default_value) {
    node.param(name, value, default_value);
}

inline void info(const std::string &msg) { ROS_INFO("%s", msg.c_str()); }
inline void warn(const std::string &msg) { ROS_WARN("%s", msg.c_str()); }
inline void error(const std::string &msg) { ROS_ERROR("%s", msg.c_str()); }
}  // namespace insight9_ros

#elif defined(USE_ROS2)
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

namespace insight9_ros {
using Node = rclcpp::Node;
using NodePtr = rclcpp::Node::SharedPtr;
template <typename MsgT>
using Publisher = typename rclcpp::Publisher<MsgT>::SharedPtr;
using Timer = rclcpp::TimerBase::SharedPtr;
using ClockTime = rclcpp::Time;
using Stamp = builtin_interfaces::msg::Time;
using Duration = rclcpp::Duration;
using ImageMsg = sensor_msgs::msg::Image;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using ImuMsg = sensor_msgs::msg::Imu;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using PathMsg = nav_msgs::msg::Path;
using StringMsg = std_msgs::msg::String;

inline bool ok() { return rclcpp::ok(); }
inline void shutdown() { rclcpp::shutdown(); }
inline ClockTime now_time(const NodePtr &node = nullptr) {
    return node ? node->get_clock()->now() : rclcpp::Clock().now();
}
inline Stamp now_stamp(const NodePtr &node = nullptr) {
    return now_time(node).to_msg();
}
inline ClockTime time_from_us(uint64_t us) { return rclcpp::Time(static_cast<int64_t>(us * 1000ULL)); }
inline Stamp stamp_from_us(uint64_t us) { return time_from_us(us).to_msg(); }
inline double duration_seconds(const Duration &duration) { return duration.seconds(); }

template <typename MsgT>
inline Publisher<MsgT> create_publisher(const NodePtr &node, const std::string &topic, uint32_t queue_size, bool transient_local = false) {
    auto qos = rclcpp::QoS(queue_size);
    if (transient_local) {
        qos.transient_local();
    }
    return node->create_publisher<MsgT>(topic, qos);
}

template <typename MsgT>
inline void publish(const Publisher<MsgT> &pub, const MsgT &msg) {
    pub->publish(msg);
}

template <typename T>
inline void load_rosparam(const NodePtr &node, const std::string &name, T &value, const T &default_value) {
    node->declare_parameter<T>(name, default_value);
    node->get_parameter(name, value);
}

inline void info(const std::string &msg) { RCLCPP_INFO(rclcpp::get_logger("insight_ros_driver"), "%s", msg.c_str()); }
inline void warn(const std::string &msg) { RCLCPP_WARN(rclcpp::get_logger("insight_ros_driver"), "%s", msg.c_str()); }
inline void error(const std::string &msg) { RCLCPP_ERROR(rclcpp::get_logger("insight_ros_driver"), "%s", msg.c_str()); }
}  // namespace insight9_ros

#endif

#endif  // INGHT_ROS_DRIVER_ROS_UTILS_H
