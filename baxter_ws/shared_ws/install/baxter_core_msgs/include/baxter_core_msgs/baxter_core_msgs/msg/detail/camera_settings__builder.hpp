// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/CameraSettings.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/camera_settings__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_CameraSettings_controls
{
public:
  explicit Init_CameraSettings_controls(::baxter_core_msgs::msg::CameraSettings & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::CameraSettings controls(::baxter_core_msgs::msg::CameraSettings::_controls_type arg)
  {
    msg_.controls = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::CameraSettings msg_;
};

class Init_CameraSettings_fps
{
public:
  explicit Init_CameraSettings_fps(::baxter_core_msgs::msg::CameraSettings & msg)
  : msg_(msg)
  {}
  Init_CameraSettings_controls fps(::baxter_core_msgs::msg::CameraSettings::_fps_type arg)
  {
    msg_.fps = std::move(arg);
    return Init_CameraSettings_controls(msg_);
  }

private:
  ::baxter_core_msgs::msg::CameraSettings msg_;
};

class Init_CameraSettings_height
{
public:
  explicit Init_CameraSettings_height(::baxter_core_msgs::msg::CameraSettings & msg)
  : msg_(msg)
  {}
  Init_CameraSettings_fps height(::baxter_core_msgs::msg::CameraSettings::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_CameraSettings_fps(msg_);
  }

private:
  ::baxter_core_msgs::msg::CameraSettings msg_;
};

class Init_CameraSettings_width
{
public:
  Init_CameraSettings_width()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CameraSettings_height width(::baxter_core_msgs::msg::CameraSettings::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_CameraSettings_height(msg_);
  }

private:
  ::baxter_core_msgs::msg::CameraSettings msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::CameraSettings>()
{
  return baxter_core_msgs::msg::builder::Init_CameraSettings_width();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__BUILDER_HPP_
