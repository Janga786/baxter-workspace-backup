// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/CameraControl.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_CONTROL__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/camera_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_CameraControl_value
{
public:
  explicit Init_CameraControl_value(::baxter_core_msgs::msg::CameraControl & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::CameraControl value(::baxter_core_msgs::msg::CameraControl::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::CameraControl msg_;
};

class Init_CameraControl_id
{
public:
  Init_CameraControl_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CameraControl_value id(::baxter_core_msgs::msg::CameraControl::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_CameraControl_value(msg_);
  }

private:
  ::baxter_core_msgs::msg::CameraControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::CameraControl>()
{
  return baxter_core_msgs::msg::builder::Init_CameraControl_id();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_CONTROL__BUILDER_HPP_
