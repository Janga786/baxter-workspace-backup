// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/EndpointState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/endpoint_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_EndpointState_wrench
{
public:
  explicit Init_EndpointState_wrench(::baxter_core_msgs::msg::EndpointState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::EndpointState wrench(::baxter_core_msgs::msg::EndpointState::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndpointState msg_;
};

class Init_EndpointState_twist
{
public:
  explicit Init_EndpointState_twist(::baxter_core_msgs::msg::EndpointState & msg)
  : msg_(msg)
  {}
  Init_EndpointState_wrench twist(::baxter_core_msgs::msg::EndpointState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_EndpointState_wrench(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndpointState msg_;
};

class Init_EndpointState_pose
{
public:
  explicit Init_EndpointState_pose(::baxter_core_msgs::msg::EndpointState & msg)
  : msg_(msg)
  {}
  Init_EndpointState_twist pose(::baxter_core_msgs::msg::EndpointState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_EndpointState_twist(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndpointState msg_;
};

class Init_EndpointState_header
{
public:
  Init_EndpointState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EndpointState_pose header(::baxter_core_msgs::msg::EndpointState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EndpointState_pose(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndpointState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::EndpointState>()
{
  return baxter_core_msgs::msg::builder::Init_EndpointState_header();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATE__BUILDER_HPP_
