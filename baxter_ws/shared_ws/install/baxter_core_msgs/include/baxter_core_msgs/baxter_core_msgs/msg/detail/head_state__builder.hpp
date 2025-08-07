// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/HeadState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/head_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_HeadState_is_pan_enabled
{
public:
  explicit Init_HeadState_is_pan_enabled(::baxter_core_msgs::msg::HeadState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::HeadState is_pan_enabled(::baxter_core_msgs::msg::HeadState::_is_pan_enabled_type arg)
  {
    msg_.is_pan_enabled = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::HeadState msg_;
};

class Init_HeadState_is_nodding
{
public:
  explicit Init_HeadState_is_nodding(::baxter_core_msgs::msg::HeadState & msg)
  : msg_(msg)
  {}
  Init_HeadState_is_pan_enabled is_nodding(::baxter_core_msgs::msg::HeadState::_is_nodding_type arg)
  {
    msg_.is_nodding = std::move(arg);
    return Init_HeadState_is_pan_enabled(msg_);
  }

private:
  ::baxter_core_msgs::msg::HeadState msg_;
};

class Init_HeadState_is_turning
{
public:
  explicit Init_HeadState_is_turning(::baxter_core_msgs::msg::HeadState & msg)
  : msg_(msg)
  {}
  Init_HeadState_is_nodding is_turning(::baxter_core_msgs::msg::HeadState::_is_turning_type arg)
  {
    msg_.is_turning = std::move(arg);
    return Init_HeadState_is_nodding(msg_);
  }

private:
  ::baxter_core_msgs::msg::HeadState msg_;
};

class Init_HeadState_pan
{
public:
  Init_HeadState_pan()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeadState_is_turning pan(::baxter_core_msgs::msg::HeadState::_pan_type arg)
  {
    msg_.pan = std::move(arg);
    return Init_HeadState_is_turning(msg_);
  }

private:
  ::baxter_core_msgs::msg::HeadState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::HeadState>()
{
  return baxter_core_msgs::msg::builder::Init_HeadState_pan();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__BUILDER_HPP_
