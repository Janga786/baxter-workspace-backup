// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/AssemblyState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/assembly_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_AssemblyState_estop_source
{
public:
  explicit Init_AssemblyState_estop_source(::baxter_core_msgs::msg::AssemblyState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::AssemblyState estop_source(::baxter_core_msgs::msg::AssemblyState::_estop_source_type arg)
  {
    msg_.estop_source = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::AssemblyState msg_;
};

class Init_AssemblyState_estop_button
{
public:
  explicit Init_AssemblyState_estop_button(::baxter_core_msgs::msg::AssemblyState & msg)
  : msg_(msg)
  {}
  Init_AssemblyState_estop_source estop_button(::baxter_core_msgs::msg::AssemblyState::_estop_button_type arg)
  {
    msg_.estop_button = std::move(arg);
    return Init_AssemblyState_estop_source(msg_);
  }

private:
  ::baxter_core_msgs::msg::AssemblyState msg_;
};

class Init_AssemblyState_error
{
public:
  explicit Init_AssemblyState_error(::baxter_core_msgs::msg::AssemblyState & msg)
  : msg_(msg)
  {}
  Init_AssemblyState_estop_button error(::baxter_core_msgs::msg::AssemblyState::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_AssemblyState_estop_button(msg_);
  }

private:
  ::baxter_core_msgs::msg::AssemblyState msg_;
};

class Init_AssemblyState_stopped
{
public:
  explicit Init_AssemblyState_stopped(::baxter_core_msgs::msg::AssemblyState & msg)
  : msg_(msg)
  {}
  Init_AssemblyState_error stopped(::baxter_core_msgs::msg::AssemblyState::_stopped_type arg)
  {
    msg_.stopped = std::move(arg);
    return Init_AssemblyState_error(msg_);
  }

private:
  ::baxter_core_msgs::msg::AssemblyState msg_;
};

class Init_AssemblyState_enabled
{
public:
  explicit Init_AssemblyState_enabled(::baxter_core_msgs::msg::AssemblyState & msg)
  : msg_(msg)
  {}
  Init_AssemblyState_stopped enabled(::baxter_core_msgs::msg::AssemblyState::_enabled_type arg)
  {
    msg_.enabled = std::move(arg);
    return Init_AssemblyState_stopped(msg_);
  }

private:
  ::baxter_core_msgs::msg::AssemblyState msg_;
};

class Init_AssemblyState_ready
{
public:
  Init_AssemblyState_ready()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AssemblyState_enabled ready(::baxter_core_msgs::msg::AssemblyState::_ready_type arg)
  {
    msg_.ready = std::move(arg);
    return Init_AssemblyState_enabled(msg_);
  }

private:
  ::baxter_core_msgs::msg::AssemblyState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::AssemblyState>()
{
  return baxter_core_msgs::msg::builder::Init_AssemblyState_ready();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__BUILDER_HPP_
