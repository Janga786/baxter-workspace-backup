// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/AnalogIOState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/analog_io_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_AnalogIOState_is_input_only
{
public:
  explicit Init_AnalogIOState_is_input_only(::baxter_core_msgs::msg::AnalogIOState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::AnalogIOState is_input_only(::baxter_core_msgs::msg::AnalogIOState::_is_input_only_type arg)
  {
    msg_.is_input_only = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::AnalogIOState msg_;
};

class Init_AnalogIOState_value
{
public:
  explicit Init_AnalogIOState_value(::baxter_core_msgs::msg::AnalogIOState & msg)
  : msg_(msg)
  {}
  Init_AnalogIOState_is_input_only value(::baxter_core_msgs::msg::AnalogIOState::_value_type arg)
  {
    msg_.value = std::move(arg);
    return Init_AnalogIOState_is_input_only(msg_);
  }

private:
  ::baxter_core_msgs::msg::AnalogIOState msg_;
};

class Init_AnalogIOState_timestamp
{
public:
  Init_AnalogIOState_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AnalogIOState_value timestamp(::baxter_core_msgs::msg::AnalogIOState::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_AnalogIOState_value(msg_);
  }

private:
  ::baxter_core_msgs::msg::AnalogIOState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::AnalogIOState>()
{
  return baxter_core_msgs::msg::builder::Init_AnalogIOState_timestamp();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATE__BUILDER_HPP_
