// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/DigitalIOState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/digital_io_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_DigitalIOState_is_input_only
{
public:
  explicit Init_DigitalIOState_is_input_only(::baxter_core_msgs::msg::DigitalIOState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::DigitalIOState is_input_only(::baxter_core_msgs::msg::DigitalIOState::_is_input_only_type arg)
  {
    msg_.is_input_only = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::DigitalIOState msg_;
};

class Init_DigitalIOState_state
{
public:
  Init_DigitalIOState_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DigitalIOState_is_input_only state(::baxter_core_msgs::msg::DigitalIOState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_DigitalIOState_is_input_only(msg_);
  }

private:
  ::baxter_core_msgs::msg::DigitalIOState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::DigitalIOState>()
{
  return baxter_core_msgs::msg::builder::Init_DigitalIOState_state();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATE__BUILDER_HPP_
