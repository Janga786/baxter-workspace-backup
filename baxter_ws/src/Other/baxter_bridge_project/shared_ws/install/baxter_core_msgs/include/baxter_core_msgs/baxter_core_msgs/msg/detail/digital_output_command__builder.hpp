// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/DigitalOutputCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_OUTPUT_COMMAND__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_OUTPUT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/digital_output_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_DigitalOutputCommand_value
{
public:
  explicit Init_DigitalOutputCommand_value(::baxter_core_msgs::msg::DigitalOutputCommand & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::DigitalOutputCommand value(::baxter_core_msgs::msg::DigitalOutputCommand::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::DigitalOutputCommand msg_;
};

class Init_DigitalOutputCommand_name
{
public:
  Init_DigitalOutputCommand_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DigitalOutputCommand_value name(::baxter_core_msgs::msg::DigitalOutputCommand::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_DigitalOutputCommand_value(msg_);
  }

private:
  ::baxter_core_msgs::msg::DigitalOutputCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::DigitalOutputCommand>()
{
  return baxter_core_msgs::msg::builder::Init_DigitalOutputCommand_name();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_OUTPUT_COMMAND__BUILDER_HPP_
