// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/AnalogOutputCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_OUTPUT_COMMAND__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_OUTPUT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/analog_output_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_AnalogOutputCommand_value
{
public:
  explicit Init_AnalogOutputCommand_value(::baxter_core_msgs::msg::AnalogOutputCommand & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::AnalogOutputCommand value(::baxter_core_msgs::msg::AnalogOutputCommand::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::AnalogOutputCommand msg_;
};

class Init_AnalogOutputCommand_name
{
public:
  Init_AnalogOutputCommand_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AnalogOutputCommand_value name(::baxter_core_msgs::msg::AnalogOutputCommand::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_AnalogOutputCommand_value(msg_);
  }

private:
  ::baxter_core_msgs::msg::AnalogOutputCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::AnalogOutputCommand>()
{
  return baxter_core_msgs::msg::builder::Init_AnalogOutputCommand_name();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_OUTPUT_COMMAND__BUILDER_HPP_
