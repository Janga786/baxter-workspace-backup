// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/AssemblyStates.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATES__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/assembly_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_AssemblyStates_states
{
public:
  explicit Init_AssemblyStates_states(::baxter_core_msgs::msg::AssemblyStates & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::AssemblyStates states(::baxter_core_msgs::msg::AssemblyStates::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::AssemblyStates msg_;
};

class Init_AssemblyStates_names
{
public:
  Init_AssemblyStates_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AssemblyStates_states names(::baxter_core_msgs::msg::AssemblyStates::_names_type arg)
  {
    msg_.names = std::move(arg);
    return Init_AssemblyStates_states(msg_);
  }

private:
  ::baxter_core_msgs::msg::AssemblyStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::AssemblyStates>()
{
  return baxter_core_msgs::msg::builder::Init_AssemblyStates_names();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATES__BUILDER_HPP_
