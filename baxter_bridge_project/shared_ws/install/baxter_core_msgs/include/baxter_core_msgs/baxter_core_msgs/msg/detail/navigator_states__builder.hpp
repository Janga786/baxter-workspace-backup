// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/NavigatorStates.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATES__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/navigator_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_NavigatorStates_states
{
public:
  explicit Init_NavigatorStates_states(::baxter_core_msgs::msg::NavigatorStates & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::NavigatorStates states(::baxter_core_msgs::msg::NavigatorStates::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::NavigatorStates msg_;
};

class Init_NavigatorStates_names
{
public:
  Init_NavigatorStates_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigatorStates_states names(::baxter_core_msgs::msg::NavigatorStates::_names_type arg)
  {
    msg_.names = std::move(arg);
    return Init_NavigatorStates_states(msg_);
  }

private:
  ::baxter_core_msgs::msg::NavigatorStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::NavigatorStates>()
{
  return baxter_core_msgs::msg::builder::Init_NavigatorStates_names();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATES__BUILDER_HPP_
