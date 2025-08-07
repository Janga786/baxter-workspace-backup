// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/EndpointStates.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATES__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/endpoint_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_EndpointStates_states
{
public:
  explicit Init_EndpointStates_states(::baxter_core_msgs::msg::EndpointStates & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::EndpointStates states(::baxter_core_msgs::msg::EndpointStates::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndpointStates msg_;
};

class Init_EndpointStates_names
{
public:
  Init_EndpointStates_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EndpointStates_states names(::baxter_core_msgs::msg::EndpointStates::_names_type arg)
  {
    msg_.names = std::move(arg);
    return Init_EndpointStates_states(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndpointStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::EndpointStates>()
{
  return baxter_core_msgs::msg::builder::Init_EndpointStates_names();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATES__BUILDER_HPP_
