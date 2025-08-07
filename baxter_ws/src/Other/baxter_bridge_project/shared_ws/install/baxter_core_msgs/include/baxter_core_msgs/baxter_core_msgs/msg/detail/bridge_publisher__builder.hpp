// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/BridgePublisher.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__BRIDGE_PUBLISHER__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__BRIDGE_PUBLISHER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/bridge_publisher__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_BridgePublisher_time
{
public:
  explicit Init_BridgePublisher_time(::baxter_core_msgs::msg::BridgePublisher & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::BridgePublisher time(::baxter_core_msgs::msg::BridgePublisher::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::BridgePublisher msg_;
};

class Init_BridgePublisher_user
{
public:
  explicit Init_BridgePublisher_user(::baxter_core_msgs::msg::BridgePublisher & msg)
  : msg_(msg)
  {}
  Init_BridgePublisher_time user(::baxter_core_msgs::msg::BridgePublisher::_user_type arg)
  {
    msg_.user = std::move(arg);
    return Init_BridgePublisher_time(msg_);
  }

private:
  ::baxter_core_msgs::msg::BridgePublisher msg_;
};

class Init_BridgePublisher_topic
{
public:
  Init_BridgePublisher_topic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BridgePublisher_user topic(::baxter_core_msgs::msg::BridgePublisher::_topic_type arg)
  {
    msg_.topic = std::move(arg);
    return Init_BridgePublisher_user(msg_);
  }

private:
  ::baxter_core_msgs::msg::BridgePublisher msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::BridgePublisher>()
{
  return baxter_core_msgs::msg::builder::Init_BridgePublisher_topic();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__BRIDGE_PUBLISHER__BUILDER_HPP_
