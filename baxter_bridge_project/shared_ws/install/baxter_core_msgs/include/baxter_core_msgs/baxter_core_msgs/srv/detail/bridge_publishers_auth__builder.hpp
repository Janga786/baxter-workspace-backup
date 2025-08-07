// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:srv/BridgePublishersAuth.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__BUILDER_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/srv/detail/bridge_publishers_auth__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace srv
{

namespace builder
{

class Init_BridgePublishersAuth_Request_user
{
public:
  explicit Init_BridgePublishersAuth_Request_user(::baxter_core_msgs::srv::BridgePublishersAuth_Request & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::srv::BridgePublishersAuth_Request user(::baxter_core_msgs::srv::BridgePublishersAuth_Request::_user_type arg)
  {
    msg_.user = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::srv::BridgePublishersAuth_Request msg_;
};

class Init_BridgePublishersAuth_Request_topic
{
public:
  Init_BridgePublishersAuth_Request_topic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BridgePublishersAuth_Request_user topic(::baxter_core_msgs::srv::BridgePublishersAuth_Request::_topic_type arg)
  {
    msg_.topic = std::move(arg);
    return Init_BridgePublishersAuth_Request_user(msg_);
  }

private:
  ::baxter_core_msgs::srv::BridgePublishersAuth_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::BridgePublishersAuth_Request>()
{
  return baxter_core_msgs::srv::builder::Init_BridgePublishersAuth_Request_topic();
}

}  // namespace baxter_core_msgs


namespace baxter_core_msgs
{

namespace srv
{

namespace builder
{

class Init_BridgePublishersAuth_Response_forced_right
{
public:
  explicit Init_BridgePublishersAuth_Response_forced_right(::baxter_core_msgs::srv::BridgePublishersAuth_Response & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::srv::BridgePublishersAuth_Response forced_right(::baxter_core_msgs::srv::BridgePublishersAuth_Response::_forced_right_type arg)
  {
    msg_.forced_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::srv::BridgePublishersAuth_Response msg_;
};

class Init_BridgePublishersAuth_Response_forced_left
{
public:
  explicit Init_BridgePublishersAuth_Response_forced_left(::baxter_core_msgs::srv::BridgePublishersAuth_Response & msg)
  : msg_(msg)
  {}
  Init_BridgePublishersAuth_Response_forced_right forced_left(::baxter_core_msgs::srv::BridgePublishersAuth_Response::_forced_left_type arg)
  {
    msg_.forced_left = std::move(arg);
    return Init_BridgePublishersAuth_Response_forced_right(msg_);
  }

private:
  ::baxter_core_msgs::srv::BridgePublishersAuth_Response msg_;
};

class Init_BridgePublishersAuth_Response_publishers
{
public:
  Init_BridgePublishersAuth_Response_publishers()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BridgePublishersAuth_Response_forced_left publishers(::baxter_core_msgs::srv::BridgePublishersAuth_Response::_publishers_type arg)
  {
    msg_.publishers = std::move(arg);
    return Init_BridgePublishersAuth_Response_forced_left(msg_);
  }

private:
  ::baxter_core_msgs::srv::BridgePublishersAuth_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::BridgePublishersAuth_Response>()
{
  return baxter_core_msgs::srv::builder::Init_BridgePublishersAuth_Response_publishers();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__BUILDER_HPP_
