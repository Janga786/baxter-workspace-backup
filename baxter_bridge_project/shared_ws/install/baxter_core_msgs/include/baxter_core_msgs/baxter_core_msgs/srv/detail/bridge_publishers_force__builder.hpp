// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:srv/BridgePublishersForce.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/srv/detail/bridge_publishers_force__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace srv
{

namespace builder
{

class Init_BridgePublishersForce_Request_right_user
{
public:
  explicit Init_BridgePublishersForce_Request_right_user(::baxter_core_msgs::srv::BridgePublishersForce_Request & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::srv::BridgePublishersForce_Request right_user(::baxter_core_msgs::srv::BridgePublishersForce_Request::_right_user_type arg)
  {
    msg_.right_user = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::srv::BridgePublishersForce_Request msg_;
};

class Init_BridgePublishersForce_Request_left_user
{
public:
  Init_BridgePublishersForce_Request_left_user()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BridgePublishersForce_Request_right_user left_user(::baxter_core_msgs::srv::BridgePublishersForce_Request::_left_user_type arg)
  {
    msg_.left_user = std::move(arg);
    return Init_BridgePublishersForce_Request_right_user(msg_);
  }

private:
  ::baxter_core_msgs::srv::BridgePublishersForce_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::BridgePublishersForce_Request>()
{
  return baxter_core_msgs::srv::builder::Init_BridgePublishersForce_Request_left_user();
}

}  // namespace baxter_core_msgs


namespace baxter_core_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::BridgePublishersForce_Response>()
{
  return ::baxter_core_msgs::srv::BridgePublishersForce_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__BUILDER_HPP_
