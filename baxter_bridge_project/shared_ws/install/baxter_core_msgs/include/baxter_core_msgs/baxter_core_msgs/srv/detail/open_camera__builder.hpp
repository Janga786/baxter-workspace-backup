// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:srv/OpenCamera.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__OPEN_CAMERA__BUILDER_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__OPEN_CAMERA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/srv/detail/open_camera__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace srv
{

namespace builder
{

class Init_OpenCamera_Request_settings
{
public:
  explicit Init_OpenCamera_Request_settings(::baxter_core_msgs::srv::OpenCamera_Request & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::srv::OpenCamera_Request settings(::baxter_core_msgs::srv::OpenCamera_Request::_settings_type arg)
  {
    msg_.settings = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::srv::OpenCamera_Request msg_;
};

class Init_OpenCamera_Request_name
{
public:
  Init_OpenCamera_Request_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenCamera_Request_settings name(::baxter_core_msgs::srv::OpenCamera_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_OpenCamera_Request_settings(msg_);
  }

private:
  ::baxter_core_msgs::srv::OpenCamera_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::OpenCamera_Request>()
{
  return baxter_core_msgs::srv::builder::Init_OpenCamera_Request_name();
}

}  // namespace baxter_core_msgs


namespace baxter_core_msgs
{

namespace srv
{

namespace builder
{

class Init_OpenCamera_Response_err
{
public:
  Init_OpenCamera_Response_err()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::baxter_core_msgs::srv::OpenCamera_Response err(::baxter_core_msgs::srv::OpenCamera_Response::_err_type arg)
  {
    msg_.err = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::srv::OpenCamera_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::OpenCamera_Response>()
{
  return baxter_core_msgs::srv::builder::Init_OpenCamera_Response_err();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__OPEN_CAMERA__BUILDER_HPP_
