// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:srv/ListCameras.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__BUILDER_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/srv/detail/list_cameras__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::ListCameras_Request>()
{
  return ::baxter_core_msgs::srv::ListCameras_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace baxter_core_msgs


namespace baxter_core_msgs
{

namespace srv
{

namespace builder
{

class Init_ListCameras_Response_cameras
{
public:
  Init_ListCameras_Response_cameras()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::baxter_core_msgs::srv::ListCameras_Response cameras(::baxter_core_msgs::srv::ListCameras_Response::_cameras_type arg)
  {
    msg_.cameras = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::srv::ListCameras_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::ListCameras_Response>()
{
  return baxter_core_msgs::srv::builder::Init_ListCameras_Response_cameras();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__BUILDER_HPP_
