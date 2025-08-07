// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/URDFConfiguration.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/urdf_configuration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_URDFConfiguration_urdf
{
public:
  explicit Init_URDFConfiguration_urdf(::baxter_core_msgs::msg::URDFConfiguration & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::URDFConfiguration urdf(::baxter_core_msgs::msg::URDFConfiguration::_urdf_type arg)
  {
    msg_.urdf = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::URDFConfiguration msg_;
};

class Init_URDFConfiguration_joint
{
public:
  explicit Init_URDFConfiguration_joint(::baxter_core_msgs::msg::URDFConfiguration & msg)
  : msg_(msg)
  {}
  Init_URDFConfiguration_urdf joint(::baxter_core_msgs::msg::URDFConfiguration::_joint_type arg)
  {
    msg_.joint = std::move(arg);
    return Init_URDFConfiguration_urdf(msg_);
  }

private:
  ::baxter_core_msgs::msg::URDFConfiguration msg_;
};

class Init_URDFConfiguration_link
{
public:
  explicit Init_URDFConfiguration_link(::baxter_core_msgs::msg::URDFConfiguration & msg)
  : msg_(msg)
  {}
  Init_URDFConfiguration_joint link(::baxter_core_msgs::msg::URDFConfiguration::_link_type arg)
  {
    msg_.link = std::move(arg);
    return Init_URDFConfiguration_joint(msg_);
  }

private:
  ::baxter_core_msgs::msg::URDFConfiguration msg_;
};

class Init_URDFConfiguration_time
{
public:
  Init_URDFConfiguration_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_URDFConfiguration_link time(::baxter_core_msgs::msg::URDFConfiguration::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_URDFConfiguration_link(msg_);
  }

private:
  ::baxter_core_msgs::msg::URDFConfiguration msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::URDFConfiguration>()
{
  return baxter_core_msgs::msg::builder::Init_URDFConfiguration_time();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__BUILDER_HPP_
