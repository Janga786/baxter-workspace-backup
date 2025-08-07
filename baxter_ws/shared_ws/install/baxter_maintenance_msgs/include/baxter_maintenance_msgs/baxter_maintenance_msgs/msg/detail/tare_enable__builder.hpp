// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_maintenance_msgs:msg/TareEnable.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_ENABLE__BUILDER_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_ENABLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_maintenance_msgs/msg/detail/tare_enable__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_maintenance_msgs
{

namespace msg
{

namespace builder
{

class Init_TareEnable_data
{
public:
  explicit Init_TareEnable_data(::baxter_maintenance_msgs::msg::TareEnable & msg)
  : msg_(msg)
  {}
  ::baxter_maintenance_msgs::msg::TareEnable data(::baxter_maintenance_msgs::msg::TareEnable::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::TareEnable msg_;
};

class Init_TareEnable_uid
{
public:
  explicit Init_TareEnable_uid(::baxter_maintenance_msgs::msg::TareEnable & msg)
  : msg_(msg)
  {}
  Init_TareEnable_data uid(::baxter_maintenance_msgs::msg::TareEnable::_uid_type arg)
  {
    msg_.uid = std::move(arg);
    return Init_TareEnable_data(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::TareEnable msg_;
};

class Init_TareEnable_is_enabled
{
public:
  Init_TareEnable_is_enabled()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TareEnable_uid is_enabled(::baxter_maintenance_msgs::msg::TareEnable::_is_enabled_type arg)
  {
    msg_.is_enabled = std::move(arg);
    return Init_TareEnable_uid(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::TareEnable msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_maintenance_msgs::msg::TareEnable>()
{
  return baxter_maintenance_msgs::msg::builder::Init_TareEnable_is_enabled();
}

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_ENABLE__BUILDER_HPP_
