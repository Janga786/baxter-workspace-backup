// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmEnable.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__BUILDER_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_enable__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_maintenance_msgs
{

namespace msg
{

namespace builder
{

class Init_CalibrateArmEnable_data
{
public:
  explicit Init_CalibrateArmEnable_data(::baxter_maintenance_msgs::msg::CalibrateArmEnable & msg)
  : msg_(msg)
  {}
  ::baxter_maintenance_msgs::msg::CalibrateArmEnable data(::baxter_maintenance_msgs::msg::CalibrateArmEnable::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::CalibrateArmEnable msg_;
};

class Init_CalibrateArmEnable_uid
{
public:
  explicit Init_CalibrateArmEnable_uid(::baxter_maintenance_msgs::msg::CalibrateArmEnable & msg)
  : msg_(msg)
  {}
  Init_CalibrateArmEnable_data uid(::baxter_maintenance_msgs::msg::CalibrateArmEnable::_uid_type arg)
  {
    msg_.uid = std::move(arg);
    return Init_CalibrateArmEnable_data(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::CalibrateArmEnable msg_;
};

class Init_CalibrateArmEnable_is_enabled
{
public:
  Init_CalibrateArmEnable_is_enabled()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrateArmEnable_uid is_enabled(::baxter_maintenance_msgs::msg::CalibrateArmEnable::_is_enabled_type arg)
  {
    msg_.is_enabled = std::move(arg);
    return Init_CalibrateArmEnable_uid(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::CalibrateArmEnable msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_maintenance_msgs::msg::CalibrateArmEnable>()
{
  return baxter_maintenance_msgs::msg::builder::Init_CalibrateArmEnable_is_enabled();
}

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__BUILDER_HPP_
