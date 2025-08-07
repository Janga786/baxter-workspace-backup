// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmData.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__BUILDER_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_maintenance_msgs
{

namespace msg
{

namespace builder
{

class Init_CalibrateArmData_suppress_write_to_file
{
public:
  Init_CalibrateArmData_suppress_write_to_file()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::baxter_maintenance_msgs::msg::CalibrateArmData suppress_write_to_file(::baxter_maintenance_msgs::msg::CalibrateArmData::_suppress_write_to_file_type arg)
  {
    msg_.suppress_write_to_file = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::CalibrateArmData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_maintenance_msgs::msg::CalibrateArmData>()
{
  return baxter_maintenance_msgs::msg::builder::Init_CalibrateArmData_suppress_write_to_file();
}

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__BUILDER_HPP_
