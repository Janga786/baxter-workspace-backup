// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_maintenance_msgs:msg/TareData.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_DATA__BUILDER_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_maintenance_msgs/msg/detail/tare_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_maintenance_msgs
{

namespace msg
{

namespace builder
{

class Init_TareData_tune_gravity_spring
{
public:
  Init_TareData_tune_gravity_spring()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::baxter_maintenance_msgs::msg::TareData tune_gravity_spring(::baxter_maintenance_msgs::msg::TareData::_tune_gravity_spring_type arg)
  {
    msg_.tune_gravity_spring = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::TareData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_maintenance_msgs::msg::TareData>()
{
  return baxter_maintenance_msgs::msg::builder::Init_TareData_tune_gravity_spring();
}

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_DATA__BUILDER_HPP_
