// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/CollisionAvoidanceState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_AVOIDANCE_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_AVOIDANCE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/collision_avoidance_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_CollisionAvoidanceState_collision_object
{
public:
  explicit Init_CollisionAvoidanceState_collision_object(::baxter_core_msgs::msg::CollisionAvoidanceState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::CollisionAvoidanceState collision_object(::baxter_core_msgs::msg::CollisionAvoidanceState::_collision_object_type arg)
  {
    msg_.collision_object = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::CollisionAvoidanceState msg_;
};

class Init_CollisionAvoidanceState_other_arm
{
public:
  explicit Init_CollisionAvoidanceState_other_arm(::baxter_core_msgs::msg::CollisionAvoidanceState & msg)
  : msg_(msg)
  {}
  Init_CollisionAvoidanceState_collision_object other_arm(::baxter_core_msgs::msg::CollisionAvoidanceState::_other_arm_type arg)
  {
    msg_.other_arm = std::move(arg);
    return Init_CollisionAvoidanceState_collision_object(msg_);
  }

private:
  ::baxter_core_msgs::msg::CollisionAvoidanceState msg_;
};

class Init_CollisionAvoidanceState_header
{
public:
  Init_CollisionAvoidanceState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CollisionAvoidanceState_other_arm header(::baxter_core_msgs::msg::CollisionAvoidanceState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CollisionAvoidanceState_other_arm(msg_);
  }

private:
  ::baxter_core_msgs::msg::CollisionAvoidanceState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::CollisionAvoidanceState>()
{
  return baxter_core_msgs::msg::builder::Init_CollisionAvoidanceState_header();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_AVOIDANCE_STATE__BUILDER_HPP_
