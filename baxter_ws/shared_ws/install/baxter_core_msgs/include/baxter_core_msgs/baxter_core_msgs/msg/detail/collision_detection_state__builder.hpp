// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/CollisionDetectionState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_DETECTION_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_DETECTION_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/collision_detection_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_CollisionDetectionState_collision_state
{
public:
  explicit Init_CollisionDetectionState_collision_state(::baxter_core_msgs::msg::CollisionDetectionState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::CollisionDetectionState collision_state(::baxter_core_msgs::msg::CollisionDetectionState::_collision_state_type arg)
  {
    msg_.collision_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::CollisionDetectionState msg_;
};

class Init_CollisionDetectionState_header
{
public:
  Init_CollisionDetectionState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CollisionDetectionState_collision_state header(::baxter_core_msgs::msg::CollisionDetectionState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CollisionDetectionState_collision_state(msg_);
  }

private:
  ::baxter_core_msgs::msg::CollisionDetectionState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::CollisionDetectionState>()
{
  return baxter_core_msgs::msg::builder::Init_CollisionDetectionState_header();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_DETECTION_STATE__BUILDER_HPP_
