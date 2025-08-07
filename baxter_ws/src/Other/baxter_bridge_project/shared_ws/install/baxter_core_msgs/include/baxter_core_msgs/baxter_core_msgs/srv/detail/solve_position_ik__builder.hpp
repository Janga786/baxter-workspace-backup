// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:srv/SolvePositionIK.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__BUILDER_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/srv/detail/solve_position_ik__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace srv
{

namespace builder
{

class Init_SolvePositionIK_Request_seed_mode
{
public:
  explicit Init_SolvePositionIK_Request_seed_mode(::baxter_core_msgs::srv::SolvePositionIK_Request & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::srv::SolvePositionIK_Request seed_mode(::baxter_core_msgs::srv::SolvePositionIK_Request::_seed_mode_type arg)
  {
    msg_.seed_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::srv::SolvePositionIK_Request msg_;
};

class Init_SolvePositionIK_Request_seed_angles
{
public:
  explicit Init_SolvePositionIK_Request_seed_angles(::baxter_core_msgs::srv::SolvePositionIK_Request & msg)
  : msg_(msg)
  {}
  Init_SolvePositionIK_Request_seed_mode seed_angles(::baxter_core_msgs::srv::SolvePositionIK_Request::_seed_angles_type arg)
  {
    msg_.seed_angles = std::move(arg);
    return Init_SolvePositionIK_Request_seed_mode(msg_);
  }

private:
  ::baxter_core_msgs::srv::SolvePositionIK_Request msg_;
};

class Init_SolvePositionIK_Request_pose_stamp
{
public:
  Init_SolvePositionIK_Request_pose_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SolvePositionIK_Request_seed_angles pose_stamp(::baxter_core_msgs::srv::SolvePositionIK_Request::_pose_stamp_type arg)
  {
    msg_.pose_stamp = std::move(arg);
    return Init_SolvePositionIK_Request_seed_angles(msg_);
  }

private:
  ::baxter_core_msgs::srv::SolvePositionIK_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::SolvePositionIK_Request>()
{
  return baxter_core_msgs::srv::builder::Init_SolvePositionIK_Request_pose_stamp();
}

}  // namespace baxter_core_msgs


namespace baxter_core_msgs
{

namespace srv
{

namespace builder
{

class Init_SolvePositionIK_Response_result_type
{
public:
  explicit Init_SolvePositionIK_Response_result_type(::baxter_core_msgs::srv::SolvePositionIK_Response & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::srv::SolvePositionIK_Response result_type(::baxter_core_msgs::srv::SolvePositionIK_Response::_result_type_type arg)
  {
    msg_.result_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::srv::SolvePositionIK_Response msg_;
};

class Init_SolvePositionIK_Response_is_valid
{
public:
  explicit Init_SolvePositionIK_Response_is_valid(::baxter_core_msgs::srv::SolvePositionIK_Response & msg)
  : msg_(msg)
  {}
  Init_SolvePositionIK_Response_result_type is_valid(::baxter_core_msgs::srv::SolvePositionIK_Response::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return Init_SolvePositionIK_Response_result_type(msg_);
  }

private:
  ::baxter_core_msgs::srv::SolvePositionIK_Response msg_;
};

class Init_SolvePositionIK_Response_joints
{
public:
  Init_SolvePositionIK_Response_joints()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SolvePositionIK_Response_is_valid joints(::baxter_core_msgs::srv::SolvePositionIK_Response::_joints_type arg)
  {
    msg_.joints = std::move(arg);
    return Init_SolvePositionIK_Response_is_valid(msg_);
  }

private:
  ::baxter_core_msgs::srv::SolvePositionIK_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::srv::SolvePositionIK_Response>()
{
  return baxter_core_msgs::srv::builder::Init_SolvePositionIK_Response_joints();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__BUILDER_HPP_
