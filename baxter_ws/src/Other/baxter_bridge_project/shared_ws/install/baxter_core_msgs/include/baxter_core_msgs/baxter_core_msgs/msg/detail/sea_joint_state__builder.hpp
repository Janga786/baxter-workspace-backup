// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/SEAJointState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/sea_joint_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_SEAJointState_hyst_state
{
public:
  explicit Init_SEAJointState_hyst_state(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::SEAJointState hyst_state(::baxter_core_msgs::msg::SEAJointState::_hyst_state_type arg)
  {
    msg_.hyst_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_crosstalk_model_effort
{
public:
  explicit Init_SEAJointState_crosstalk_model_effort(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_hyst_state crosstalk_model_effort(::baxter_core_msgs::msg::SEAJointState::_crosstalk_model_effort_type arg)
  {
    msg_.crosstalk_model_effort = std::move(arg);
    return Init_SEAJointState_hyst_state(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_hysteresis_model_effort
{
public:
  explicit Init_SEAJointState_hysteresis_model_effort(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_crosstalk_model_effort hysteresis_model_effort(::baxter_core_msgs::msg::SEAJointState::_hysteresis_model_effort_type arg)
  {
    msg_.hysteresis_model_effort = std::move(arg);
    return Init_SEAJointState_crosstalk_model_effort(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_gravity_only
{
public:
  explicit Init_SEAJointState_gravity_only(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_hysteresis_model_effort gravity_only(::baxter_core_msgs::msg::SEAJointState::_gravity_only_type arg)
  {
    msg_.gravity_only = std::move(arg);
    return Init_SEAJointState_hysteresis_model_effort(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_gravity_model_effort
{
public:
  explicit Init_SEAJointState_gravity_model_effort(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_gravity_only gravity_model_effort(::baxter_core_msgs::msg::SEAJointState::_gravity_model_effort_type arg)
  {
    msg_.gravity_model_effort = std::move(arg);
    return Init_SEAJointState_gravity_only(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_actual_effort
{
public:
  explicit Init_SEAJointState_actual_effort(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_gravity_model_effort actual_effort(::baxter_core_msgs::msg::SEAJointState::_actual_effort_type arg)
  {
    msg_.actual_effort = std::move(arg);
    return Init_SEAJointState_gravity_model_effort(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_actual_velocity
{
public:
  explicit Init_SEAJointState_actual_velocity(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_actual_effort actual_velocity(::baxter_core_msgs::msg::SEAJointState::_actual_velocity_type arg)
  {
    msg_.actual_velocity = std::move(arg);
    return Init_SEAJointState_actual_effort(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_actual_position
{
public:
  explicit Init_SEAJointState_actual_position(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_actual_velocity actual_position(::baxter_core_msgs::msg::SEAJointState::_actual_position_type arg)
  {
    msg_.actual_position = std::move(arg);
    return Init_SEAJointState_actual_velocity(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_commanded_effort
{
public:
  explicit Init_SEAJointState_commanded_effort(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_actual_position commanded_effort(::baxter_core_msgs::msg::SEAJointState::_commanded_effort_type arg)
  {
    msg_.commanded_effort = std::move(arg);
    return Init_SEAJointState_actual_position(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_commanded_acceleration
{
public:
  explicit Init_SEAJointState_commanded_acceleration(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_commanded_effort commanded_acceleration(::baxter_core_msgs::msg::SEAJointState::_commanded_acceleration_type arg)
  {
    msg_.commanded_acceleration = std::move(arg);
    return Init_SEAJointState_commanded_effort(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_commanded_velocity
{
public:
  explicit Init_SEAJointState_commanded_velocity(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_commanded_acceleration commanded_velocity(::baxter_core_msgs::msg::SEAJointState::_commanded_velocity_type arg)
  {
    msg_.commanded_velocity = std::move(arg);
    return Init_SEAJointState_commanded_acceleration(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_commanded_position
{
public:
  explicit Init_SEAJointState_commanded_position(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_commanded_velocity commanded_position(::baxter_core_msgs::msg::SEAJointState::_commanded_position_type arg)
  {
    msg_.commanded_position = std::move(arg);
    return Init_SEAJointState_commanded_velocity(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_name
{
public:
  explicit Init_SEAJointState_name(::baxter_core_msgs::msg::SEAJointState & msg)
  : msg_(msg)
  {}
  Init_SEAJointState_commanded_position name(::baxter_core_msgs::msg::SEAJointState::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_SEAJointState_commanded_position(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

class Init_SEAJointState_header
{
public:
  Init_SEAJointState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SEAJointState_name header(::baxter_core_msgs::msg::SEAJointState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SEAJointState_name(msg_);
  }

private:
  ::baxter_core_msgs::msg::SEAJointState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::SEAJointState>()
{
  return baxter_core_msgs::msg::builder::Init_SEAJointState_header();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__BUILDER_HPP_
