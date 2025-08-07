// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/EndEffectorState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/end_effector_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_EndEffectorState_command_sequence
{
public:
  explicit Init_EndEffectorState_command_sequence(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::EndEffectorState command_sequence(::baxter_core_msgs::msg::EndEffectorState::_command_sequence_type arg)
  {
    msg_.command_sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_command_sender
{
public:
  explicit Init_EndEffectorState_command_sender(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_command_sequence command_sender(::baxter_core_msgs::msg::EndEffectorState::_command_sender_type arg)
  {
    msg_.command_sender = std::move(arg);
    return Init_EndEffectorState_command_sequence(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_command
{
public:
  explicit Init_EndEffectorState_command(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_command_sender command(::baxter_core_msgs::msg::EndEffectorState::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_EndEffectorState_command_sender(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_state
{
public:
  explicit Init_EndEffectorState_state(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_command state(::baxter_core_msgs::msg::EndEffectorState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_EndEffectorState_command(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_force
{
public:
  explicit Init_EndEffectorState_force(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_state force(::baxter_core_msgs::msg::EndEffectorState::_force_type arg)
  {
    msg_.force = std::move(arg);
    return Init_EndEffectorState_state(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_position
{
public:
  explicit Init_EndEffectorState_position(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_force position(::baxter_core_msgs::msg::EndEffectorState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_EndEffectorState_force(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_reverse
{
public:
  explicit Init_EndEffectorState_reverse(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_position reverse(::baxter_core_msgs::msg::EndEffectorState::_reverse_type arg)
  {
    msg_.reverse = std::move(arg);
    return Init_EndEffectorState_position(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_error
{
public:
  explicit Init_EndEffectorState_error(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_reverse error(::baxter_core_msgs::msg::EndEffectorState::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_EndEffectorState_reverse(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_missed
{
public:
  explicit Init_EndEffectorState_missed(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_error missed(::baxter_core_msgs::msg::EndEffectorState::_missed_type arg)
  {
    msg_.missed = std::move(arg);
    return Init_EndEffectorState_error(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_gripping
{
public:
  explicit Init_EndEffectorState_gripping(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_missed gripping(::baxter_core_msgs::msg::EndEffectorState::_gripping_type arg)
  {
    msg_.gripping = std::move(arg);
    return Init_EndEffectorState_missed(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_moving
{
public:
  explicit Init_EndEffectorState_moving(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_gripping moving(::baxter_core_msgs::msg::EndEffectorState::_moving_type arg)
  {
    msg_.moving = std::move(arg);
    return Init_EndEffectorState_gripping(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_ready
{
public:
  explicit Init_EndEffectorState_ready(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_moving ready(::baxter_core_msgs::msg::EndEffectorState::_ready_type arg)
  {
    msg_.ready = std::move(arg);
    return Init_EndEffectorState_moving(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_calibrated
{
public:
  explicit Init_EndEffectorState_calibrated(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_ready calibrated(::baxter_core_msgs::msg::EndEffectorState::_calibrated_type arg)
  {
    msg_.calibrated = std::move(arg);
    return Init_EndEffectorState_ready(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_enabled
{
public:
  explicit Init_EndEffectorState_enabled(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_calibrated enabled(::baxter_core_msgs::msg::EndEffectorState::_enabled_type arg)
  {
    msg_.enabled = std::move(arg);
    return Init_EndEffectorState_calibrated(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_id
{
public:
  explicit Init_EndEffectorState_id(::baxter_core_msgs::msg::EndEffectorState & msg)
  : msg_(msg)
  {}
  Init_EndEffectorState_enabled id(::baxter_core_msgs::msg::EndEffectorState::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_EndEffectorState_enabled(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

class Init_EndEffectorState_timestamp
{
public:
  Init_EndEffectorState_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EndEffectorState_id timestamp(::baxter_core_msgs::msg::EndEffectorState::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_EndEffectorState_id(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::EndEffectorState>()
{
  return baxter_core_msgs::msg::builder::Init_EndEffectorState_timestamp();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__BUILDER_HPP_
