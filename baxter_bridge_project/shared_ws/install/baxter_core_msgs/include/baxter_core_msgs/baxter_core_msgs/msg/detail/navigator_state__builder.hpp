// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/NavigatorState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATE__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/navigator_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_NavigatorState_lights
{
public:
  explicit Init_NavigatorState_lights(::baxter_core_msgs::msg::NavigatorState & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::NavigatorState lights(::baxter_core_msgs::msg::NavigatorState::_lights_type arg)
  {
    msg_.lights = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::NavigatorState msg_;
};

class Init_NavigatorState_light_names
{
public:
  explicit Init_NavigatorState_light_names(::baxter_core_msgs::msg::NavigatorState & msg)
  : msg_(msg)
  {}
  Init_NavigatorState_lights light_names(::baxter_core_msgs::msg::NavigatorState::_light_names_type arg)
  {
    msg_.light_names = std::move(arg);
    return Init_NavigatorState_lights(msg_);
  }

private:
  ::baxter_core_msgs::msg::NavigatorState msg_;
};

class Init_NavigatorState_wheel
{
public:
  explicit Init_NavigatorState_wheel(::baxter_core_msgs::msg::NavigatorState & msg)
  : msg_(msg)
  {}
  Init_NavigatorState_light_names wheel(::baxter_core_msgs::msg::NavigatorState::_wheel_type arg)
  {
    msg_.wheel = std::move(arg);
    return Init_NavigatorState_light_names(msg_);
  }

private:
  ::baxter_core_msgs::msg::NavigatorState msg_;
};

class Init_NavigatorState_buttons
{
public:
  explicit Init_NavigatorState_buttons(::baxter_core_msgs::msg::NavigatorState & msg)
  : msg_(msg)
  {}
  Init_NavigatorState_wheel buttons(::baxter_core_msgs::msg::NavigatorState::_buttons_type arg)
  {
    msg_.buttons = std::move(arg);
    return Init_NavigatorState_wheel(msg_);
  }

private:
  ::baxter_core_msgs::msg::NavigatorState msg_;
};

class Init_NavigatorState_button_names
{
public:
  Init_NavigatorState_button_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigatorState_buttons button_names(::baxter_core_msgs::msg::NavigatorState::_button_names_type arg)
  {
    msg_.button_names = std::move(arg);
    return Init_NavigatorState_buttons(msg_);
  }

private:
  ::baxter_core_msgs::msg::NavigatorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::NavigatorState>()
{
  return baxter_core_msgs::msg::builder::Init_NavigatorState_button_names();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATE__BUILDER_HPP_
