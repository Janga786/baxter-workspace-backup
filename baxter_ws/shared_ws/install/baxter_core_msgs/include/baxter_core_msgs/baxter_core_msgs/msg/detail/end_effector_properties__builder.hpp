// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/EndEffectorProperties.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/end_effector_properties__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_EndEffectorProperties_properties
{
public:
  explicit Init_EndEffectorProperties_properties(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::EndEffectorProperties properties(::baxter_core_msgs::msg::EndEffectorProperties::_properties_type arg)
  {
    msg_.properties = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_senses_position
{
public:
  explicit Init_EndEffectorProperties_senses_position(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_properties senses_position(::baxter_core_msgs::msg::EndEffectorProperties::_senses_position_type arg)
  {
    msg_.senses_position = std::move(arg);
    return Init_EndEffectorProperties_properties(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_controls_position
{
public:
  explicit Init_EndEffectorProperties_controls_position(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_senses_position controls_position(::baxter_core_msgs::msg::EndEffectorProperties::_controls_position_type arg)
  {
    msg_.controls_position = std::move(arg);
    return Init_EndEffectorProperties_senses_position(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_senses_force
{
public:
  explicit Init_EndEffectorProperties_senses_force(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_controls_position senses_force(::baxter_core_msgs::msg::EndEffectorProperties::_senses_force_type arg)
  {
    msg_.senses_force = std::move(arg);
    return Init_EndEffectorProperties_controls_position(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_controls_force
{
public:
  explicit Init_EndEffectorProperties_controls_force(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_senses_force controls_force(::baxter_core_msgs::msg::EndEffectorProperties::_controls_force_type arg)
  {
    msg_.controls_force = std::move(arg);
    return Init_EndEffectorProperties_senses_force(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_reverses_grip
{
public:
  explicit Init_EndEffectorProperties_reverses_grip(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_controls_force reverses_grip(::baxter_core_msgs::msg::EndEffectorProperties::_reverses_grip_type arg)
  {
    msg_.reverses_grip = std::move(arg);
    return Init_EndEffectorProperties_controls_force(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_senses_grip
{
public:
  explicit Init_EndEffectorProperties_senses_grip(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_reverses_grip senses_grip(::baxter_core_msgs::msg::EndEffectorProperties::_senses_grip_type arg)
  {
    msg_.senses_grip = std::move(arg);
    return Init_EndEffectorProperties_reverses_grip(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_controls_grip
{
public:
  explicit Init_EndEffectorProperties_controls_grip(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_senses_grip controls_grip(::baxter_core_msgs::msg::EndEffectorProperties::_controls_grip_type arg)
  {
    msg_.controls_grip = std::move(arg);
    return Init_EndEffectorProperties_senses_grip(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_has_calibration
{
public:
  explicit Init_EndEffectorProperties_has_calibration(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_controls_grip has_calibration(::baxter_core_msgs::msg::EndEffectorProperties::_has_calibration_type arg)
  {
    msg_.has_calibration = std::move(arg);
    return Init_EndEffectorProperties_controls_grip(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_firmware_date
{
public:
  explicit Init_EndEffectorProperties_firmware_date(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_has_calibration firmware_date(::baxter_core_msgs::msg::EndEffectorProperties::_firmware_date_type arg)
  {
    msg_.firmware_date = std::move(arg);
    return Init_EndEffectorProperties_has_calibration(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_firmware_rev
{
public:
  explicit Init_EndEffectorProperties_firmware_rev(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_firmware_date firmware_rev(::baxter_core_msgs::msg::EndEffectorProperties::_firmware_rev_type arg)
  {
    msg_.firmware_rev = std::move(arg);
    return Init_EndEffectorProperties_firmware_date(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_hardware_rev
{
public:
  explicit Init_EndEffectorProperties_hardware_rev(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_firmware_rev hardware_rev(::baxter_core_msgs::msg::EndEffectorProperties::_hardware_rev_type arg)
  {
    msg_.hardware_rev = std::move(arg);
    return Init_EndEffectorProperties_firmware_rev(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_serial_number
{
public:
  explicit Init_EndEffectorProperties_serial_number(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_hardware_rev serial_number(::baxter_core_msgs::msg::EndEffectorProperties::_serial_number_type arg)
  {
    msg_.serial_number = std::move(arg);
    return Init_EndEffectorProperties_hardware_rev(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_product
{
public:
  explicit Init_EndEffectorProperties_product(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_serial_number product(::baxter_core_msgs::msg::EndEffectorProperties::_product_type arg)
  {
    msg_.product = std::move(arg);
    return Init_EndEffectorProperties_serial_number(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_manufacturer
{
public:
  explicit Init_EndEffectorProperties_manufacturer(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_product manufacturer(::baxter_core_msgs::msg::EndEffectorProperties::_manufacturer_type arg)
  {
    msg_.manufacturer = std::move(arg);
    return Init_EndEffectorProperties_product(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_ui_type
{
public:
  explicit Init_EndEffectorProperties_ui_type(::baxter_core_msgs::msg::EndEffectorProperties & msg)
  : msg_(msg)
  {}
  Init_EndEffectorProperties_manufacturer ui_type(::baxter_core_msgs::msg::EndEffectorProperties::_ui_type_type arg)
  {
    msg_.ui_type = std::move(arg);
    return Init_EndEffectorProperties_manufacturer(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

class Init_EndEffectorProperties_id
{
public:
  Init_EndEffectorProperties_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EndEffectorProperties_ui_type id(::baxter_core_msgs::msg::EndEffectorProperties::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_EndEffectorProperties_ui_type(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorProperties msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::EndEffectorProperties>()
{
  return baxter_core_msgs::msg::builder::Init_EndEffectorProperties_id();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__BUILDER_HPP_
