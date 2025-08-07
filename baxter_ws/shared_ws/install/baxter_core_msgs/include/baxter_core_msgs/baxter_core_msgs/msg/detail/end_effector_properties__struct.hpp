// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/EndEffectorProperties.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__EndEffectorProperties __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__EndEffectorProperties __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EndEffectorProperties_
{
  using Type = EndEffectorProperties_<ContainerAllocator>;

  explicit EndEffectorProperties_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->ui_type = 0;
      this->manufacturer = "";
      this->product = "";
      this->serial_number = "";
      this->hardware_rev = "";
      this->firmware_rev = "";
      this->firmware_date = "";
      this->has_calibration = false;
      this->controls_grip = false;
      this->senses_grip = false;
      this->reverses_grip = false;
      this->controls_force = false;
      this->senses_force = false;
      this->controls_position = false;
      this->senses_position = false;
      this->properties = "";
    }
  }

  explicit EndEffectorProperties_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : manufacturer(_alloc),
    product(_alloc),
    serial_number(_alloc),
    hardware_rev(_alloc),
    firmware_rev(_alloc),
    firmware_date(_alloc),
    properties(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->ui_type = 0;
      this->manufacturer = "";
      this->product = "";
      this->serial_number = "";
      this->hardware_rev = "";
      this->firmware_rev = "";
      this->firmware_date = "";
      this->has_calibration = false;
      this->controls_grip = false;
      this->senses_grip = false;
      this->reverses_grip = false;
      this->controls_force = false;
      this->senses_force = false;
      this->controls_position = false;
      this->senses_position = false;
      this->properties = "";
    }
  }

  // field types and members
  using _id_type =
    uint32_t;
  _id_type id;
  using _ui_type_type =
    uint8_t;
  _ui_type_type ui_type;
  using _manufacturer_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _manufacturer_type manufacturer;
  using _product_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _product_type product;
  using _serial_number_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _serial_number_type serial_number;
  using _hardware_rev_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _hardware_rev_type hardware_rev;
  using _firmware_rev_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _firmware_rev_type firmware_rev;
  using _firmware_date_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _firmware_date_type firmware_date;
  using _has_calibration_type =
    bool;
  _has_calibration_type has_calibration;
  using _controls_grip_type =
    bool;
  _controls_grip_type controls_grip;
  using _senses_grip_type =
    bool;
  _senses_grip_type senses_grip;
  using _reverses_grip_type =
    bool;
  _reverses_grip_type reverses_grip;
  using _controls_force_type =
    bool;
  _controls_force_type controls_force;
  using _senses_force_type =
    bool;
  _senses_force_type senses_force;
  using _controls_position_type =
    bool;
  _controls_position_type controls_position;
  using _senses_position_type =
    bool;
  _senses_position_type senses_position;
  using _properties_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _properties_type properties;

  // setters for named parameter idiom
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__ui_type(
    const uint8_t & _arg)
  {
    this->ui_type = _arg;
    return *this;
  }
  Type & set__manufacturer(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->manufacturer = _arg;
    return *this;
  }
  Type & set__product(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->product = _arg;
    return *this;
  }
  Type & set__serial_number(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->serial_number = _arg;
    return *this;
  }
  Type & set__hardware_rev(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->hardware_rev = _arg;
    return *this;
  }
  Type & set__firmware_rev(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->firmware_rev = _arg;
    return *this;
  }
  Type & set__firmware_date(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->firmware_date = _arg;
    return *this;
  }
  Type & set__has_calibration(
    const bool & _arg)
  {
    this->has_calibration = _arg;
    return *this;
  }
  Type & set__controls_grip(
    const bool & _arg)
  {
    this->controls_grip = _arg;
    return *this;
  }
  Type & set__senses_grip(
    const bool & _arg)
  {
    this->senses_grip = _arg;
    return *this;
  }
  Type & set__reverses_grip(
    const bool & _arg)
  {
    this->reverses_grip = _arg;
    return *this;
  }
  Type & set__controls_force(
    const bool & _arg)
  {
    this->controls_force = _arg;
    return *this;
  }
  Type & set__senses_force(
    const bool & _arg)
  {
    this->senses_force = _arg;
    return *this;
  }
  Type & set__controls_position(
    const bool & _arg)
  {
    this->controls_position = _arg;
    return *this;
  }
  Type & set__senses_position(
    const bool & _arg)
  {
    this->senses_position = _arg;
    return *this;
  }
  Type & set__properties(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->properties = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NO_GRIPPER =
    0u;
  static constexpr uint8_t SUCTION_CUP_GRIPPER =
    1u;
  static constexpr uint8_t ELECTRIC_GRIPPER =
    2u;
  static constexpr uint8_t PASSIVE_GRIPPER =
    3u;

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__EndEffectorProperties
    std::shared_ptr<baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__EndEffectorProperties
    std::shared_ptr<baxter_core_msgs::msg::EndEffectorProperties_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EndEffectorProperties_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->ui_type != other.ui_type) {
      return false;
    }
    if (this->manufacturer != other.manufacturer) {
      return false;
    }
    if (this->product != other.product) {
      return false;
    }
    if (this->serial_number != other.serial_number) {
      return false;
    }
    if (this->hardware_rev != other.hardware_rev) {
      return false;
    }
    if (this->firmware_rev != other.firmware_rev) {
      return false;
    }
    if (this->firmware_date != other.firmware_date) {
      return false;
    }
    if (this->has_calibration != other.has_calibration) {
      return false;
    }
    if (this->controls_grip != other.controls_grip) {
      return false;
    }
    if (this->senses_grip != other.senses_grip) {
      return false;
    }
    if (this->reverses_grip != other.reverses_grip) {
      return false;
    }
    if (this->controls_force != other.controls_force) {
      return false;
    }
    if (this->senses_force != other.senses_force) {
      return false;
    }
    if (this->controls_position != other.controls_position) {
      return false;
    }
    if (this->senses_position != other.senses_position) {
      return false;
    }
    if (this->properties != other.properties) {
      return false;
    }
    return true;
  }
  bool operator!=(const EndEffectorProperties_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EndEffectorProperties_

// alias to use template instance with default allocator
using EndEffectorProperties =
  baxter_core_msgs::msg::EndEffectorProperties_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t EndEffectorProperties_<ContainerAllocator>::NO_GRIPPER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t EndEffectorProperties_<ContainerAllocator>::SUCTION_CUP_GRIPPER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t EndEffectorProperties_<ContainerAllocator>::ELECTRIC_GRIPPER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t EndEffectorProperties_<ContainerAllocator>::PASSIVE_GRIPPER;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__STRUCT_HPP_
