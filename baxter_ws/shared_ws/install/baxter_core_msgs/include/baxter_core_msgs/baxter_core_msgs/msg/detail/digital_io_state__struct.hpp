// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/DigitalIOState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATE__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__DigitalIOState __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__DigitalIOState __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DigitalIOState_
{
  using Type = DigitalIOState_<ContainerAllocator>;

  explicit DigitalIOState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->is_input_only = false;
    }
  }

  explicit DigitalIOState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->is_input_only = false;
    }
  }

  // field types and members
  using _state_type =
    int8_t;
  _state_type state;
  using _is_input_only_type =
    bool;
  _is_input_only_type is_input_only;

  // setters for named parameter idiom
  Type & set__state(
    const int8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__is_input_only(
    const bool & _arg)
  {
    this->is_input_only = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t OFF =
    0;
  static constexpr int8_t ON =
    1;
  static constexpr int8_t PRESSED =
    1;
  static constexpr int8_t UNPRESSED =
    0;

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__DigitalIOState
    std::shared_ptr<baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__DigitalIOState
    std::shared_ptr<baxter_core_msgs::msg::DigitalIOState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DigitalIOState_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    if (this->is_input_only != other.is_input_only) {
      return false;
    }
    return true;
  }
  bool operator!=(const DigitalIOState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DigitalIOState_

// alias to use template instance with default allocator
using DigitalIOState =
  baxter_core_msgs::msg::DigitalIOState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t DigitalIOState_<ContainerAllocator>::OFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t DigitalIOState_<ContainerAllocator>::ON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t DigitalIOState_<ContainerAllocator>::PRESSED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t DigitalIOState_<ContainerAllocator>::UNPRESSED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATE__STRUCT_HPP_
