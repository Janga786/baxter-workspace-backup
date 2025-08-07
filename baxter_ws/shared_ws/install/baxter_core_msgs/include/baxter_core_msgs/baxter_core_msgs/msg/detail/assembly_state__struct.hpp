// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/AssemblyState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__AssemblyState __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__AssemblyState __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AssemblyState_
{
  using Type = AssemblyState_<ContainerAllocator>;

  explicit AssemblyState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ready = false;
      this->enabled = false;
      this->stopped = false;
      this->error = false;
      this->estop_button = 0;
      this->estop_source = 0;
    }
  }

  explicit AssemblyState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ready = false;
      this->enabled = false;
      this->stopped = false;
      this->error = false;
      this->estop_button = 0;
      this->estop_source = 0;
    }
  }

  // field types and members
  using _ready_type =
    bool;
  _ready_type ready;
  using _enabled_type =
    bool;
  _enabled_type enabled;
  using _stopped_type =
    bool;
  _stopped_type stopped;
  using _error_type =
    bool;
  _error_type error;
  using _estop_button_type =
    uint8_t;
  _estop_button_type estop_button;
  using _estop_source_type =
    uint8_t;
  _estop_source_type estop_source;

  // setters for named parameter idiom
  Type & set__ready(
    const bool & _arg)
  {
    this->ready = _arg;
    return *this;
  }
  Type & set__enabled(
    const bool & _arg)
  {
    this->enabled = _arg;
    return *this;
  }
  Type & set__stopped(
    const bool & _arg)
  {
    this->stopped = _arg;
    return *this;
  }
  Type & set__error(
    const bool & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__estop_button(
    const uint8_t & _arg)
  {
    this->estop_button = _arg;
    return *this;
  }
  Type & set__estop_source(
    const uint8_t & _arg)
  {
    this->estop_source = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ESTOP_BUTTON_UNPRESSED =
    0u;
  static constexpr uint8_t ESTOP_BUTTON_PRESSED =
    1u;
  static constexpr uint8_t ESTOP_BUTTON_UNKNOWN =
    2u;
  static constexpr uint8_t ESTOP_BUTTON_RELEASED =
    3u;
  static constexpr uint8_t ESTOP_SOURCE_NONE =
    0u;
  static constexpr uint8_t ESTOP_SOURCE_USER =
    1u;
  static constexpr uint8_t ESTOP_SOURCE_UNKNOWN =
    2u;
  static constexpr uint8_t ESTOP_SOURCE_FAULT =
    3u;
  static constexpr uint8_t ESTOP_SOURCE_BRAIN =
    4u;

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::AssemblyState_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::AssemblyState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::AssemblyState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::AssemblyState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::AssemblyState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::AssemblyState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::AssemblyState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::AssemblyState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::AssemblyState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::AssemblyState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__AssemblyState
    std::shared_ptr<baxter_core_msgs::msg::AssemblyState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__AssemblyState
    std::shared_ptr<baxter_core_msgs::msg::AssemblyState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AssemblyState_ & other) const
  {
    if (this->ready != other.ready) {
      return false;
    }
    if (this->enabled != other.enabled) {
      return false;
    }
    if (this->stopped != other.stopped) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->estop_button != other.estop_button) {
      return false;
    }
    if (this->estop_source != other.estop_source) {
      return false;
    }
    return true;
  }
  bool operator!=(const AssemblyState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AssemblyState_

// alias to use template instance with default allocator
using AssemblyState =
  baxter_core_msgs::msg::AssemblyState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AssemblyState_<ContainerAllocator>::ESTOP_BUTTON_UNPRESSED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AssemblyState_<ContainerAllocator>::ESTOP_BUTTON_PRESSED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AssemblyState_<ContainerAllocator>::ESTOP_BUTTON_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AssemblyState_<ContainerAllocator>::ESTOP_BUTTON_RELEASED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AssemblyState_<ContainerAllocator>::ESTOP_SOURCE_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AssemblyState_<ContainerAllocator>::ESTOP_SOURCE_USER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AssemblyState_<ContainerAllocator>::ESTOP_SOURCE_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AssemblyState_<ContainerAllocator>::ESTOP_SOURCE_FAULT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AssemblyState_<ContainerAllocator>::ESTOP_SOURCE_BRAIN;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__STRUCT_HPP_
