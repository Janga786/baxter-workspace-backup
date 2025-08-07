// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/HeadPanCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__HeadPanCommand __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__HeadPanCommand __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HeadPanCommand_
{
  using Type = HeadPanCommand_<ContainerAllocator>;

  explicit HeadPanCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = 0.0f;
      this->speed_ratio = 0.0f;
      this->enable_pan_request = 0;
    }
  }

  explicit HeadPanCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = 0.0f;
      this->speed_ratio = 0.0f;
      this->enable_pan_request = 0;
    }
  }

  // field types and members
  using _target_type =
    float;
  _target_type target;
  using _speed_ratio_type =
    float;
  _speed_ratio_type speed_ratio;
  using _enable_pan_request_type =
    uint8_t;
  _enable_pan_request_type enable_pan_request;

  // setters for named parameter idiom
  Type & set__target(
    const float & _arg)
  {
    this->target = _arg;
    return *this;
  }
  Type & set__speed_ratio(
    const float & _arg)
  {
    this->speed_ratio = _arg;
    return *this;
  }
  Type & set__enable_pan_request(
    const uint8_t & _arg)
  {
    this->enable_pan_request = _arg;
    return *this;
  }

  // constant declarations
  static constexpr float MAX_SPEED_RATIO =
    1.0;
  static constexpr float MIN_SPEED_RATIO =
    0.0;
  static constexpr uint8_t REQUEST_PAN_DISABLE =
    0u;
  static constexpr uint8_t REQUEST_PAN_ENABLE =
    1u;
  static constexpr uint8_t REQUEST_PAN_VOID =
    2u;

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__HeadPanCommand
    std::shared_ptr<baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__HeadPanCommand
    std::shared_ptr<baxter_core_msgs::msg::HeadPanCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HeadPanCommand_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    if (this->speed_ratio != other.speed_ratio) {
      return false;
    }
    if (this->enable_pan_request != other.enable_pan_request) {
      return false;
    }
    return true;
  }
  bool operator!=(const HeadPanCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HeadPanCommand_

// alias to use template instance with default allocator
using HeadPanCommand =
  baxter_core_msgs::msg::HeadPanCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr float HeadPanCommand_<ContainerAllocator>::MAX_SPEED_RATIO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr float HeadPanCommand_<ContainerAllocator>::MIN_SPEED_RATIO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t HeadPanCommand_<ContainerAllocator>::REQUEST_PAN_DISABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t HeadPanCommand_<ContainerAllocator>::REQUEST_PAN_ENABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t HeadPanCommand_<ContainerAllocator>::REQUEST_PAN_VOID;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__STRUCT_HPP_
