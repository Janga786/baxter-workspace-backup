// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/CameraControl.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__CameraControl __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__CameraControl __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraControl_
{
  using Type = CameraControl_<ContainerAllocator>;

  explicit CameraControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->value = 0l;
    }
  }

  explicit CameraControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->value = 0l;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _value_type =
    int32_t;
  _value_type value;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__value(
    const int32_t & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t CAMERA_CONTROL_EXPOSURE =
    100;
  static constexpr int32_t CAMERA_CONTROL_GAIN =
    101;
  static constexpr int32_t CAMERA_CONTROL_WHITE_BALANCE_R =
    102;
  static constexpr int32_t CAMERA_CONTROL_WHITE_BALANCE_G =
    103;
  static constexpr int32_t CAMERA_CONTROL_WHITE_BALANCE_B =
    104;
  static constexpr int32_t CAMERA_CONTROL_WINDOW_X =
    105;
  static constexpr int32_t CAMERA_CONTROL_WINDOW_Y =
    106;
  static constexpr int32_t CAMERA_CONTROL_FLIP =
    107;
  static constexpr int32_t CAMERA_CONTROL_MIRROR =
    108;
  static constexpr int32_t CAMERA_CONTROL_RESOLUTION_HALF =
    109;

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::CameraControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::CameraControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::CameraControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::CameraControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::CameraControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::CameraControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::CameraControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::CameraControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::CameraControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::CameraControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__CameraControl
    std::shared_ptr<baxter_core_msgs::msg::CameraControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__CameraControl
    std::shared_ptr<baxter_core_msgs::msg::CameraControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraControl_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraControl_

// alias to use template instance with default allocator
using CameraControl =
  baxter_core_msgs::msg::CameraControl_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_EXPOSURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_GAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_WHITE_BALANCE_R;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_WHITE_BALANCE_G;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_WHITE_BALANCE_B;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_WINDOW_X;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_WINDOW_Y;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_FLIP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_MIRROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t CameraControl_<ContainerAllocator>::CAMERA_CONTROL_RESOLUTION_HALF;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_HPP_
