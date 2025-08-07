// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/HeadState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__HeadState __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__HeadState __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HeadState_
{
  using Type = HeadState_<ContainerAllocator>;

  explicit HeadState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pan = 0.0f;
      this->is_turning = false;
      this->is_nodding = false;
      this->is_pan_enabled = false;
    }
  }

  explicit HeadState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pan = 0.0f;
      this->is_turning = false;
      this->is_nodding = false;
      this->is_pan_enabled = false;
    }
  }

  // field types and members
  using _pan_type =
    float;
  _pan_type pan;
  using _is_turning_type =
    bool;
  _is_turning_type is_turning;
  using _is_nodding_type =
    bool;
  _is_nodding_type is_nodding;
  using _is_pan_enabled_type =
    bool;
  _is_pan_enabled_type is_pan_enabled;

  // setters for named parameter idiom
  Type & set__pan(
    const float & _arg)
  {
    this->pan = _arg;
    return *this;
  }
  Type & set__is_turning(
    const bool & _arg)
  {
    this->is_turning = _arg;
    return *this;
  }
  Type & set__is_nodding(
    const bool & _arg)
  {
    this->is_nodding = _arg;
    return *this;
  }
  Type & set__is_pan_enabled(
    const bool & _arg)
  {
    this->is_pan_enabled = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::HeadState_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::HeadState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::HeadState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::HeadState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::HeadState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::HeadState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::HeadState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::HeadState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::HeadState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::HeadState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__HeadState
    std::shared_ptr<baxter_core_msgs::msg::HeadState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__HeadState
    std::shared_ptr<baxter_core_msgs::msg::HeadState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HeadState_ & other) const
  {
    if (this->pan != other.pan) {
      return false;
    }
    if (this->is_turning != other.is_turning) {
      return false;
    }
    if (this->is_nodding != other.is_nodding) {
      return false;
    }
    if (this->is_pan_enabled != other.is_pan_enabled) {
      return false;
    }
    return true;
  }
  bool operator!=(const HeadState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HeadState_

// alias to use template instance with default allocator
using HeadState =
  baxter_core_msgs::msg::HeadState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__STRUCT_HPP_
