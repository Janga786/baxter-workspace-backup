// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/RobustControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__RobustControllerStatus __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__RobustControllerStatus __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobustControllerStatus_
{
  using Type = RobustControllerStatus_<ContainerAllocator>;

  explicit RobustControllerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_enabled = false;
      this->complete = 0l;
      this->control_uid = "";
      this->timed_out = false;
    }
  }

  explicit RobustControllerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : control_uid(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_enabled = false;
      this->complete = 0l;
      this->control_uid = "";
      this->timed_out = false;
    }
  }

  // field types and members
  using _is_enabled_type =
    bool;
  _is_enabled_type is_enabled;
  using _complete_type =
    int32_t;
  _complete_type complete;
  using _control_uid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _control_uid_type control_uid;
  using _timed_out_type =
    bool;
  _timed_out_type timed_out;
  using _error_codes_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _error_codes_type error_codes;
  using _labels_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _labels_type labels;

  // setters for named parameter idiom
  Type & set__is_enabled(
    const bool & _arg)
  {
    this->is_enabled = _arg;
    return *this;
  }
  Type & set__complete(
    const int32_t & _arg)
  {
    this->complete = _arg;
    return *this;
  }
  Type & set__control_uid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->control_uid = _arg;
    return *this;
  }
  Type & set__timed_out(
    const bool & _arg)
  {
    this->timed_out = _arg;
    return *this;
  }
  Type & set__error_codes(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->error_codes = _arg;
    return *this;
  }
  Type & set__labels(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->labels = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t NOT_COMPLETE =
    0;
  static constexpr int32_t COMPLETE_W_FAILURE =
    1;
  static constexpr int32_t COMPLETE_W_SUCCESS =
    2;

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__RobustControllerStatus
    std::shared_ptr<baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__RobustControllerStatus
    std::shared_ptr<baxter_core_msgs::msg::RobustControllerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobustControllerStatus_ & other) const
  {
    if (this->is_enabled != other.is_enabled) {
      return false;
    }
    if (this->complete != other.complete) {
      return false;
    }
    if (this->control_uid != other.control_uid) {
      return false;
    }
    if (this->timed_out != other.timed_out) {
      return false;
    }
    if (this->error_codes != other.error_codes) {
      return false;
    }
    if (this->labels != other.labels) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobustControllerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobustControllerStatus_

// alias to use template instance with default allocator
using RobustControllerStatus =
  baxter_core_msgs::msg::RobustControllerStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t RobustControllerStatus_<ContainerAllocator>::NOT_COMPLETE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t RobustControllerStatus_<ContainerAllocator>::COMPLETE_W_FAILURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t RobustControllerStatus_<ContainerAllocator>::COMPLETE_W_SUCCESS;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__STRUCT_HPP_
