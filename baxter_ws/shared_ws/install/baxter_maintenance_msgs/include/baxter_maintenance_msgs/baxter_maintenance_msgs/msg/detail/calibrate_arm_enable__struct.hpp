// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmEnable.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__STRUCT_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'data'
#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_maintenance_msgs__msg__CalibrateArmEnable __attribute__((deprecated))
#else
# define DEPRECATED__baxter_maintenance_msgs__msg__CalibrateArmEnable __declspec(deprecated)
#endif

namespace baxter_maintenance_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CalibrateArmEnable_
{
  using Type = CalibrateArmEnable_<ContainerAllocator>;

  explicit CalibrateArmEnable_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_enabled = false;
      this->uid = "";
    }
  }

  explicit CalibrateArmEnable_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : uid(_alloc),
    data(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_enabled = false;
      this->uid = "";
    }
  }

  // field types and members
  using _is_enabled_type =
    bool;
  _is_enabled_type is_enabled;
  using _uid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _uid_type uid;
  using _data_type =
    baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__is_enabled(
    const bool & _arg)
  {
    this->is_enabled = _arg;
    return *this;
  }
  Type & set__uid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->uid = _arg;
    return *this;
  }
  Type & set__data(
    const baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_maintenance_msgs__msg__CalibrateArmEnable
    std::shared_ptr<baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_maintenance_msgs__msg__CalibrateArmEnable
    std::shared_ptr<baxter_maintenance_msgs::msg::CalibrateArmEnable_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrateArmEnable_ & other) const
  {
    if (this->is_enabled != other.is_enabled) {
      return false;
    }
    if (this->uid != other.uid) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrateArmEnable_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrateArmEnable_

// alias to use template instance with default allocator
using CalibrateArmEnable =
  baxter_maintenance_msgs::msg::CalibrateArmEnable_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__STRUCT_HPP_
