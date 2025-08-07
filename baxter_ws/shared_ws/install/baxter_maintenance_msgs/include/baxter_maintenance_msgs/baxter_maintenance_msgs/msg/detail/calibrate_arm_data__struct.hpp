// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmData.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__STRUCT_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_maintenance_msgs__msg__CalibrateArmData __attribute__((deprecated))
#else
# define DEPRECATED__baxter_maintenance_msgs__msg__CalibrateArmData __declspec(deprecated)
#endif

namespace baxter_maintenance_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CalibrateArmData_
{
  using Type = CalibrateArmData_<ContainerAllocator>;

  explicit CalibrateArmData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->suppress_write_to_file = false;
    }
  }

  explicit CalibrateArmData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->suppress_write_to_file = false;
    }
  }

  // field types and members
  using _suppress_write_to_file_type =
    bool;
  _suppress_write_to_file_type suppress_write_to_file;

  // setters for named parameter idiom
  Type & set__suppress_write_to_file(
    const bool & _arg)
  {
    this->suppress_write_to_file = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_maintenance_msgs__msg__CalibrateArmData
    std::shared_ptr<baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_maintenance_msgs__msg__CalibrateArmData
    std::shared_ptr<baxter_maintenance_msgs::msg::CalibrateArmData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrateArmData_ & other) const
  {
    if (this->suppress_write_to_file != other.suppress_write_to_file) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrateArmData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrateArmData_

// alias to use template instance with default allocator
using CalibrateArmData =
  baxter_maintenance_msgs::msg::CalibrateArmData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__STRUCT_HPP_
