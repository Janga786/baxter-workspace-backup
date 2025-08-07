// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_maintenance_msgs:msg/TareData.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_DATA__STRUCT_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_maintenance_msgs__msg__TareData __attribute__((deprecated))
#else
# define DEPRECATED__baxter_maintenance_msgs__msg__TareData __declspec(deprecated)
#endif

namespace baxter_maintenance_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TareData_
{
  using Type = TareData_<ContainerAllocator>;

  explicit TareData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tune_gravity_spring = false;
    }
  }

  explicit TareData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tune_gravity_spring = false;
    }
  }

  // field types and members
  using _tune_gravity_spring_type =
    bool;
  _tune_gravity_spring_type tune_gravity_spring;

  // setters for named parameter idiom
  Type & set__tune_gravity_spring(
    const bool & _arg)
  {
    this->tune_gravity_spring = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_maintenance_msgs::msg::TareData_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_maintenance_msgs::msg::TareData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::TareData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::TareData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::TareData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::TareData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::TareData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::TareData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::TareData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::TareData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_maintenance_msgs__msg__TareData
    std::shared_ptr<baxter_maintenance_msgs::msg::TareData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_maintenance_msgs__msg__TareData
    std::shared_ptr<baxter_maintenance_msgs::msg::TareData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TareData_ & other) const
  {
    if (this->tune_gravity_spring != other.tune_gravity_spring) {
      return false;
    }
    return true;
  }
  bool operator!=(const TareData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TareData_

// alias to use template instance with default allocator
using TareData =
  baxter_maintenance_msgs::msg::TareData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_DATA__STRUCT_HPP_
