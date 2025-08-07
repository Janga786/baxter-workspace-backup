// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/URDFConfiguration.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__URDFConfiguration __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__URDFConfiguration __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct URDFConfiguration_
{
  using Type = URDFConfiguration_<ContainerAllocator>;

  explicit URDFConfiguration_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->link = "";
      this->joint = "";
      this->urdf = "";
    }
  }

  explicit URDFConfiguration_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time(_alloc, _init),
    link(_alloc),
    joint(_alloc),
    urdf(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->link = "";
      this->joint = "";
      this->urdf = "";
    }
  }

  // field types and members
  using _time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _time_type time;
  using _link_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _link_type link;
  using _joint_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_type joint;
  using _urdf_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _urdf_type urdf;

  // setters for named parameter idiom
  Type & set__time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__link(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->link = _arg;
    return *this;
  }
  Type & set__joint(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->joint = _arg;
    return *this;
  }
  Type & set__urdf(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->urdf = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__URDFConfiguration
    std::shared_ptr<baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__URDFConfiguration
    std::shared_ptr<baxter_core_msgs::msg::URDFConfiguration_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const URDFConfiguration_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->link != other.link) {
      return false;
    }
    if (this->joint != other.joint) {
      return false;
    }
    if (this->urdf != other.urdf) {
      return false;
    }
    return true;
  }
  bool operator!=(const URDFConfiguration_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct URDFConfiguration_

// alias to use template instance with default allocator
using URDFConfiguration =
  baxter_core_msgs::msg::URDFConfiguration_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__STRUCT_HPP_
