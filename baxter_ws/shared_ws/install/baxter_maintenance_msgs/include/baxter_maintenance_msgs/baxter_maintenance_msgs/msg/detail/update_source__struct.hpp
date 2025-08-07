// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_maintenance_msgs:msg/UpdateSource.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__STRUCT_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_maintenance_msgs__msg__UpdateSource __attribute__((deprecated))
#else
# define DEPRECATED__baxter_maintenance_msgs__msg__UpdateSource __declspec(deprecated)
#endif

namespace baxter_maintenance_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UpdateSource_
{
  using Type = UpdateSource_<ContainerAllocator>;

  explicit UpdateSource_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->devname = "";
      this->filename = "";
      this->version = "";
      this->uuid = "";
    }
  }

  explicit UpdateSource_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : devname(_alloc),
    filename(_alloc),
    version(_alloc),
    uuid(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->devname = "";
      this->filename = "";
      this->version = "";
      this->uuid = "";
    }
  }

  // field types and members
  using _devname_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _devname_type devname;
  using _filename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _filename_type filename;
  using _version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _version_type version;
  using _uuid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _uuid_type uuid;

  // setters for named parameter idiom
  Type & set__devname(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->devname = _arg;
    return *this;
  }
  Type & set__filename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->filename = _arg;
    return *this;
  }
  Type & set__version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__uuid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->uuid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_maintenance_msgs__msg__UpdateSource
    std::shared_ptr<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_maintenance_msgs__msg__UpdateSource
    std::shared_ptr<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateSource_ & other) const
  {
    if (this->devname != other.devname) {
      return false;
    }
    if (this->filename != other.filename) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->uuid != other.uuid) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateSource_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateSource_

// alias to use template instance with default allocator
using UpdateSource =
  baxter_maintenance_msgs::msg::UpdateSource_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__STRUCT_HPP_
