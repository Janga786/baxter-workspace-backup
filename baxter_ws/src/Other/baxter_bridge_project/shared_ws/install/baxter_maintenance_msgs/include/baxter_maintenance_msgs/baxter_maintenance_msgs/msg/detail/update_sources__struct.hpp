// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_maintenance_msgs:msg/UpdateSources.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__STRUCT_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'sources'
#include "baxter_maintenance_msgs/msg/detail/update_source__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_maintenance_msgs__msg__UpdateSources __attribute__((deprecated))
#else
# define DEPRECATED__baxter_maintenance_msgs__msg__UpdateSources __declspec(deprecated)
#endif

namespace baxter_maintenance_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UpdateSources_
{
  using Type = UpdateSources_<ContainerAllocator>;

  explicit UpdateSources_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
    }
  }

  explicit UpdateSources_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : uuid(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
    }
  }

  // field types and members
  using _uuid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _uuid_type uuid;
  using _sources_type =
    std::vector<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>>>;
  _sources_type sources;

  // setters for named parameter idiom
  Type & set__uuid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->uuid = _arg;
    return *this;
  }
  Type & set__sources(
    const std::vector<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<baxter_maintenance_msgs::msg::UpdateSource_<ContainerAllocator>>> & _arg)
  {
    this->sources = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_maintenance_msgs__msg__UpdateSources
    std::shared_ptr<baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_maintenance_msgs__msg__UpdateSources
    std::shared_ptr<baxter_maintenance_msgs::msg::UpdateSources_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateSources_ & other) const
  {
    if (this->uuid != other.uuid) {
      return false;
    }
    if (this->sources != other.sources) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateSources_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateSources_

// alias to use template instance with default allocator
using UpdateSources =
  baxter_maintenance_msgs::msg::UpdateSources_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__STRUCT_HPP_
