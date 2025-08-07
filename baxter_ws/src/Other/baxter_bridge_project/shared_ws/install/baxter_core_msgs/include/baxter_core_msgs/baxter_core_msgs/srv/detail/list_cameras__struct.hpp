// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:srv/ListCameras.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__STRUCT_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__srv__ListCameras_Request __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__srv__ListCameras_Request __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ListCameras_Request_
{
  using Type = ListCameras_Request_<ContainerAllocator>;

  explicit ListCameras_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit ListCameras_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__srv__ListCameras_Request
    std::shared_ptr<baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__srv__ListCameras_Request
    std::shared_ptr<baxter_core_msgs::srv::ListCameras_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ListCameras_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const ListCameras_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ListCameras_Request_

// alias to use template instance with default allocator
using ListCameras_Request =
  baxter_core_msgs::srv::ListCameras_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace baxter_core_msgs


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__srv__ListCameras_Response __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__srv__ListCameras_Response __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ListCameras_Response_
{
  using Type = ListCameras_Response_<ContainerAllocator>;

  explicit ListCameras_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ListCameras_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _cameras_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _cameras_type cameras;

  // setters for named parameter idiom
  Type & set__cameras(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->cameras = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__srv__ListCameras_Response
    std::shared_ptr<baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__srv__ListCameras_Response
    std::shared_ptr<baxter_core_msgs::srv::ListCameras_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ListCameras_Response_ & other) const
  {
    if (this->cameras != other.cameras) {
      return false;
    }
    return true;
  }
  bool operator!=(const ListCameras_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ListCameras_Response_

// alias to use template instance with default allocator
using ListCameras_Response =
  baxter_core_msgs::srv::ListCameras_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace baxter_core_msgs

namespace baxter_core_msgs
{

namespace srv
{

struct ListCameras
{
  using Request = baxter_core_msgs::srv::ListCameras_Request;
  using Response = baxter_core_msgs::srv::ListCameras_Response;
};

}  // namespace srv

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__STRUCT_HPP_
