// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:srv/BridgePublishersForce.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__STRUCT_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__srv__BridgePublishersForce_Request __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__srv__BridgePublishersForce_Request __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BridgePublishersForce_Request_
{
  using Type = BridgePublishersForce_Request_<ContainerAllocator>;

  explicit BridgePublishersForce_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_user = "";
      this->right_user = "";
    }
  }

  explicit BridgePublishersForce_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : left_user(_alloc),
    right_user(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_user = "";
      this->right_user = "";
    }
  }

  // field types and members
  using _left_user_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _left_user_type left_user;
  using _right_user_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _right_user_type right_user;

  // setters for named parameter idiom
  Type & set__left_user(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->left_user = _arg;
    return *this;
  }
  Type & set__right_user(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->right_user = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__srv__BridgePublishersForce_Request
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__srv__BridgePublishersForce_Request
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersForce_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BridgePublishersForce_Request_ & other) const
  {
    if (this->left_user != other.left_user) {
      return false;
    }
    if (this->right_user != other.right_user) {
      return false;
    }
    return true;
  }
  bool operator!=(const BridgePublishersForce_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BridgePublishersForce_Request_

// alias to use template instance with default allocator
using BridgePublishersForce_Request =
  baxter_core_msgs::srv::BridgePublishersForce_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace baxter_core_msgs


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__srv__BridgePublishersForce_Response __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__srv__BridgePublishersForce_Response __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BridgePublishersForce_Response_
{
  using Type = BridgePublishersForce_Response_<ContainerAllocator>;

  explicit BridgePublishersForce_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit BridgePublishersForce_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__srv__BridgePublishersForce_Response
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__srv__BridgePublishersForce_Response
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersForce_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BridgePublishersForce_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const BridgePublishersForce_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BridgePublishersForce_Response_

// alias to use template instance with default allocator
using BridgePublishersForce_Response =
  baxter_core_msgs::srv::BridgePublishersForce_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace baxter_core_msgs

namespace baxter_core_msgs
{

namespace srv
{

struct BridgePublishersForce
{
  using Request = baxter_core_msgs::srv::BridgePublishersForce_Request;
  using Response = baxter_core_msgs::srv::BridgePublishersForce_Response;
};

}  // namespace srv

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__STRUCT_HPP_
