// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:srv/BridgePublishersAuth.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__STRUCT_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__srv__BridgePublishersAuth_Request __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__srv__BridgePublishersAuth_Request __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BridgePublishersAuth_Request_
{
  using Type = BridgePublishersAuth_Request_<ContainerAllocator>;

  explicit BridgePublishersAuth_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic = "";
      this->user = "";
    }
  }

  explicit BridgePublishersAuth_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : topic(_alloc),
    user(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic = "";
      this->user = "";
    }
  }

  // field types and members
  using _topic_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _topic_type topic;
  using _user_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _user_type user;

  // setters for named parameter idiom
  Type & set__topic(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->topic = _arg;
    return *this;
  }
  Type & set__user(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->user = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__srv__BridgePublishersAuth_Request
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__srv__BridgePublishersAuth_Request
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BridgePublishersAuth_Request_ & other) const
  {
    if (this->topic != other.topic) {
      return false;
    }
    if (this->user != other.user) {
      return false;
    }
    return true;
  }
  bool operator!=(const BridgePublishersAuth_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BridgePublishersAuth_Request_

// alias to use template instance with default allocator
using BridgePublishersAuth_Request =
  baxter_core_msgs::srv::BridgePublishersAuth_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace baxter_core_msgs


// Include directives for member types
// Member 'publishers'
#include "baxter_core_msgs/msg/detail/bridge_publisher__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__srv__BridgePublishersAuth_Response __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__srv__BridgePublishersAuth_Response __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BridgePublishersAuth_Response_
{
  using Type = BridgePublishersAuth_Response_<ContainerAllocator>;

  explicit BridgePublishersAuth_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->forced_left = "";
      this->forced_right = "";
    }
  }

  explicit BridgePublishersAuth_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : forced_left(_alloc),
    forced_right(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->forced_left = "";
      this->forced_right = "";
    }
  }

  // field types and members
  using _publishers_type =
    std::vector<baxter_core_msgs::msg::BridgePublisher_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<baxter_core_msgs::msg::BridgePublisher_<ContainerAllocator>>>;
  _publishers_type publishers;
  using _forced_left_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _forced_left_type forced_left;
  using _forced_right_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _forced_right_type forced_right;

  // setters for named parameter idiom
  Type & set__publishers(
    const std::vector<baxter_core_msgs::msg::BridgePublisher_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<baxter_core_msgs::msg::BridgePublisher_<ContainerAllocator>>> & _arg)
  {
    this->publishers = _arg;
    return *this;
  }
  Type & set__forced_left(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->forced_left = _arg;
    return *this;
  }
  Type & set__forced_right(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->forced_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__srv__BridgePublishersAuth_Response
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__srv__BridgePublishersAuth_Response
    std::shared_ptr<baxter_core_msgs::srv::BridgePublishersAuth_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BridgePublishersAuth_Response_ & other) const
  {
    if (this->publishers != other.publishers) {
      return false;
    }
    if (this->forced_left != other.forced_left) {
      return false;
    }
    if (this->forced_right != other.forced_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const BridgePublishersAuth_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BridgePublishersAuth_Response_

// alias to use template instance with default allocator
using BridgePublishersAuth_Response =
  baxter_core_msgs::srv::BridgePublishersAuth_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace baxter_core_msgs

namespace baxter_core_msgs
{

namespace srv
{

struct BridgePublishersAuth
{
  using Request = baxter_core_msgs::srv::BridgePublishersAuth_Request;
  using Response = baxter_core_msgs::srv::BridgePublishersAuth_Response;
};

}  // namespace srv

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__STRUCT_HPP_
