// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/NavigatorStates.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATES__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'states'
#include "baxter_core_msgs/msg/detail/navigator_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__NavigatorStates __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__NavigatorStates __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavigatorStates_
{
  using Type = NavigatorStates_<ContainerAllocator>;

  explicit NavigatorStates_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit NavigatorStates_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _names_type names;
  using _states_type =
    std::vector<baxter_core_msgs::msg::NavigatorState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<baxter_core_msgs::msg::NavigatorState_<ContainerAllocator>>>;
  _states_type states;

  // setters for named parameter idiom
  Type & set__names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->names = _arg;
    return *this;
  }
  Type & set__states(
    const std::vector<baxter_core_msgs::msg::NavigatorState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<baxter_core_msgs::msg::NavigatorState_<ContainerAllocator>>> & _arg)
  {
    this->states = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__NavigatorStates
    std::shared_ptr<baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__NavigatorStates
    std::shared_ptr<baxter_core_msgs::msg::NavigatorStates_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigatorStates_ & other) const
  {
    if (this->names != other.names) {
      return false;
    }
    if (this->states != other.states) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigatorStates_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigatorStates_

// alias to use template instance with default allocator
using NavigatorStates =
  baxter_core_msgs::msg::NavigatorStates_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATES__STRUCT_HPP_
