// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/EndEffectorState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__EndEffectorState __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__EndEffectorState __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EndEffectorState_
{
  using Type = EndEffectorState_<ContainerAllocator>;

  explicit EndEffectorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->enabled = 0;
      this->calibrated = 0;
      this->ready = 0;
      this->moving = 0;
      this->gripping = 0;
      this->missed = 0;
      this->error = 0;
      this->reverse = 0;
      this->position = 0.0f;
      this->force = 0.0f;
      this->state = "";
      this->command = "";
      this->command_sender = "";
      this->command_sequence = 0ul;
    }
  }

  explicit EndEffectorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init),
    state(_alloc),
    command(_alloc),
    command_sender(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->enabled = 0;
      this->calibrated = 0;
      this->ready = 0;
      this->moving = 0;
      this->gripping = 0;
      this->missed = 0;
      this->error = 0;
      this->reverse = 0;
      this->position = 0.0f;
      this->force = 0.0f;
      this->state = "";
      this->command = "";
      this->command_sender = "";
      this->command_sequence = 0ul;
    }
  }

  // field types and members
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _id_type =
    uint32_t;
  _id_type id;
  using _enabled_type =
    uint8_t;
  _enabled_type enabled;
  using _calibrated_type =
    uint8_t;
  _calibrated_type calibrated;
  using _ready_type =
    uint8_t;
  _ready_type ready;
  using _moving_type =
    uint8_t;
  _moving_type moving;
  using _gripping_type =
    uint8_t;
  _gripping_type gripping;
  using _missed_type =
    uint8_t;
  _missed_type missed;
  using _error_type =
    uint8_t;
  _error_type error;
  using _reverse_type =
    uint8_t;
  _reverse_type reverse;
  using _position_type =
    float;
  _position_type position;
  using _force_type =
    float;
  _force_type force;
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;
  using _command_sender_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_sender_type command_sender;
  using _command_sequence_type =
    uint32_t;
  _command_sequence_type command_sequence;

  // setters for named parameter idiom
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__enabled(
    const uint8_t & _arg)
  {
    this->enabled = _arg;
    return *this;
  }
  Type & set__calibrated(
    const uint8_t & _arg)
  {
    this->calibrated = _arg;
    return *this;
  }
  Type & set__ready(
    const uint8_t & _arg)
  {
    this->ready = _arg;
    return *this;
  }
  Type & set__moving(
    const uint8_t & _arg)
  {
    this->moving = _arg;
    return *this;
  }
  Type & set__gripping(
    const uint8_t & _arg)
  {
    this->gripping = _arg;
    return *this;
  }
  Type & set__missed(
    const uint8_t & _arg)
  {
    this->missed = _arg;
    return *this;
  }
  Type & set__error(
    const uint8_t & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__reverse(
    const uint8_t & _arg)
  {
    this->reverse = _arg;
    return *this;
  }
  Type & set__position(
    const float & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__force(
    const float & _arg)
  {
    this->force = _arg;
    return *this;
  }
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__command_sender(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command_sender = _arg;
    return *this;
  }
  Type & set__command_sequence(
    const uint32_t & _arg)
  {
    this->command_sequence = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATE_FALSE =
    0u;
  static constexpr uint8_t STATE_TRUE =
    1u;
  static constexpr uint8_t STATE_UNKNOWN =
    2u;
  static constexpr float POSITION_CLOSED =
    0.0;
  static constexpr float POSITION_OPEN =
    100.0;
  static constexpr float FORCE_MIN =
    0.0;
  static constexpr float FORCE_MAX =
    100.0;

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__EndEffectorState
    std::shared_ptr<baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__EndEffectorState
    std::shared_ptr<baxter_core_msgs::msg::EndEffectorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EndEffectorState_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->enabled != other.enabled) {
      return false;
    }
    if (this->calibrated != other.calibrated) {
      return false;
    }
    if (this->ready != other.ready) {
      return false;
    }
    if (this->moving != other.moving) {
      return false;
    }
    if (this->gripping != other.gripping) {
      return false;
    }
    if (this->missed != other.missed) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->reverse != other.reverse) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->force != other.force) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->command_sender != other.command_sender) {
      return false;
    }
    if (this->command_sequence != other.command_sequence) {
      return false;
    }
    return true;
  }
  bool operator!=(const EndEffectorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EndEffectorState_

// alias to use template instance with default allocator
using EndEffectorState =
  baxter_core_msgs::msg::EndEffectorState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t EndEffectorState_<ContainerAllocator>::STATE_FALSE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t EndEffectorState_<ContainerAllocator>::STATE_TRUE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t EndEffectorState_<ContainerAllocator>::STATE_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr float EndEffectorState_<ContainerAllocator>::POSITION_CLOSED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr float EndEffectorState_<ContainerAllocator>::POSITION_OPEN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr float EndEffectorState_<ContainerAllocator>::FORCE_MIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr float EndEffectorState_<ContainerAllocator>::FORCE_MAX;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__STRUCT_HPP_
