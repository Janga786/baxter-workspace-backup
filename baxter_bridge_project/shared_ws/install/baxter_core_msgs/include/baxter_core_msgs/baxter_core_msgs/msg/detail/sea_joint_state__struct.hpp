// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/SEAJointState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__SEAJointState __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__SEAJointState __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SEAJointState_
{
  using Type = SEAJointState_<ContainerAllocator>;

  explicit SEAJointState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hyst_state = 0.0;
    }
  }

  explicit SEAJointState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hyst_state = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _name_type name;
  using _commanded_position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _commanded_position_type commanded_position;
  using _commanded_velocity_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _commanded_velocity_type commanded_velocity;
  using _commanded_acceleration_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _commanded_acceleration_type commanded_acceleration;
  using _commanded_effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _commanded_effort_type commanded_effort;
  using _actual_position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _actual_position_type actual_position;
  using _actual_velocity_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _actual_velocity_type actual_velocity;
  using _actual_effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _actual_effort_type actual_effort;
  using _gravity_model_effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _gravity_model_effort_type gravity_model_effort;
  using _gravity_only_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _gravity_only_type gravity_only;
  using _hysteresis_model_effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _hysteresis_model_effort_type hysteresis_model_effort;
  using _crosstalk_model_effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _crosstalk_model_effort_type crosstalk_model_effort;
  using _hyst_state_type =
    double;
  _hyst_state_type hyst_state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__commanded_position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->commanded_position = _arg;
    return *this;
  }
  Type & set__commanded_velocity(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->commanded_velocity = _arg;
    return *this;
  }
  Type & set__commanded_acceleration(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->commanded_acceleration = _arg;
    return *this;
  }
  Type & set__commanded_effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->commanded_effort = _arg;
    return *this;
  }
  Type & set__actual_position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->actual_position = _arg;
    return *this;
  }
  Type & set__actual_velocity(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->actual_velocity = _arg;
    return *this;
  }
  Type & set__actual_effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->actual_effort = _arg;
    return *this;
  }
  Type & set__gravity_model_effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->gravity_model_effort = _arg;
    return *this;
  }
  Type & set__gravity_only(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->gravity_only = _arg;
    return *this;
  }
  Type & set__hysteresis_model_effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->hysteresis_model_effort = _arg;
    return *this;
  }
  Type & set__crosstalk_model_effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->crosstalk_model_effort = _arg;
    return *this;
  }
  Type & set__hyst_state(
    const double & _arg)
  {
    this->hyst_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::SEAJointState_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::SEAJointState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::SEAJointState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::SEAJointState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::SEAJointState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::SEAJointState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::SEAJointState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::SEAJointState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::SEAJointState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::SEAJointState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__SEAJointState
    std::shared_ptr<baxter_core_msgs::msg::SEAJointState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__SEAJointState
    std::shared_ptr<baxter_core_msgs::msg::SEAJointState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SEAJointState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->commanded_position != other.commanded_position) {
      return false;
    }
    if (this->commanded_velocity != other.commanded_velocity) {
      return false;
    }
    if (this->commanded_acceleration != other.commanded_acceleration) {
      return false;
    }
    if (this->commanded_effort != other.commanded_effort) {
      return false;
    }
    if (this->actual_position != other.actual_position) {
      return false;
    }
    if (this->actual_velocity != other.actual_velocity) {
      return false;
    }
    if (this->actual_effort != other.actual_effort) {
      return false;
    }
    if (this->gravity_model_effort != other.gravity_model_effort) {
      return false;
    }
    if (this->gravity_only != other.gravity_only) {
      return false;
    }
    if (this->hysteresis_model_effort != other.hysteresis_model_effort) {
      return false;
    }
    if (this->crosstalk_model_effort != other.crosstalk_model_effort) {
      return false;
    }
    if (this->hyst_state != other.hyst_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SEAJointState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SEAJointState_

// alias to use template instance with default allocator
using SEAJointState =
  baxter_core_msgs::msg::SEAJointState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__STRUCT_HPP_
