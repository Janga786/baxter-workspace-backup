// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:srv/SolvePositionIK.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__STRUCT_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose_stamp'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'seed_angles'
#include "sensor_msgs/msg/detail/joint_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__srv__SolvePositionIK_Request __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__srv__SolvePositionIK_Request __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SolvePositionIK_Request_
{
  using Type = SolvePositionIK_Request_<ContainerAllocator>;

  explicit SolvePositionIK_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seed_mode = 0;
    }
  }

  explicit SolvePositionIK_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seed_mode = 0;
    }
  }

  // field types and members
  using _pose_stamp_type =
    std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>;
  _pose_stamp_type pose_stamp;
  using _seed_angles_type =
    std::vector<sensor_msgs::msg::JointState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::JointState_<ContainerAllocator>>>;
  _seed_angles_type seed_angles;
  using _seed_mode_type =
    uint8_t;
  _seed_mode_type seed_mode;

  // setters for named parameter idiom
  Type & set__pose_stamp(
    const std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>> & _arg)
  {
    this->pose_stamp = _arg;
    return *this;
  }
  Type & set__seed_angles(
    const std::vector<sensor_msgs::msg::JointState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::JointState_<ContainerAllocator>>> & _arg)
  {
    this->seed_angles = _arg;
    return *this;
  }
  Type & set__seed_mode(
    const uint8_t & _arg)
  {
    this->seed_mode = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SEED_AUTO =
    0u;
  static constexpr uint8_t SEED_USER =
    1u;
  static constexpr uint8_t SEED_CURRENT =
    2u;
  static constexpr uint8_t SEED_NS_MAP =
    3u;

  // pointer types
  using RawPtr =
    baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__srv__SolvePositionIK_Request
    std::shared_ptr<baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__srv__SolvePositionIK_Request
    std::shared_ptr<baxter_core_msgs::srv::SolvePositionIK_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SolvePositionIK_Request_ & other) const
  {
    if (this->pose_stamp != other.pose_stamp) {
      return false;
    }
    if (this->seed_angles != other.seed_angles) {
      return false;
    }
    if (this->seed_mode != other.seed_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const SolvePositionIK_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SolvePositionIK_Request_

// alias to use template instance with default allocator
using SolvePositionIK_Request =
  baxter_core_msgs::srv::SolvePositionIK_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolvePositionIK_Request_<ContainerAllocator>::SEED_AUTO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolvePositionIK_Request_<ContainerAllocator>::SEED_USER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolvePositionIK_Request_<ContainerAllocator>::SEED_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolvePositionIK_Request_<ContainerAllocator>::SEED_NS_MAP;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace baxter_core_msgs


// Include directives for member types
// Member 'joints'
// already included above
// #include "sensor_msgs/msg/detail/joint_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__srv__SolvePositionIK_Response __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__srv__SolvePositionIK_Response __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SolvePositionIK_Response_
{
  using Type = SolvePositionIK_Response_<ContainerAllocator>;

  explicit SolvePositionIK_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SolvePositionIK_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _joints_type =
    std::vector<sensor_msgs::msg::JointState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::JointState_<ContainerAllocator>>>;
  _joints_type joints;
  using _is_valid_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _is_valid_type is_valid;
  using _result_type_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _result_type_type result_type;

  // setters for named parameter idiom
  Type & set__joints(
    const std::vector<sensor_msgs::msg::JointState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::JointState_<ContainerAllocator>>> & _arg)
  {
    this->joints = _arg;
    return *this;
  }
  Type & set__is_valid(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->is_valid = _arg;
    return *this;
  }
  Type & set__result_type(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->result_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t RESULT_INVALID =
    0u;

  // pointer types
  using RawPtr =
    baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__srv__SolvePositionIK_Response
    std::shared_ptr<baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__srv__SolvePositionIK_Response
    std::shared_ptr<baxter_core_msgs::srv::SolvePositionIK_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SolvePositionIK_Response_ & other) const
  {
    if (this->joints != other.joints) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    if (this->result_type != other.result_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const SolvePositionIK_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SolvePositionIK_Response_

// alias to use template instance with default allocator
using SolvePositionIK_Response =
  baxter_core_msgs::srv::SolvePositionIK_Response_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolvePositionIK_Response_<ContainerAllocator>::RESULT_INVALID;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace baxter_core_msgs

namespace baxter_core_msgs
{

namespace srv
{

struct SolvePositionIK
{
  using Request = baxter_core_msgs::srv::SolvePositionIK_Request;
  using Response = baxter_core_msgs::srv::SolvePositionIK_Response;
};

}  // namespace srv

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__STRUCT_HPP_
