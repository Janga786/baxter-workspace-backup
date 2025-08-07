// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from baxter_core_msgs:msg/CameraSettings.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__STRUCT_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'controls'
#include "baxter_core_msgs/msg/detail/camera_control__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__baxter_core_msgs__msg__CameraSettings __attribute__((deprecated))
#else
# define DEPRECATED__baxter_core_msgs__msg__CameraSettings __declspec(deprecated)
#endif

namespace baxter_core_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraSettings_
{
  using Type = CameraSettings_<ContainerAllocator>;

  explicit CameraSettings_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0l;
      this->height = 0l;
      this->fps = 0.0f;
    }
  }

  explicit CameraSettings_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0l;
      this->height = 0l;
      this->fps = 0.0f;
    }
  }

  // field types and members
  using _width_type =
    int32_t;
  _width_type width;
  using _height_type =
    int32_t;
  _height_type height;
  using _fps_type =
    float;
  _fps_type fps;
  using _controls_type =
    std::vector<baxter_core_msgs::msg::CameraControl_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<baxter_core_msgs::msg::CameraControl_<ContainerAllocator>>>;
  _controls_type controls;

  // setters for named parameter idiom
  Type & set__width(
    const int32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const int32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__fps(
    const float & _arg)
  {
    this->fps = _arg;
    return *this;
  }
  Type & set__controls(
    const std::vector<baxter_core_msgs::msg::CameraControl_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<baxter_core_msgs::msg::CameraControl_<ContainerAllocator>>> & _arg)
  {
    this->controls = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    baxter_core_msgs::msg::CameraSettings_<ContainerAllocator> *;
  using ConstRawPtr =
    const baxter_core_msgs::msg::CameraSettings_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::CameraSettings_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<baxter_core_msgs::msg::CameraSettings_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::CameraSettings_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::CameraSettings_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      baxter_core_msgs::msg::CameraSettings_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<baxter_core_msgs::msg::CameraSettings_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::CameraSettings_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<baxter_core_msgs::msg::CameraSettings_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__baxter_core_msgs__msg__CameraSettings
    std::shared_ptr<baxter_core_msgs::msg::CameraSettings_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__baxter_core_msgs__msg__CameraSettings
    std::shared_ptr<baxter_core_msgs::msg::CameraSettings_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraSettings_ & other) const
  {
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->fps != other.fps) {
      return false;
    }
    if (this->controls != other.controls) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraSettings_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraSettings_

// alias to use template instance with default allocator
using CameraSettings =
  baxter_core_msgs::msg::CameraSettings_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__STRUCT_HPP_
