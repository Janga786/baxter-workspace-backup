// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from baxter_core_msgs:srv/SolvePositionIK.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "baxter_core_msgs/srv/detail/solve_position_ik__rosidl_typesupport_introspection_c.h"
#include "baxter_core_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "baxter_core_msgs/srv/detail/solve_position_ik__functions.h"
#include "baxter_core_msgs/srv/detail/solve_position_ik__struct.h"


// Include directives for member types
// Member `pose_stamp`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `pose_stamp`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `seed_angles`
#include "sensor_msgs/msg/joint_state.h"
// Member `seed_angles`
#include "sensor_msgs/msg/detail/joint_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  baxter_core_msgs__srv__SolvePositionIK_Request__init(message_memory);
}

void baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_fini_function(void * message_memory)
{
  baxter_core_msgs__srv__SolvePositionIK_Request__fini(message_memory);
}

size_t baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Request__pose_stamp(
  const void * untyped_member)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Request__pose_stamp(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Request__pose_stamp(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Request__pose_stamp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Request__pose_stamp(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Request__pose_stamp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Request__pose_stamp(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Request__pose_stamp(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  geometry_msgs__msg__PoseStamped__Sequence__fini(member);
  return geometry_msgs__msg__PoseStamped__Sequence__init(member, size);
}

size_t baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Request__seed_angles(
  const void * untyped_member)
{
  const sensor_msgs__msg__JointState__Sequence * member =
    (const sensor_msgs__msg__JointState__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Request__seed_angles(
  const void * untyped_member, size_t index)
{
  const sensor_msgs__msg__JointState__Sequence * member =
    (const sensor_msgs__msg__JointState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Request__seed_angles(
  void * untyped_member, size_t index)
{
  sensor_msgs__msg__JointState__Sequence * member =
    (sensor_msgs__msg__JointState__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Request__seed_angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sensor_msgs__msg__JointState * item =
    ((const sensor_msgs__msg__JointState *)
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Request__seed_angles(untyped_member, index));
  sensor_msgs__msg__JointState * value =
    (sensor_msgs__msg__JointState *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Request__seed_angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sensor_msgs__msg__JointState * item =
    ((sensor_msgs__msg__JointState *)
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Request__seed_angles(untyped_member, index));
  const sensor_msgs__msg__JointState * value =
    (const sensor_msgs__msg__JointState *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Request__seed_angles(
  void * untyped_member, size_t size)
{
  sensor_msgs__msg__JointState__Sequence * member =
    (sensor_msgs__msg__JointState__Sequence *)(untyped_member);
  sensor_msgs__msg__JointState__Sequence__fini(member);
  return sensor_msgs__msg__JointState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_member_array[3] = {
  {
    "pose_stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__srv__SolvePositionIK_Request, pose_stamp),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Request__pose_stamp,  // size() function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Request__pose_stamp,  // get_const(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Request__pose_stamp,  // get(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Request__pose_stamp,  // fetch(index, &value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Request__pose_stamp,  // assign(index, value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Request__pose_stamp  // resize(index) function pointer
  },
  {
    "seed_angles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__srv__SolvePositionIK_Request, seed_angles),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Request__seed_angles,  // size() function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Request__seed_angles,  // get_const(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Request__seed_angles,  // get(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Request__seed_angles,  // fetch(index, &value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Request__seed_angles,  // assign(index, value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Request__seed_angles  // resize(index) function pointer
  },
  {
    "seed_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__srv__SolvePositionIK_Request, seed_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_members = {
  "baxter_core_msgs__srv",  // message namespace
  "SolvePositionIK_Request",  // message name
  3,  // number of fields
  sizeof(baxter_core_msgs__srv__SolvePositionIK_Request),
  baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_member_array,  // message members
  baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_type_support_handle = {
  0,
  &baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_core_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, srv, SolvePositionIK_Request)() {
  baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, JointState)();
  if (!baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_type_support_handle.typesupport_identifier) {
    baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &baxter_core_msgs__srv__SolvePositionIK_Request__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "baxter_core_msgs/srv/detail/solve_position_ik__rosidl_typesupport_introspection_c.h"
// already included above
// #include "baxter_core_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "baxter_core_msgs/srv/detail/solve_position_ik__functions.h"
// already included above
// #include "baxter_core_msgs/srv/detail/solve_position_ik__struct.h"


// Include directives for member types
// Member `joints`
// already included above
// #include "sensor_msgs/msg/joint_state.h"
// Member `joints`
// already included above
// #include "sensor_msgs/msg/detail/joint_state__rosidl_typesupport_introspection_c.h"
// Member `is_valid`
// Member `result_type`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  baxter_core_msgs__srv__SolvePositionIK_Response__init(message_memory);
}

void baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_fini_function(void * message_memory)
{
  baxter_core_msgs__srv__SolvePositionIK_Response__fini(message_memory);
}

size_t baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Response__joints(
  const void * untyped_member)
{
  const sensor_msgs__msg__JointState__Sequence * member =
    (const sensor_msgs__msg__JointState__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Response__joints(
  const void * untyped_member, size_t index)
{
  const sensor_msgs__msg__JointState__Sequence * member =
    (const sensor_msgs__msg__JointState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Response__joints(
  void * untyped_member, size_t index)
{
  sensor_msgs__msg__JointState__Sequence * member =
    (sensor_msgs__msg__JointState__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Response__joints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sensor_msgs__msg__JointState * item =
    ((const sensor_msgs__msg__JointState *)
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Response__joints(untyped_member, index));
  sensor_msgs__msg__JointState * value =
    (sensor_msgs__msg__JointState *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Response__joints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sensor_msgs__msg__JointState * item =
    ((sensor_msgs__msg__JointState *)
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Response__joints(untyped_member, index));
  const sensor_msgs__msg__JointState * value =
    (const sensor_msgs__msg__JointState *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Response__joints(
  void * untyped_member, size_t size)
{
  sensor_msgs__msg__JointState__Sequence * member =
    (sensor_msgs__msg__JointState__Sequence *)(untyped_member);
  sensor_msgs__msg__JointState__Sequence__fini(member);
  return sensor_msgs__msg__JointState__Sequence__init(member, size);
}

size_t baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Response__is_valid(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Response__is_valid(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Response__is_valid(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Response__is_valid(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Response__is_valid(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Response__is_valid(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Response__is_valid(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Response__is_valid(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Response__result_type(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Response__result_type(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Response__result_type(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Response__result_type(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Response__result_type(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Response__result_type(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Response__result_type(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Response__result_type(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_member_array[3] = {
  {
    "joints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__srv__SolvePositionIK_Response, joints),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Response__joints,  // size() function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Response__joints,  // get_const(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Response__joints,  // get(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Response__joints,  // fetch(index, &value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Response__joints,  // assign(index, value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Response__joints  // resize(index) function pointer
  },
  {
    "is_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__srv__SolvePositionIK_Response, is_valid),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Response__is_valid,  // size() function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Response__is_valid,  // get_const(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Response__is_valid,  // get(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Response__is_valid,  // fetch(index, &value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Response__is_valid,  // assign(index, value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Response__is_valid  // resize(index) function pointer
  },
  {
    "result_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__srv__SolvePositionIK_Response, result_type),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__size_function__SolvePositionIK_Response__result_type,  // size() function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_const_function__SolvePositionIK_Response__result_type,  // get_const(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__get_function__SolvePositionIK_Response__result_type,  // get(index) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__fetch_function__SolvePositionIK_Response__result_type,  // fetch(index, &value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__assign_function__SolvePositionIK_Response__result_type,  // assign(index, value) function pointer
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__resize_function__SolvePositionIK_Response__result_type  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_members = {
  "baxter_core_msgs__srv",  // message namespace
  "SolvePositionIK_Response",  // message name
  3,  // number of fields
  sizeof(baxter_core_msgs__srv__SolvePositionIK_Response),
  baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_member_array,  // message members
  baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_type_support_handle = {
  0,
  &baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_core_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, srv, SolvePositionIK_Response)() {
  baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, JointState)();
  if (!baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_type_support_handle.typesupport_identifier) {
    baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &baxter_core_msgs__srv__SolvePositionIK_Response__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "baxter_core_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "baxter_core_msgs/srv/detail/solve_position_ik__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers baxter_core_msgs__srv__detail__solve_position_ik__rosidl_typesupport_introspection_c__SolvePositionIK_service_members = {
  "baxter_core_msgs__srv",  // service namespace
  "SolvePositionIK",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // baxter_core_msgs__srv__detail__solve_position_ik__rosidl_typesupport_introspection_c__SolvePositionIK_Request_message_type_support_handle,
  NULL  // response message
  // baxter_core_msgs__srv__detail__solve_position_ik__rosidl_typesupport_introspection_c__SolvePositionIK_Response_message_type_support_handle
};

static rosidl_service_type_support_t baxter_core_msgs__srv__detail__solve_position_ik__rosidl_typesupport_introspection_c__SolvePositionIK_service_type_support_handle = {
  0,
  &baxter_core_msgs__srv__detail__solve_position_ik__rosidl_typesupport_introspection_c__SolvePositionIK_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, srv, SolvePositionIK_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, srv, SolvePositionIK_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_core_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, srv, SolvePositionIK)() {
  if (!baxter_core_msgs__srv__detail__solve_position_ik__rosidl_typesupport_introspection_c__SolvePositionIK_service_type_support_handle.typesupport_identifier) {
    baxter_core_msgs__srv__detail__solve_position_ik__rosidl_typesupport_introspection_c__SolvePositionIK_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)baxter_core_msgs__srv__detail__solve_position_ik__rosidl_typesupport_introspection_c__SolvePositionIK_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, srv, SolvePositionIK_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, srv, SolvePositionIK_Response)()->data;
  }

  return &baxter_core_msgs__srv__detail__solve_position_ik__rosidl_typesupport_introspection_c__SolvePositionIK_service_type_support_handle;
}
