// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:srv/SolvePositionIK.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__STRUCT_H_
#define BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SEED_AUTO'.
/**
  * Seed Type Mode
  * * default (SEED_AUTO) mode: iterate through seed types until first valid
  *                             solution is found
  * * setting any other mode:   try only that seed type
 */
enum
{
  baxter_core_msgs__srv__SolvePositionIK_Request__SEED_AUTO = 0
};

/// Constant 'SEED_USER'.
enum
{
  baxter_core_msgs__srv__SolvePositionIK_Request__SEED_USER = 1
};

/// Constant 'SEED_CURRENT'.
enum
{
  baxter_core_msgs__srv__SolvePositionIK_Request__SEED_CURRENT = 2
};

/// Constant 'SEED_NS_MAP'.
enum
{
  baxter_core_msgs__srv__SolvePositionIK_Request__SEED_NS_MAP = 3
};

// Include directives for member types
// Member 'pose_stamp'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'seed_angles'
#include "sensor_msgs/msg/detail/joint_state__struct.h"

/// Struct defined in srv/SolvePositionIK in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__SolvePositionIK_Request
{
  geometry_msgs__msg__PoseStamped__Sequence pose_stamp;
  /// (optional) Joint Angle Seed(s) for IK solver.
  /// * specify a JointState seed for each pose_stamp, using name[] and position[]
  /// * empty arrays or a non-default seed_mode will cause user seed to not be used
  sensor_msgs__msg__JointState__Sequence seed_angles;
  uint8_t seed_mode;
} baxter_core_msgs__srv__SolvePositionIK_Request;

// Struct for a sequence of baxter_core_msgs__srv__SolvePositionIK_Request.
typedef struct baxter_core_msgs__srv__SolvePositionIK_Request__Sequence
{
  baxter_core_msgs__srv__SolvePositionIK_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__SolvePositionIK_Request__Sequence;


// Constants defined in the message

/// Constant 'RESULT_INVALID'.
/**
  * result_type[i] == seed type used to find valid solution, joints[i];
  * otherwise,     == RESULT_INVALID (no valid solution found).
 */
enum
{
  baxter_core_msgs__srv__SolvePositionIK_Response__RESULT_INVALID = 0
};

// Include directives for member types
// Member 'joints'
// already included above
// #include "sensor_msgs/msg/detail/joint_state__struct.h"
// Member 'is_valid'
// Member 'result_type'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/SolvePositionIK in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__SolvePositionIK_Response
{
  sensor_msgs__msg__JointState__Sequence joints;
  /// NOTE: isValid will be deprecated by result_type in future versions
  rosidl_runtime_c__boolean__Sequence is_valid;
  rosidl_runtime_c__uint8__Sequence result_type;
} baxter_core_msgs__srv__SolvePositionIK_Response;

// Struct for a sequence of baxter_core_msgs__srv__SolvePositionIK_Response.
typedef struct baxter_core_msgs__srv__SolvePositionIK_Response__Sequence
{
  baxter_core_msgs__srv__SolvePositionIK_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__SolvePositionIK_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__STRUCT_H_
