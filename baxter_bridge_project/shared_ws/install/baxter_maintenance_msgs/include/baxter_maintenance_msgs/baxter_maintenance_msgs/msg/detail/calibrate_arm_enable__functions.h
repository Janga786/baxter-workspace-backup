// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmEnable.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__FUNCTIONS_H_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "baxter_maintenance_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_enable__struct.h"

/// Initialize msg/CalibrateArmEnable message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * baxter_maintenance_msgs__msg__CalibrateArmEnable
 * )) before or use
 * baxter_maintenance_msgs__msg__CalibrateArmEnable__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
bool
baxter_maintenance_msgs__msg__CalibrateArmEnable__init(baxter_maintenance_msgs__msg__CalibrateArmEnable * msg);

/// Finalize msg/CalibrateArmEnable message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
void
baxter_maintenance_msgs__msg__CalibrateArmEnable__fini(baxter_maintenance_msgs__msg__CalibrateArmEnable * msg);

/// Create msg/CalibrateArmEnable message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * baxter_maintenance_msgs__msg__CalibrateArmEnable__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
baxter_maintenance_msgs__msg__CalibrateArmEnable *
baxter_maintenance_msgs__msg__CalibrateArmEnable__create();

/// Destroy msg/CalibrateArmEnable message.
/**
 * It calls
 * baxter_maintenance_msgs__msg__CalibrateArmEnable__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
void
baxter_maintenance_msgs__msg__CalibrateArmEnable__destroy(baxter_maintenance_msgs__msg__CalibrateArmEnable * msg);

/// Check for msg/CalibrateArmEnable message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
bool
baxter_maintenance_msgs__msg__CalibrateArmEnable__are_equal(const baxter_maintenance_msgs__msg__CalibrateArmEnable * lhs, const baxter_maintenance_msgs__msg__CalibrateArmEnable * rhs);

/// Copy a msg/CalibrateArmEnable message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
bool
baxter_maintenance_msgs__msg__CalibrateArmEnable__copy(
  const baxter_maintenance_msgs__msg__CalibrateArmEnable * input,
  baxter_maintenance_msgs__msg__CalibrateArmEnable * output);

/// Initialize array of msg/CalibrateArmEnable messages.
/**
 * It allocates the memory for the number of elements and calls
 * baxter_maintenance_msgs__msg__CalibrateArmEnable__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
bool
baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence__init(baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence * array, size_t size);

/// Finalize array of msg/CalibrateArmEnable messages.
/**
 * It calls
 * baxter_maintenance_msgs__msg__CalibrateArmEnable__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
void
baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence__fini(baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence * array);

/// Create array of msg/CalibrateArmEnable messages.
/**
 * It allocates the memory for the array and calls
 * baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence *
baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence__create(size_t size);

/// Destroy array of msg/CalibrateArmEnable messages.
/**
 * It calls
 * baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
void
baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence__destroy(baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence * array);

/// Check for msg/CalibrateArmEnable message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
bool
baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence__are_equal(const baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence * lhs, const baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence * rhs);

/// Copy an array of msg/CalibrateArmEnable messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_maintenance_msgs
bool
baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence__copy(
  const baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence * input,
  baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__FUNCTIONS_H_
