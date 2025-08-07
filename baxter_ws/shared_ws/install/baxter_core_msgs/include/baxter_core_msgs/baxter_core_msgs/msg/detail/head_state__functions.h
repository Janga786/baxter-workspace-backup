// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from baxter_core_msgs:msg/HeadState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__FUNCTIONS_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "baxter_core_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "baxter_core_msgs/msg/detail/head_state__struct.h"

/// Initialize msg/HeadState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * baxter_core_msgs__msg__HeadState
 * )) before or use
 * baxter_core_msgs__msg__HeadState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__HeadState__init(baxter_core_msgs__msg__HeadState * msg);

/// Finalize msg/HeadState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
void
baxter_core_msgs__msg__HeadState__fini(baxter_core_msgs__msg__HeadState * msg);

/// Create msg/HeadState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * baxter_core_msgs__msg__HeadState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
baxter_core_msgs__msg__HeadState *
baxter_core_msgs__msg__HeadState__create();

/// Destroy msg/HeadState message.
/**
 * It calls
 * baxter_core_msgs__msg__HeadState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
void
baxter_core_msgs__msg__HeadState__destroy(baxter_core_msgs__msg__HeadState * msg);

/// Check for msg/HeadState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__HeadState__are_equal(const baxter_core_msgs__msg__HeadState * lhs, const baxter_core_msgs__msg__HeadState * rhs);

/// Copy a msg/HeadState message.
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
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__HeadState__copy(
  const baxter_core_msgs__msg__HeadState * input,
  baxter_core_msgs__msg__HeadState * output);

/// Initialize array of msg/HeadState messages.
/**
 * It allocates the memory for the number of elements and calls
 * baxter_core_msgs__msg__HeadState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__HeadState__Sequence__init(baxter_core_msgs__msg__HeadState__Sequence * array, size_t size);

/// Finalize array of msg/HeadState messages.
/**
 * It calls
 * baxter_core_msgs__msg__HeadState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
void
baxter_core_msgs__msg__HeadState__Sequence__fini(baxter_core_msgs__msg__HeadState__Sequence * array);

/// Create array of msg/HeadState messages.
/**
 * It allocates the memory for the array and calls
 * baxter_core_msgs__msg__HeadState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
baxter_core_msgs__msg__HeadState__Sequence *
baxter_core_msgs__msg__HeadState__Sequence__create(size_t size);

/// Destroy array of msg/HeadState messages.
/**
 * It calls
 * baxter_core_msgs__msg__HeadState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
void
baxter_core_msgs__msg__HeadState__Sequence__destroy(baxter_core_msgs__msg__HeadState__Sequence * array);

/// Check for msg/HeadState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__HeadState__Sequence__are_equal(const baxter_core_msgs__msg__HeadState__Sequence * lhs, const baxter_core_msgs__msg__HeadState__Sequence * rhs);

/// Copy an array of msg/HeadState messages.
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
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__HeadState__Sequence__copy(
  const baxter_core_msgs__msg__HeadState__Sequence * input,
  baxter_core_msgs__msg__HeadState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__FUNCTIONS_H_
