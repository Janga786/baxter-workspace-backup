// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from baxter_core_msgs:msg/AnalogIOStates.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATES__FUNCTIONS_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "baxter_core_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "baxter_core_msgs/msg/detail/analog_io_states__struct.h"

/// Initialize msg/AnalogIOStates message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * baxter_core_msgs__msg__AnalogIOStates
 * )) before or use
 * baxter_core_msgs__msg__AnalogIOStates__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__AnalogIOStates__init(baxter_core_msgs__msg__AnalogIOStates * msg);

/// Finalize msg/AnalogIOStates message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
void
baxter_core_msgs__msg__AnalogIOStates__fini(baxter_core_msgs__msg__AnalogIOStates * msg);

/// Create msg/AnalogIOStates message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * baxter_core_msgs__msg__AnalogIOStates__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
baxter_core_msgs__msg__AnalogIOStates *
baxter_core_msgs__msg__AnalogIOStates__create();

/// Destroy msg/AnalogIOStates message.
/**
 * It calls
 * baxter_core_msgs__msg__AnalogIOStates__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
void
baxter_core_msgs__msg__AnalogIOStates__destroy(baxter_core_msgs__msg__AnalogIOStates * msg);

/// Check for msg/AnalogIOStates message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__AnalogIOStates__are_equal(const baxter_core_msgs__msg__AnalogIOStates * lhs, const baxter_core_msgs__msg__AnalogIOStates * rhs);

/// Copy a msg/AnalogIOStates message.
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
baxter_core_msgs__msg__AnalogIOStates__copy(
  const baxter_core_msgs__msg__AnalogIOStates * input,
  baxter_core_msgs__msg__AnalogIOStates * output);

/// Initialize array of msg/AnalogIOStates messages.
/**
 * It allocates the memory for the number of elements and calls
 * baxter_core_msgs__msg__AnalogIOStates__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__AnalogIOStates__Sequence__init(baxter_core_msgs__msg__AnalogIOStates__Sequence * array, size_t size);

/// Finalize array of msg/AnalogIOStates messages.
/**
 * It calls
 * baxter_core_msgs__msg__AnalogIOStates__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
void
baxter_core_msgs__msg__AnalogIOStates__Sequence__fini(baxter_core_msgs__msg__AnalogIOStates__Sequence * array);

/// Create array of msg/AnalogIOStates messages.
/**
 * It allocates the memory for the array and calls
 * baxter_core_msgs__msg__AnalogIOStates__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
baxter_core_msgs__msg__AnalogIOStates__Sequence *
baxter_core_msgs__msg__AnalogIOStates__Sequence__create(size_t size);

/// Destroy array of msg/AnalogIOStates messages.
/**
 * It calls
 * baxter_core_msgs__msg__AnalogIOStates__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
void
baxter_core_msgs__msg__AnalogIOStates__Sequence__destroy(baxter_core_msgs__msg__AnalogIOStates__Sequence * array);

/// Check for msg/AnalogIOStates message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_baxter_core_msgs
bool
baxter_core_msgs__msg__AnalogIOStates__Sequence__are_equal(const baxter_core_msgs__msg__AnalogIOStates__Sequence * lhs, const baxter_core_msgs__msg__AnalogIOStates__Sequence * rhs);

/// Copy an array of msg/AnalogIOStates messages.
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
baxter_core_msgs__msg__AnalogIOStates__Sequence__copy(
  const baxter_core_msgs__msg__AnalogIOStates__Sequence * input,
  baxter_core_msgs__msg__AnalogIOStates__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATES__FUNCTIONS_H_
