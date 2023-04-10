// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pub_cpp:msg/SinAngle.idl
// generated code does not contain a copyright notice

#ifndef PUB_CPP__MSG__DETAIL__SIN_ANGLE__FUNCTIONS_H_
#define PUB_CPP__MSG__DETAIL__SIN_ANGLE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pub_cpp/msg/rosidl_generator_c__visibility_control.h"

#include "pub_cpp/msg/detail/sin_angle__struct.h"

/// Initialize msg/SinAngle message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pub_cpp__msg__SinAngle
 * )) before or use
 * pub_cpp__msg__SinAngle__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
bool
pub_cpp__msg__SinAngle__init(pub_cpp__msg__SinAngle * msg);

/// Finalize msg/SinAngle message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
void
pub_cpp__msg__SinAngle__fini(pub_cpp__msg__SinAngle * msg);

/// Create msg/SinAngle message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pub_cpp__msg__SinAngle__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
pub_cpp__msg__SinAngle *
pub_cpp__msg__SinAngle__create();

/// Destroy msg/SinAngle message.
/**
 * It calls
 * pub_cpp__msg__SinAngle__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
void
pub_cpp__msg__SinAngle__destroy(pub_cpp__msg__SinAngle * msg);

/// Check for msg/SinAngle message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
bool
pub_cpp__msg__SinAngle__are_equal(const pub_cpp__msg__SinAngle * lhs, const pub_cpp__msg__SinAngle * rhs);

/// Copy a msg/SinAngle message.
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
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
bool
pub_cpp__msg__SinAngle__copy(
  const pub_cpp__msg__SinAngle * input,
  pub_cpp__msg__SinAngle * output);

/// Initialize array of msg/SinAngle messages.
/**
 * It allocates the memory for the number of elements and calls
 * pub_cpp__msg__SinAngle__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
bool
pub_cpp__msg__SinAngle__Sequence__init(pub_cpp__msg__SinAngle__Sequence * array, size_t size);

/// Finalize array of msg/SinAngle messages.
/**
 * It calls
 * pub_cpp__msg__SinAngle__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
void
pub_cpp__msg__SinAngle__Sequence__fini(pub_cpp__msg__SinAngle__Sequence * array);

/// Create array of msg/SinAngle messages.
/**
 * It allocates the memory for the array and calls
 * pub_cpp__msg__SinAngle__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
pub_cpp__msg__SinAngle__Sequence *
pub_cpp__msg__SinAngle__Sequence__create(size_t size);

/// Destroy array of msg/SinAngle messages.
/**
 * It calls
 * pub_cpp__msg__SinAngle__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
void
pub_cpp__msg__SinAngle__Sequence__destroy(pub_cpp__msg__SinAngle__Sequence * array);

/// Check for msg/SinAngle message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
bool
pub_cpp__msg__SinAngle__Sequence__are_equal(const pub_cpp__msg__SinAngle__Sequence * lhs, const pub_cpp__msg__SinAngle__Sequence * rhs);

/// Copy an array of msg/SinAngle messages.
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
ROSIDL_GENERATOR_C_PUBLIC_pub_cpp
bool
pub_cpp__msg__SinAngle__Sequence__copy(
  const pub_cpp__msg__SinAngle__Sequence * input,
  pub_cpp__msg__SinAngle__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PUB_CPP__MSG__DETAIL__SIN_ANGLE__FUNCTIONS_H_
