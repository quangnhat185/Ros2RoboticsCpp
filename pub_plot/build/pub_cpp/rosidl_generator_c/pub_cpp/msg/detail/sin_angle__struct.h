// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pub_cpp:msg/SinAngle.idl
// generated code does not contain a copyright notice

#ifndef PUB_CPP__MSG__DETAIL__SIN_ANGLE__STRUCT_H_
#define PUB_CPP__MSG__DETAIL__SIN_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SinAngle in the package pub_cpp.
typedef struct pub_cpp__msg__SinAngle
{
  float angle;
  float sin_angle;
} pub_cpp__msg__SinAngle;

// Struct for a sequence of pub_cpp__msg__SinAngle.
typedef struct pub_cpp__msg__SinAngle__Sequence
{
  pub_cpp__msg__SinAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pub_cpp__msg__SinAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PUB_CPP__MSG__DETAIL__SIN_ANGLE__STRUCT_H_
