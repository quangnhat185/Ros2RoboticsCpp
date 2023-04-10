// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from pub_cpp:msg/SinAngle.idl
// generated code does not contain a copyright notice

#ifndef PUB_CPP__MSG__DETAIL__SIN_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define PUB_CPP__MSG__DETAIL__SIN_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "pub_cpp/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "pub_cpp/msg/detail/sin_angle__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace pub_cpp
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pub_cpp
cdr_serialize(
  const pub_cpp::msg::SinAngle & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pub_cpp
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  pub_cpp::msg::SinAngle & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pub_cpp
get_serialized_size(
  const pub_cpp::msg::SinAngle & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pub_cpp
max_serialized_size_SinAngle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace pub_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pub_cpp
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pub_cpp, msg, SinAngle)();

#ifdef __cplusplus
}
#endif

#endif  // PUB_CPP__MSG__DETAIL__SIN_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
