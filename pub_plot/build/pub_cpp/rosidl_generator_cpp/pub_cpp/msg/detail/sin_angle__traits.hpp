// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pub_cpp:msg/SinAngle.idl
// generated code does not contain a copyright notice

#ifndef PUB_CPP__MSG__DETAIL__SIN_ANGLE__TRAITS_HPP_
#define PUB_CPP__MSG__DETAIL__SIN_ANGLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pub_cpp/msg/detail/sin_angle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pub_cpp
{

namespace msg
{

inline void to_flow_style_yaml(
  const SinAngle & msg,
  std::ostream & out)
{
  out << "{";
  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: sin_angle
  {
    out << "sin_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.sin_angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SinAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: sin_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sin_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.sin_angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SinAngle & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace pub_cpp

namespace rosidl_generator_traits
{

[[deprecated("use pub_cpp::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pub_cpp::msg::SinAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  pub_cpp::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pub_cpp::msg::to_yaml() instead")]]
inline std::string to_yaml(const pub_cpp::msg::SinAngle & msg)
{
  return pub_cpp::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pub_cpp::msg::SinAngle>()
{
  return "pub_cpp::msg::SinAngle";
}

template<>
inline const char * name<pub_cpp::msg::SinAngle>()
{
  return "pub_cpp/msg/SinAngle";
}

template<>
struct has_fixed_size<pub_cpp::msg::SinAngle>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pub_cpp::msg::SinAngle>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pub_cpp::msg::SinAngle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PUB_CPP__MSG__DETAIL__SIN_ANGLE__TRAITS_HPP_
