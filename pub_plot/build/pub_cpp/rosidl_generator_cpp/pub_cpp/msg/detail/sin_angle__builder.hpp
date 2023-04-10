// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pub_cpp:msg/SinAngle.idl
// generated code does not contain a copyright notice

#ifndef PUB_CPP__MSG__DETAIL__SIN_ANGLE__BUILDER_HPP_
#define PUB_CPP__MSG__DETAIL__SIN_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pub_cpp/msg/detail/sin_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pub_cpp
{

namespace msg
{

namespace builder
{

class Init_SinAngle_sin_angle
{
public:
  explicit Init_SinAngle_sin_angle(::pub_cpp::msg::SinAngle & msg)
  : msg_(msg)
  {}
  ::pub_cpp::msg::SinAngle sin_angle(::pub_cpp::msg::SinAngle::_sin_angle_type arg)
  {
    msg_.sin_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pub_cpp::msg::SinAngle msg_;
};

class Init_SinAngle_angle
{
public:
  Init_SinAngle_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SinAngle_sin_angle angle(::pub_cpp::msg::SinAngle::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_SinAngle_sin_angle(msg_);
  }

private:
  ::pub_cpp::msg::SinAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pub_cpp::msg::SinAngle>()
{
  return pub_cpp::msg::builder::Init_SinAngle_angle();
}

}  // namespace pub_cpp

#endif  // PUB_CPP__MSG__DETAIL__SIN_ANGLE__BUILDER_HPP_
