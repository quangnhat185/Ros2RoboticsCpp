// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pub_cpp:msg/SinAngle.idl
// generated code does not contain a copyright notice

#ifndef PUB_CPP__MSG__DETAIL__SIN_ANGLE__STRUCT_HPP_
#define PUB_CPP__MSG__DETAIL__SIN_ANGLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pub_cpp__msg__SinAngle __attribute__((deprecated))
#else
# define DEPRECATED__pub_cpp__msg__SinAngle __declspec(deprecated)
#endif

namespace pub_cpp
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SinAngle_
{
  using Type = SinAngle_<ContainerAllocator>;

  explicit SinAngle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->sin_angle = 0.0f;
    }
  }

  explicit SinAngle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->sin_angle = 0.0f;
    }
  }

  // field types and members
  using _angle_type =
    float;
  _angle_type angle;
  using _sin_angle_type =
    float;
  _sin_angle_type sin_angle;

  // setters for named parameter idiom
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__sin_angle(
    const float & _arg)
  {
    this->sin_angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pub_cpp::msg::SinAngle_<ContainerAllocator> *;
  using ConstRawPtr =
    const pub_cpp::msg::SinAngle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pub_cpp::msg::SinAngle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pub_cpp::msg::SinAngle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pub_cpp::msg::SinAngle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pub_cpp::msg::SinAngle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pub_cpp::msg::SinAngle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pub_cpp::msg::SinAngle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pub_cpp::msg::SinAngle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pub_cpp::msg::SinAngle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pub_cpp__msg__SinAngle
    std::shared_ptr<pub_cpp::msg::SinAngle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pub_cpp__msg__SinAngle
    std::shared_ptr<pub_cpp::msg::SinAngle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SinAngle_ & other) const
  {
    if (this->angle != other.angle) {
      return false;
    }
    if (this->sin_angle != other.sin_angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const SinAngle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SinAngle_

// alias to use template instance with default allocator
using SinAngle =
  pub_cpp::msg::SinAngle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pub_cpp

#endif  // PUB_CPP__MSG__DETAIL__SIN_ANGLE__STRUCT_HPP_
