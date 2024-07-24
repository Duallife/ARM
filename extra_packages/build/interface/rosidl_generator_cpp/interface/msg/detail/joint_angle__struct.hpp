// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface:msg/JointAngle.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__JOINT_ANGLE__STRUCT_HPP_
#define INTERFACE__MSG__DETAIL__JOINT_ANGLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface__msg__JointAngle __attribute__((deprecated))
#else
# define DEPRECATED__interface__msg__JointAngle __declspec(deprecated)
#endif

namespace interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointAngle_
{
  using Type = JointAngle_<ContainerAllocator>;

  explicit JointAngle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint1 = 0.0f;
      this->joint2 = 0.0f;
      this->joint3 = 0.0f;
      this->joint4 = 0.0f;
      this->joint5 = 0.0f;
      this->joint6 = 0.0f;
    }
  }

  explicit JointAngle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint1 = 0.0f;
      this->joint2 = 0.0f;
      this->joint3 = 0.0f;
      this->joint4 = 0.0f;
      this->joint5 = 0.0f;
      this->joint6 = 0.0f;
    }
  }

  // field types and members
  using _joint1_type =
    float;
  _joint1_type joint1;
  using _joint2_type =
    float;
  _joint2_type joint2;
  using _joint3_type =
    float;
  _joint3_type joint3;
  using _joint4_type =
    float;
  _joint4_type joint4;
  using _joint5_type =
    float;
  _joint5_type joint5;
  using _joint6_type =
    float;
  _joint6_type joint6;

  // setters for named parameter idiom
  Type & set__joint1(
    const float & _arg)
  {
    this->joint1 = _arg;
    return *this;
  }
  Type & set__joint2(
    const float & _arg)
  {
    this->joint2 = _arg;
    return *this;
  }
  Type & set__joint3(
    const float & _arg)
  {
    this->joint3 = _arg;
    return *this;
  }
  Type & set__joint4(
    const float & _arg)
  {
    this->joint4 = _arg;
    return *this;
  }
  Type & set__joint5(
    const float & _arg)
  {
    this->joint5 = _arg;
    return *this;
  }
  Type & set__joint6(
    const float & _arg)
  {
    this->joint6 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface::msg::JointAngle_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::msg::JointAngle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::msg::JointAngle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::msg::JointAngle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::msg::JointAngle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::msg::JointAngle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::msg::JointAngle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::msg::JointAngle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::msg::JointAngle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::msg::JointAngle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__msg__JointAngle
    std::shared_ptr<interface::msg::JointAngle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__msg__JointAngle
    std::shared_ptr<interface::msg::JointAngle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointAngle_ & other) const
  {
    if (this->joint1 != other.joint1) {
      return false;
    }
    if (this->joint2 != other.joint2) {
      return false;
    }
    if (this->joint3 != other.joint3) {
      return false;
    }
    if (this->joint4 != other.joint4) {
      return false;
    }
    if (this->joint5 != other.joint5) {
      return false;
    }
    if (this->joint6 != other.joint6) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointAngle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointAngle_

// alias to use template instance with default allocator
using JointAngle =
  interface::msg::JointAngle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interface

#endif  // INTERFACE__MSG__DETAIL__JOINT_ANGLE__STRUCT_HPP_
