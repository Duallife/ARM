// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:msg/JointAngle.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__JOINT_ANGLE__BUILDER_HPP_
#define INTERFACE__MSG__DETAIL__JOINT_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface/msg/detail/joint_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface
{

namespace msg
{

namespace builder
{

class Init_JointAngle_joint6
{
public:
  explicit Init_JointAngle_joint6(::interface::msg::JointAngle & msg)
  : msg_(msg)
  {}
  ::interface::msg::JointAngle joint6(::interface::msg::JointAngle::_joint6_type arg)
  {
    msg_.joint6 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::msg::JointAngle msg_;
};

class Init_JointAngle_joint5
{
public:
  explicit Init_JointAngle_joint5(::interface::msg::JointAngle & msg)
  : msg_(msg)
  {}
  Init_JointAngle_joint6 joint5(::interface::msg::JointAngle::_joint5_type arg)
  {
    msg_.joint5 = std::move(arg);
    return Init_JointAngle_joint6(msg_);
  }

private:
  ::interface::msg::JointAngle msg_;
};

class Init_JointAngle_joint4
{
public:
  explicit Init_JointAngle_joint4(::interface::msg::JointAngle & msg)
  : msg_(msg)
  {}
  Init_JointAngle_joint5 joint4(::interface::msg::JointAngle::_joint4_type arg)
  {
    msg_.joint4 = std::move(arg);
    return Init_JointAngle_joint5(msg_);
  }

private:
  ::interface::msg::JointAngle msg_;
};

class Init_JointAngle_joint3
{
public:
  explicit Init_JointAngle_joint3(::interface::msg::JointAngle & msg)
  : msg_(msg)
  {}
  Init_JointAngle_joint4 joint3(::interface::msg::JointAngle::_joint3_type arg)
  {
    msg_.joint3 = std::move(arg);
    return Init_JointAngle_joint4(msg_);
  }

private:
  ::interface::msg::JointAngle msg_;
};

class Init_JointAngle_joint2
{
public:
  explicit Init_JointAngle_joint2(::interface::msg::JointAngle & msg)
  : msg_(msg)
  {}
  Init_JointAngle_joint3 joint2(::interface::msg::JointAngle::_joint2_type arg)
  {
    msg_.joint2 = std::move(arg);
    return Init_JointAngle_joint3(msg_);
  }

private:
  ::interface::msg::JointAngle msg_;
};

class Init_JointAngle_joint1
{
public:
  Init_JointAngle_joint1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointAngle_joint2 joint1(::interface::msg::JointAngle::_joint1_type arg)
  {
    msg_.joint1 = std::move(arg);
    return Init_JointAngle_joint2(msg_);
  }

private:
  ::interface::msg::JointAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::msg::JointAngle>()
{
  return interface::msg::builder::Init_JointAngle_joint1();
}

}  // namespace interface

#endif  // INTERFACE__MSG__DETAIL__JOINT_ANGLE__BUILDER_HPP_
