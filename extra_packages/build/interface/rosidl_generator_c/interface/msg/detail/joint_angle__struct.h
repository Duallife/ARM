// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:msg/JointAngle.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__JOINT_ANGLE__STRUCT_H_
#define INTERFACE__MSG__DETAIL__JOINT_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/JointAngle in the package interface.
typedef struct interface__msg__JointAngle
{
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
} interface__msg__JointAngle;

// Struct for a sequence of interface__msg__JointAngle.
typedef struct interface__msg__JointAngle__Sequence
{
  interface__msg__JointAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__msg__JointAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__MSG__DETAIL__JOINT_ANGLE__STRUCT_H_
