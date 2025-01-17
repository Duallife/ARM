// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface:msg/JointAngle.idl
// generated code does not contain a copyright notice
#include "interface/msg/detail/joint_angle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
interface__msg__JointAngle__init(interface__msg__JointAngle * msg)
{
  if (!msg) {
    return false;
  }
  // joint1
  // joint2
  // joint3
  // joint4
  // joint5
  // joint6
  // gripper
  return true;
}

void
interface__msg__JointAngle__fini(interface__msg__JointAngle * msg)
{
  if (!msg) {
    return;
  }
  // joint1
  // joint2
  // joint3
  // joint4
  // joint5
  // joint6
  // gripper
}

bool
interface__msg__JointAngle__are_equal(const interface__msg__JointAngle * lhs, const interface__msg__JointAngle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint1
  if (lhs->joint1 != rhs->joint1) {
    return false;
  }
  // joint2
  if (lhs->joint2 != rhs->joint2) {
    return false;
  }
  // joint3
  if (lhs->joint3 != rhs->joint3) {
    return false;
  }
  // joint4
  if (lhs->joint4 != rhs->joint4) {
    return false;
  }
  // joint5
  if (lhs->joint5 != rhs->joint5) {
    return false;
  }
  // joint6
  if (lhs->joint6 != rhs->joint6) {
    return false;
  }
  // gripper
  if (lhs->gripper != rhs->gripper) {
    return false;
  }
  return true;
}

bool
interface__msg__JointAngle__copy(
  const interface__msg__JointAngle * input,
  interface__msg__JointAngle * output)
{
  if (!input || !output) {
    return false;
  }
  // joint1
  output->joint1 = input->joint1;
  // joint2
  output->joint2 = input->joint2;
  // joint3
  output->joint3 = input->joint3;
  // joint4
  output->joint4 = input->joint4;
  // joint5
  output->joint5 = input->joint5;
  // joint6
  output->joint6 = input->joint6;
  // gripper
  output->gripper = input->gripper;
  return true;
}

interface__msg__JointAngle *
interface__msg__JointAngle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__msg__JointAngle * msg = (interface__msg__JointAngle *)allocator.allocate(sizeof(interface__msg__JointAngle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__msg__JointAngle));
  bool success = interface__msg__JointAngle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__msg__JointAngle__destroy(interface__msg__JointAngle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__msg__JointAngle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__msg__JointAngle__Sequence__init(interface__msg__JointAngle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__msg__JointAngle * data = NULL;

  if (size) {
    data = (interface__msg__JointAngle *)allocator.zero_allocate(size, sizeof(interface__msg__JointAngle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__msg__JointAngle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__msg__JointAngle__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__msg__JointAngle__Sequence__fini(interface__msg__JointAngle__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__msg__JointAngle__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__msg__JointAngle__Sequence *
interface__msg__JointAngle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__msg__JointAngle__Sequence * array = (interface__msg__JointAngle__Sequence *)allocator.allocate(sizeof(interface__msg__JointAngle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__msg__JointAngle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__msg__JointAngle__Sequence__destroy(interface__msg__JointAngle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__msg__JointAngle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__msg__JointAngle__Sequence__are_equal(const interface__msg__JointAngle__Sequence * lhs, const interface__msg__JointAngle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__msg__JointAngle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__msg__JointAngle__Sequence__copy(
  const interface__msg__JointAngle__Sequence * input,
  interface__msg__JointAngle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__msg__JointAngle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface__msg__JointAngle * data =
      (interface__msg__JointAngle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__msg__JointAngle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface__msg__JointAngle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__msg__JointAngle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
