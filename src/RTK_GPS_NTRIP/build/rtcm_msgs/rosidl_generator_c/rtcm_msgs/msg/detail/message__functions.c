// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rtcm_msgs:msg/Message.idl
// generated code does not contain a copyright notice
#include "rtcm_msgs/msg/detail/message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `message`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
rtcm_msgs__msg__Message__init(rtcm_msgs__msg__Message * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rtcm_msgs__msg__Message__fini(msg);
    return false;
  }
  // message
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->message, 0)) {
    rtcm_msgs__msg__Message__fini(msg);
    return false;
  }
  return true;
}

void
rtcm_msgs__msg__Message__fini(rtcm_msgs__msg__Message * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // message
  rosidl_runtime_c__uint8__Sequence__fini(&msg->message);
}

bool
rtcm_msgs__msg__Message__are_equal(const rtcm_msgs__msg__Message * lhs, const rtcm_msgs__msg__Message * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
rtcm_msgs__msg__Message__copy(
  const rtcm_msgs__msg__Message * input,
  rtcm_msgs__msg__Message * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

rtcm_msgs__msg__Message *
rtcm_msgs__msg__Message__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rtcm_msgs__msg__Message * msg = (rtcm_msgs__msg__Message *)allocator.allocate(sizeof(rtcm_msgs__msg__Message), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rtcm_msgs__msg__Message));
  bool success = rtcm_msgs__msg__Message__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rtcm_msgs__msg__Message__destroy(rtcm_msgs__msg__Message * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rtcm_msgs__msg__Message__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rtcm_msgs__msg__Message__Sequence__init(rtcm_msgs__msg__Message__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rtcm_msgs__msg__Message * data = NULL;

  if (size) {
    data = (rtcm_msgs__msg__Message *)allocator.zero_allocate(size, sizeof(rtcm_msgs__msg__Message), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rtcm_msgs__msg__Message__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rtcm_msgs__msg__Message__fini(&data[i - 1]);
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
rtcm_msgs__msg__Message__Sequence__fini(rtcm_msgs__msg__Message__Sequence * array)
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
      rtcm_msgs__msg__Message__fini(&array->data[i]);
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

rtcm_msgs__msg__Message__Sequence *
rtcm_msgs__msg__Message__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rtcm_msgs__msg__Message__Sequence * array = (rtcm_msgs__msg__Message__Sequence *)allocator.allocate(sizeof(rtcm_msgs__msg__Message__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rtcm_msgs__msg__Message__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rtcm_msgs__msg__Message__Sequence__destroy(rtcm_msgs__msg__Message__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rtcm_msgs__msg__Message__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rtcm_msgs__msg__Message__Sequence__are_equal(const rtcm_msgs__msg__Message__Sequence * lhs, const rtcm_msgs__msg__Message__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rtcm_msgs__msg__Message__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rtcm_msgs__msg__Message__Sequence__copy(
  const rtcm_msgs__msg__Message__Sequence * input,
  rtcm_msgs__msg__Message__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rtcm_msgs__msg__Message);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rtcm_msgs__msg__Message * data =
      (rtcm_msgs__msg__Message *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rtcm_msgs__msg__Message__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rtcm_msgs__msg__Message__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rtcm_msgs__msg__Message__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
