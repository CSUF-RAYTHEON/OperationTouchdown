// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from depthai_ros_msgs:srv/SetLocalTransform.idl
// generated code does not contain a copyright notice
#include "depthai_ros_msgs/srv/detail/set_local_transform__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `local_transform`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
depthai_ros_msgs__srv__SetLocalTransform_Request__init(depthai_ros_msgs__srv__SetLocalTransform_Request * msg)
{
  if (!msg) {
    return false;
  }
  // local_transform
  if (!geometry_msgs__msg__Pose__init(&msg->local_transform)) {
    depthai_ros_msgs__srv__SetLocalTransform_Request__fini(msg);
    return false;
  }
  return true;
}

void
depthai_ros_msgs__srv__SetLocalTransform_Request__fini(depthai_ros_msgs__srv__SetLocalTransform_Request * msg)
{
  if (!msg) {
    return;
  }
  // local_transform
  geometry_msgs__msg__Pose__fini(&msg->local_transform);
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Request__are_equal(const depthai_ros_msgs__srv__SetLocalTransform_Request * lhs, const depthai_ros_msgs__srv__SetLocalTransform_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // local_transform
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->local_transform), &(rhs->local_transform)))
  {
    return false;
  }
  return true;
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Request__copy(
  const depthai_ros_msgs__srv__SetLocalTransform_Request * input,
  depthai_ros_msgs__srv__SetLocalTransform_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // local_transform
  if (!geometry_msgs__msg__Pose__copy(
      &(input->local_transform), &(output->local_transform)))
  {
    return false;
  }
  return true;
}

depthai_ros_msgs__srv__SetLocalTransform_Request *
depthai_ros_msgs__srv__SetLocalTransform_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  depthai_ros_msgs__srv__SetLocalTransform_Request * msg = (depthai_ros_msgs__srv__SetLocalTransform_Request *)allocator.allocate(sizeof(depthai_ros_msgs__srv__SetLocalTransform_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(depthai_ros_msgs__srv__SetLocalTransform_Request));
  bool success = depthai_ros_msgs__srv__SetLocalTransform_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
depthai_ros_msgs__srv__SetLocalTransform_Request__destroy(depthai_ros_msgs__srv__SetLocalTransform_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    depthai_ros_msgs__srv__SetLocalTransform_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__init(depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  depthai_ros_msgs__srv__SetLocalTransform_Request * data = NULL;

  if (size) {
    data = (depthai_ros_msgs__srv__SetLocalTransform_Request *)allocator.zero_allocate(size, sizeof(depthai_ros_msgs__srv__SetLocalTransform_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = depthai_ros_msgs__srv__SetLocalTransform_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        depthai_ros_msgs__srv__SetLocalTransform_Request__fini(&data[i - 1]);
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
depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__fini(depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence * array)
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
      depthai_ros_msgs__srv__SetLocalTransform_Request__fini(&array->data[i]);
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

depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence *
depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence * array = (depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence *)allocator.allocate(sizeof(depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__destroy(depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__are_equal(const depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence * lhs, const depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!depthai_ros_msgs__srv__SetLocalTransform_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__copy(
  const depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence * input,
  depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(depthai_ros_msgs__srv__SetLocalTransform_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    depthai_ros_msgs__srv__SetLocalTransform_Request * data =
      (depthai_ros_msgs__srv__SetLocalTransform_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!depthai_ros_msgs__srv__SetLocalTransform_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          depthai_ros_msgs__srv__SetLocalTransform_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!depthai_ros_msgs__srv__SetLocalTransform_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
depthai_ros_msgs__srv__SetLocalTransform_Response__init(depthai_ros_msgs__srv__SetLocalTransform_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
depthai_ros_msgs__srv__SetLocalTransform_Response__fini(depthai_ros_msgs__srv__SetLocalTransform_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Response__are_equal(const depthai_ros_msgs__srv__SetLocalTransform_Response * lhs, const depthai_ros_msgs__srv__SetLocalTransform_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Response__copy(
  const depthai_ros_msgs__srv__SetLocalTransform_Response * input,
  depthai_ros_msgs__srv__SetLocalTransform_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

depthai_ros_msgs__srv__SetLocalTransform_Response *
depthai_ros_msgs__srv__SetLocalTransform_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  depthai_ros_msgs__srv__SetLocalTransform_Response * msg = (depthai_ros_msgs__srv__SetLocalTransform_Response *)allocator.allocate(sizeof(depthai_ros_msgs__srv__SetLocalTransform_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(depthai_ros_msgs__srv__SetLocalTransform_Response));
  bool success = depthai_ros_msgs__srv__SetLocalTransform_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
depthai_ros_msgs__srv__SetLocalTransform_Response__destroy(depthai_ros_msgs__srv__SetLocalTransform_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    depthai_ros_msgs__srv__SetLocalTransform_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__init(depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  depthai_ros_msgs__srv__SetLocalTransform_Response * data = NULL;

  if (size) {
    data = (depthai_ros_msgs__srv__SetLocalTransform_Response *)allocator.zero_allocate(size, sizeof(depthai_ros_msgs__srv__SetLocalTransform_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = depthai_ros_msgs__srv__SetLocalTransform_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        depthai_ros_msgs__srv__SetLocalTransform_Response__fini(&data[i - 1]);
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
depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__fini(depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence * array)
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
      depthai_ros_msgs__srv__SetLocalTransform_Response__fini(&array->data[i]);
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

depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence *
depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence * array = (depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence *)allocator.allocate(sizeof(depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__destroy(depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__are_equal(const depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence * lhs, const depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!depthai_ros_msgs__srv__SetLocalTransform_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__copy(
  const depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence * input,
  depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(depthai_ros_msgs__srv__SetLocalTransform_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    depthai_ros_msgs__srv__SetLocalTransform_Response * data =
      (depthai_ros_msgs__srv__SetLocalTransform_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!depthai_ros_msgs__srv__SetLocalTransform_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          depthai_ros_msgs__srv__SetLocalTransform_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!depthai_ros_msgs__srv__SetLocalTransform_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "depthai_ros_msgs/srv/detail/set_local_transform__functions.h"

bool
depthai_ros_msgs__srv__SetLocalTransform_Event__init(depthai_ros_msgs__srv__SetLocalTransform_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    depthai_ros_msgs__srv__SetLocalTransform_Event__fini(msg);
    return false;
  }
  // request
  if (!depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__init(&msg->request, 0)) {
    depthai_ros_msgs__srv__SetLocalTransform_Event__fini(msg);
    return false;
  }
  // response
  if (!depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__init(&msg->response, 0)) {
    depthai_ros_msgs__srv__SetLocalTransform_Event__fini(msg);
    return false;
  }
  return true;
}

void
depthai_ros_msgs__srv__SetLocalTransform_Event__fini(depthai_ros_msgs__srv__SetLocalTransform_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__fini(&msg->request);
  // response
  depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__fini(&msg->response);
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Event__are_equal(const depthai_ros_msgs__srv__SetLocalTransform_Event * lhs, const depthai_ros_msgs__srv__SetLocalTransform_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Event__copy(
  const depthai_ros_msgs__srv__SetLocalTransform_Event * input,
  depthai_ros_msgs__srv__SetLocalTransform_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

depthai_ros_msgs__srv__SetLocalTransform_Event *
depthai_ros_msgs__srv__SetLocalTransform_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  depthai_ros_msgs__srv__SetLocalTransform_Event * msg = (depthai_ros_msgs__srv__SetLocalTransform_Event *)allocator.allocate(sizeof(depthai_ros_msgs__srv__SetLocalTransform_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(depthai_ros_msgs__srv__SetLocalTransform_Event));
  bool success = depthai_ros_msgs__srv__SetLocalTransform_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
depthai_ros_msgs__srv__SetLocalTransform_Event__destroy(depthai_ros_msgs__srv__SetLocalTransform_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    depthai_ros_msgs__srv__SetLocalTransform_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence__init(depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  depthai_ros_msgs__srv__SetLocalTransform_Event * data = NULL;

  if (size) {
    data = (depthai_ros_msgs__srv__SetLocalTransform_Event *)allocator.zero_allocate(size, sizeof(depthai_ros_msgs__srv__SetLocalTransform_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = depthai_ros_msgs__srv__SetLocalTransform_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        depthai_ros_msgs__srv__SetLocalTransform_Event__fini(&data[i - 1]);
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
depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence__fini(depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence * array)
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
      depthai_ros_msgs__srv__SetLocalTransform_Event__fini(&array->data[i]);
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

depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence *
depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence * array = (depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence *)allocator.allocate(sizeof(depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence__destroy(depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence__are_equal(const depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence * lhs, const depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!depthai_ros_msgs__srv__SetLocalTransform_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence__copy(
  const depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence * input,
  depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(depthai_ros_msgs__srv__SetLocalTransform_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    depthai_ros_msgs__srv__SetLocalTransform_Event * data =
      (depthai_ros_msgs__srv__SetLocalTransform_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!depthai_ros_msgs__srv__SetLocalTransform_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          depthai_ros_msgs__srv__SetLocalTransform_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!depthai_ros_msgs__srv__SetLocalTransform_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
