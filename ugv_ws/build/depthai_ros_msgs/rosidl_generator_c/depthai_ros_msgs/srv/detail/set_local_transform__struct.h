// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from depthai_ros_msgs:srv/SetLocalTransform.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "depthai_ros_msgs/srv/set_local_transform.h"


#ifndef DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__STRUCT_H_
#define DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'local_transform'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/SetLocalTransform in the package depthai_ros_msgs.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Request
{
  geometry_msgs__msg__Pose local_transform;
} depthai_ros_msgs__srv__SetLocalTransform_Request;

// Struct for a sequence of depthai_ros_msgs__srv__SetLocalTransform_Request.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence
{
  depthai_ros_msgs__srv__SetLocalTransform_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/SetLocalTransform in the package depthai_ros_msgs.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Response
{
  bool success;
} depthai_ros_msgs__srv__SetLocalTransform_Response;

// Struct for a sequence of depthai_ros_msgs__srv__SetLocalTransform_Response.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence
{
  depthai_ros_msgs__srv__SetLocalTransform_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  depthai_ros_msgs__srv__SetLocalTransform_Event__request__MAX_SIZE = 1
};
// response
enum
{
  depthai_ros_msgs__srv__SetLocalTransform_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetLocalTransform in the package depthai_ros_msgs.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Event
{
  service_msgs__msg__ServiceEventInfo info;
  depthai_ros_msgs__srv__SetLocalTransform_Request__Sequence request;
  depthai_ros_msgs__srv__SetLocalTransform_Response__Sequence response;
} depthai_ros_msgs__srv__SetLocalTransform_Event;

// Struct for a sequence of depthai_ros_msgs__srv__SetLocalTransform_Event.
typedef struct depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence
{
  depthai_ros_msgs__srv__SetLocalTransform_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} depthai_ros_msgs__srv__SetLocalTransform_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__STRUCT_H_
