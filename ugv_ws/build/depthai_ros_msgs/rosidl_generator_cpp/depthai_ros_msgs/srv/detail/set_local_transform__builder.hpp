// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from depthai_ros_msgs:srv/SetLocalTransform.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "depthai_ros_msgs/srv/set_local_transform.hpp"


#ifndef DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__BUILDER_HPP_
#define DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "depthai_ros_msgs/srv/detail/set_local_transform__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace depthai_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLocalTransform_Request_local_transform
{
public:
  Init_SetLocalTransform_Request_local_transform()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::depthai_ros_msgs::srv::SetLocalTransform_Request local_transform(::depthai_ros_msgs::srv::SetLocalTransform_Request::_local_transform_type arg)
  {
    msg_.local_transform = std::move(arg);
    return std::move(msg_);
  }

private:
  ::depthai_ros_msgs::srv::SetLocalTransform_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::depthai_ros_msgs::srv::SetLocalTransform_Request>()
{
  return depthai_ros_msgs::srv::builder::Init_SetLocalTransform_Request_local_transform();
}

}  // namespace depthai_ros_msgs


namespace depthai_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLocalTransform_Response_success
{
public:
  Init_SetLocalTransform_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::depthai_ros_msgs::srv::SetLocalTransform_Response success(::depthai_ros_msgs::srv::SetLocalTransform_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::depthai_ros_msgs::srv::SetLocalTransform_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::depthai_ros_msgs::srv::SetLocalTransform_Response>()
{
  return depthai_ros_msgs::srv::builder::Init_SetLocalTransform_Response_success();
}

}  // namespace depthai_ros_msgs


namespace depthai_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLocalTransform_Event_response
{
public:
  explicit Init_SetLocalTransform_Event_response(::depthai_ros_msgs::srv::SetLocalTransform_Event & msg)
  : msg_(msg)
  {}
  ::depthai_ros_msgs::srv::SetLocalTransform_Event response(::depthai_ros_msgs::srv::SetLocalTransform_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::depthai_ros_msgs::srv::SetLocalTransform_Event msg_;
};

class Init_SetLocalTransform_Event_request
{
public:
  explicit Init_SetLocalTransform_Event_request(::depthai_ros_msgs::srv::SetLocalTransform_Event & msg)
  : msg_(msg)
  {}
  Init_SetLocalTransform_Event_response request(::depthai_ros_msgs::srv::SetLocalTransform_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetLocalTransform_Event_response(msg_);
  }

private:
  ::depthai_ros_msgs::srv::SetLocalTransform_Event msg_;
};

class Init_SetLocalTransform_Event_info
{
public:
  Init_SetLocalTransform_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLocalTransform_Event_request info(::depthai_ros_msgs::srv::SetLocalTransform_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetLocalTransform_Event_request(msg_);
  }

private:
  ::depthai_ros_msgs::srv::SetLocalTransform_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::depthai_ros_msgs::srv::SetLocalTransform_Event>()
{
  return depthai_ros_msgs::srv::builder::Init_SetLocalTransform_Event_info();
}

}  // namespace depthai_ros_msgs

#endif  // DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__BUILDER_HPP_
