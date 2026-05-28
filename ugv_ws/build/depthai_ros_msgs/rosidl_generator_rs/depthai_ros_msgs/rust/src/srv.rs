#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to depthai_ros_msgs__srv__TriggerNamed_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TriggerNamed_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub name: std::string::String,

}



impl Default for TriggerNamed_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::TriggerNamed_Request::default())
  }
}

impl rosidl_runtime_rs::Message for TriggerNamed_Request {
  type RmwMsg = super::srv::rmw::TriggerNamed_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      name: msg.name.to_string(),
    }
  }
}


// Corresponds to depthai_ros_msgs__srv__TriggerNamed_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TriggerNamed_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for TriggerNamed_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::TriggerNamed_Response::default())
  }
}

impl rosidl_runtime_rs::Message for TriggerNamed_Response {
  type RmwMsg = super::srv::rmw::TriggerNamed_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to depthai_ros_msgs__srv__NormalizedImageCrop_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NormalizedImageCrop_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub top_left: geometry_msgs::msg::Pose2D,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bottom_right: geometry_msgs::msg::Pose2D,

}



impl Default for NormalizedImageCrop_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::NormalizedImageCrop_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NormalizedImageCrop_Request {
  type RmwMsg = super::srv::rmw::NormalizedImageCrop_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        top_left: geometry_msgs::msg::Pose2D::into_rmw_message(std::borrow::Cow::Owned(msg.top_left)).into_owned(),
        bottom_right: geometry_msgs::msg::Pose2D::into_rmw_message(std::borrow::Cow::Owned(msg.bottom_right)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        top_left: geometry_msgs::msg::Pose2D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.top_left)).into_owned(),
        bottom_right: geometry_msgs::msg::Pose2D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bottom_right)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      top_left: geometry_msgs::msg::Pose2D::from_rmw_message(msg.top_left),
      bottom_right: geometry_msgs::msg::Pose2D::from_rmw_message(msg.bottom_right),
    }
  }
}


// Corresponds to depthai_ros_msgs__srv__NormalizedImageCrop_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NormalizedImageCrop_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i64,

}



impl Default for NormalizedImageCrop_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::NormalizedImageCrop_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NormalizedImageCrop_Response {
  type RmwMsg = super::srv::rmw::NormalizedImageCrop_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
    }
  }
}






#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__depthai_ros_msgs__srv__TriggerNamed() -> *const std::ffi::c_void;
}

// Corresponds to depthai_ros_msgs__srv__TriggerNamed
#[allow(missing_docs, non_camel_case_types)]
pub struct TriggerNamed;

impl rosidl_runtime_rs::Service for TriggerNamed {
    type Request = TriggerNamed_Request;
    type Response = TriggerNamed_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__depthai_ros_msgs__srv__TriggerNamed() }
    }
}




#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__depthai_ros_msgs__srv__NormalizedImageCrop() -> *const std::ffi::c_void;
}

// Corresponds to depthai_ros_msgs__srv__NormalizedImageCrop
#[allow(missing_docs, non_camel_case_types)]
pub struct NormalizedImageCrop;

impl rosidl_runtime_rs::Service for NormalizedImageCrop {
    type Request = NormalizedImageCrop_Request;
    type Response = NormalizedImageCrop_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__depthai_ros_msgs__srv__NormalizedImageCrop() }
    }
}


