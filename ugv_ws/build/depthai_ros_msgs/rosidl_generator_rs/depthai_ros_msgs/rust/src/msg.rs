#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to depthai_ros_msgs__msg__AutoFocusCtrl

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AutoFocusCtrl {

    // This member is not documented.
    #[allow(missing_docs)]
    pub auto_focus_mode: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub trigger_auto_focus: bool,

}

impl AutoFocusCtrl {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_AUTO: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_MACRO: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_CONTINUOUS_VIDEO: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_CONTINUOUS_PICTURE: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_EDOF: u8 = 4;

}


impl Default for AutoFocusCtrl {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AutoFocusCtrl::default())
  }
}

impl rosidl_runtime_rs::Message for AutoFocusCtrl {
  type RmwMsg = super::msg::rmw::AutoFocusCtrl;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        auto_focus_mode: msg.auto_focus_mode,
        trigger_auto_focus: msg.trigger_auto_focus,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      auto_focus_mode: msg.auto_focus_mode,
      trigger_auto_focus: msg.trigger_auto_focus,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      auto_focus_mode: msg.auto_focus_mode,
      trigger_auto_focus: msg.trigger_auto_focus,
    }
  }
}


// Corresponds to depthai_ros_msgs__msg__HandLandmark

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HandLandmark {
    /// Center of the
    pub label: std::string::String,

    /// Landmarks score.
    pub lm_score: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub landmark: Vec<geometry_msgs::msg::Pose2D>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_spatial: bool,

}



impl Default for HandLandmark {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::HandLandmark::default())
  }
}

impl rosidl_runtime_rs::Message for HandLandmark {
  type RmwMsg = super::msg::rmw::HandLandmark;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        label: msg.label.as_str().into(),
        lm_score: msg.lm_score,
        landmark: msg.landmark
          .into_iter()
          .map(|elem| geometry_msgs::msg::Pose2D::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        is_spatial: msg.is_spatial,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        label: msg.label.as_str().into(),
      lm_score: msg.lm_score,
        landmark: msg.landmark
          .iter()
          .map(|elem| geometry_msgs::msg::Pose2D::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
      is_spatial: msg.is_spatial,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      label: msg.label.to_string(),
      lm_score: msg.lm_score,
      landmark: msg.landmark
          .into_iter()
          .map(geometry_msgs::msg::Pose2D::from_rmw_message)
          .collect(),
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      is_spatial: msg.is_spatial,
    }
  }
}


// Corresponds to depthai_ros_msgs__msg__HandLandmarkArray
/// A list of hand landmark detections

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HandLandmarkArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// A list of the detected proposals. A multi-proposal detector might generate along with the 3D depth information
    ///   this list with many candidate detections generated from a single input.
    pub landmarks: Vec<super::msg::HandLandmark>,

}



impl Default for HandLandmarkArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::HandLandmarkArray::default())
  }
}

impl rosidl_runtime_rs::Message for HandLandmarkArray {
  type RmwMsg = super::msg::rmw::HandLandmarkArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        landmarks: msg.landmarks
          .into_iter()
          .map(|elem| super::msg::HandLandmark::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        landmarks: msg.landmarks
          .iter()
          .map(|elem| super::msg::HandLandmark::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      landmarks: msg.landmarks
          .into_iter()
          .map(super::msg::HandLandmark::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to depthai_ros_msgs__msg__ImuWithMagneticField

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ImuWithMagneticField {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub imu: sensor_msgs::msg::Imu,


    // This member is not documented.
    #[allow(missing_docs)]
    pub field: sensor_msgs::msg::MagneticField,

}



impl Default for ImuWithMagneticField {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ImuWithMagneticField::default())
  }
}

impl rosidl_runtime_rs::Message for ImuWithMagneticField {
  type RmwMsg = super::msg::rmw::ImuWithMagneticField;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        imu: sensor_msgs::msg::Imu::into_rmw_message(std::borrow::Cow::Owned(msg.imu)).into_owned(),
        field: sensor_msgs::msg::MagneticField::into_rmw_message(std::borrow::Cow::Owned(msg.field)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        imu: sensor_msgs::msg::Imu::into_rmw_message(std::borrow::Cow::Borrowed(&msg.imu)).into_owned(),
        field: sensor_msgs::msg::MagneticField::into_rmw_message(std::borrow::Cow::Borrowed(&msg.field)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      imu: sensor_msgs::msg::Imu::from_rmw_message(msg.imu),
      field: sensor_msgs::msg::MagneticField::from_rmw_message(msg.field),
    }
  }
}


// Corresponds to depthai_ros_msgs__msg__TrackedFeature

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackedFeature {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub age: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub harris_score: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tracking_error: f32,

}



impl Default for TrackedFeature {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TrackedFeature::default())
  }
}

impl rosidl_runtime_rs::Message for TrackedFeature {
  type RmwMsg = super::msg::rmw::TrackedFeature;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        id: msg.id,
        age: msg.age,
        harris_score: msg.harris_score,
        tracking_error: msg.tracking_error,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
      id: msg.id,
      age: msg.age,
      harris_score: msg.harris_score,
      tracking_error: msg.tracking_error,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      id: msg.id,
      age: msg.age,
      harris_score: msg.harris_score,
      tracking_error: msg.tracking_error,
    }
  }
}


// Corresponds to depthai_ros_msgs__msg__TrackedFeatures

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackedFeatures {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub features: Vec<super::msg::TrackedFeature>,

}



impl Default for TrackedFeatures {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TrackedFeatures::default())
  }
}

impl rosidl_runtime_rs::Message for TrackedFeatures {
  type RmwMsg = super::msg::rmw::TrackedFeatures;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        features: msg.features
          .into_iter()
          .map(|elem| super::msg::TrackedFeature::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        features: msg.features
          .iter()
          .map(|elem| super::msg::TrackedFeature::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      features: msg.features
          .into_iter()
          .map(super::msg::TrackedFeature::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to depthai_ros_msgs__msg__SpatialDetection

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SpatialDetection {
    /// Class probabilities
    pub results: Vec<vision_msgs::msg::ObjectHypothesis>,

    /// 2D bounding box surrounding the object.
    pub bbox: vision_msgs::msg::BoundingBox2D,

    /// Center of the detected object in meters
    pub position: geometry_msgs::msg::Point,

    /// If true, this message contains object tracking information.
    pub is_tracking: bool,

    /// ID used for consistency across multiple detection messages. This value will
    /// likely differ from the id field set in each individual ObjectHypothesis.
    /// If you set this field, be sure to also set is_tracking to True.
    pub tracking_id: std::string::String,

}



impl Default for SpatialDetection {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SpatialDetection::default())
  }
}

impl rosidl_runtime_rs::Message for SpatialDetection {
  type RmwMsg = super::msg::rmw::SpatialDetection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        results: msg.results
          .into_iter()
          .map(|elem| vision_msgs::msg::ObjectHypothesis::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        bbox: vision_msgs::msg::BoundingBox2D::into_rmw_message(std::borrow::Cow::Owned(msg.bbox)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        is_tracking: msg.is_tracking,
        tracking_id: msg.tracking_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        results: msg.results
          .iter()
          .map(|elem| vision_msgs::msg::ObjectHypothesis::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        bbox: vision_msgs::msg::BoundingBox2D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bbox)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
      is_tracking: msg.is_tracking,
        tracking_id: msg.tracking_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      results: msg.results
          .into_iter()
          .map(vision_msgs::msg::ObjectHypothesis::from_rmw_message)
          .collect(),
      bbox: vision_msgs::msg::BoundingBox2D::from_rmw_message(msg.bbox),
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      is_tracking: msg.is_tracking,
      tracking_id: msg.tracking_id.to_string(),
    }
  }
}


// Corresponds to depthai_ros_msgs__msg__SpatialDetectionArray
/// A list of 2D detections, for a multi-object 2D detector along with 3D depth information.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SpatialDetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// A list of the detected proposals. A multi-proposal detector might generate along with the 3D depth information
    ///   this list with many candidate detections generated from a single input.
    pub detections: Vec<super::msg::SpatialDetection>,

}



impl Default for SpatialDetectionArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SpatialDetectionArray::default())
  }
}

impl rosidl_runtime_rs::Message for SpatialDetectionArray {
  type RmwMsg = super::msg::rmw::SpatialDetectionArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detections: msg.detections
          .into_iter()
          .map(|elem| super::msg::SpatialDetection::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        detections: msg.detections
          .iter()
          .map(|elem| super::msg::SpatialDetection::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detections: msg.detections
          .into_iter()
          .map(super::msg::SpatialDetection::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to depthai_ros_msgs__msg__TrackDetection2D

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackDetection2D {
    /// Class probabilities
    pub results: Vec<vision_msgs::msg::ObjectHypothesisWithPose>,

    /// 2D bounding box surrounding the object.
    pub bbox: vision_msgs::msg::BoundingBox2D,

    /// If true, this message contains object tracking information.
    pub is_tracking: bool,

    /// ID used for consistency across multiple detection messages. This value will
    /// likely differ from the id field set in each individual ObjectHypothesis.
    /// If you set this field, be sure to also set is_tracking to True.
    pub tracking_id: std::string::String,

    /// Age: number of frames the object is being tracked
    pub tracking_age: i32,

    /// Status of the tracking:
    /// 0 = NEW -> the object is newly added.
    /// 1 = TRACKED -> the object is being tracked.
    /// 2 = LOST -> the object gets lost now. The object can be tracked again automatically (long term tracking)
    ///     or by specifying detected object manually (short term and zero term tracking).
    /// 3 = REMOVED -> the object is removed.
    pub tracking_status: i32,

}



impl Default for TrackDetection2D {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TrackDetection2D::default())
  }
}

impl rosidl_runtime_rs::Message for TrackDetection2D {
  type RmwMsg = super::msg::rmw::TrackDetection2D;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        results: msg.results
          .into_iter()
          .map(|elem| vision_msgs::msg::ObjectHypothesisWithPose::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        bbox: vision_msgs::msg::BoundingBox2D::into_rmw_message(std::borrow::Cow::Owned(msg.bbox)).into_owned(),
        is_tracking: msg.is_tracking,
        tracking_id: msg.tracking_id.as_str().into(),
        tracking_age: msg.tracking_age,
        tracking_status: msg.tracking_status,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        results: msg.results
          .iter()
          .map(|elem| vision_msgs::msg::ObjectHypothesisWithPose::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        bbox: vision_msgs::msg::BoundingBox2D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bbox)).into_owned(),
      is_tracking: msg.is_tracking,
        tracking_id: msg.tracking_id.as_str().into(),
      tracking_age: msg.tracking_age,
      tracking_status: msg.tracking_status,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      results: msg.results
          .into_iter()
          .map(vision_msgs::msg::ObjectHypothesisWithPose::from_rmw_message)
          .collect(),
      bbox: vision_msgs::msg::BoundingBox2D::from_rmw_message(msg.bbox),
      is_tracking: msg.is_tracking,
      tracking_id: msg.tracking_id.to_string(),
      tracking_age: msg.tracking_age,
      tracking_status: msg.tracking_status,
    }
  }
}


// Corresponds to depthai_ros_msgs__msg__TrackDetection2DArray
/// A list of 2D tracklets, for a multi-object 2D tracker.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackDetection2DArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// A list of the tracking proposals.
    pub detections: Vec<super::msg::TrackDetection2D>,

}



impl Default for TrackDetection2DArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TrackDetection2DArray::default())
  }
}

impl rosidl_runtime_rs::Message for TrackDetection2DArray {
  type RmwMsg = super::msg::rmw::TrackDetection2DArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detections: msg.detections
          .into_iter()
          .map(|elem| super::msg::TrackDetection2D::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        detections: msg.detections
          .iter()
          .map(|elem| super::msg::TrackDetection2D::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detections: msg.detections
          .into_iter()
          .map(super::msg::TrackDetection2D::from_rmw_message)
          .collect(),
    }
  }
}


