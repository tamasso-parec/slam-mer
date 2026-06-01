/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file frame.hpp
/// @brief Camera-frame abstraction used by the localization module.
/// @details Declares `localization::Frame`, which stores synchronized sensor
/// data, extracted keypoints and descriptors, pose estimates, and
/// frame-to-map association data used by matching and tracking routines.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef FRAME_H
#define FRAME_H

// SLAM-MER includes
#include <representation.hpp>
#include <sensor_data.hpp>
#include <settings.hpp>
#include <types/descriptor.hpp>
#include <types/mutex.hpp>
#include <types/point2d.hpp>
#include <types/point3d.hpp>
#include <types/pose.hpp>

namespace slam_mer {
namespace localization {

/// @brief Class for the camera frame.
class Frame {
public:
  /// Shared pointer alias for Frame.
  using shared_ptr = std::shared_ptr<Frame>;
  /// Weak pointer alias for Frame.
  using weak_ptr = std::weak_ptr<Frame>;

  /// @brief Create a shared Frame instance from sensor data.
  /// @param cameraData_ Weak pointer to sensor data associated with the frame.
  /// @return Shared pointer to the created Frame object.
  [[nodiscard("Use returned shared_ptr to keep frame ownership")]]
  static Frame::shared_ptr make_shared(
      const data_type::SensorData::weak_ptr cameraData_) {
    return std::make_shared<Frame>(cameraData_);
  }

  /// @brief Construct a frame and initialize image features from sensor input.
  /// @param cameraData_ Weak pointer to sensor data for this frame.
  Frame(const data_type::SensorData::weak_ptr cameraData_);

  /// @brief Destroy the frame object.
  inline ~Frame() {}

  /// @brief Checks if the camera data is from a monocular camera.
  /// @return `true` if sensor type is monocular, otherwise `false`.
  [[nodiscard("Use returned bool to branch on camera modality")]]
  inline bool is_monocular() const {
    return _cameraData->get_sensor_type() == data_type::SensorData::MONOCULAR;
  }

  /// @brief Get the sensor modality associated with this frame.
  /// @return Sensor type value from the underlying sensor data.
  [[nodiscard("Use returned sensor type to select localization pipeline path")]]
  inline data_type::SensorData::SensorType get_sensor_type() const {
    return _cameraData->get_sensor_type();
  }

  /// @brief Get frame identifier.
  /// @return Frame ID.
  [[nodiscard("Use returned frame ID to reference this frame in buffers")]]
  inline frame_id_t get_id() const {
    return _id;
  }

  /// @brief Set frame pose in world coordinates.
  /// @param pose_ New frame pose.
  inline void set_pose(const pose_t& pose_) { _pose = pose_; }

  /// @brief Get frame pose in world coordinates.
  /// @return Pose of this frame.
  [[nodiscard("Use returned pose for localization, mapping, or visualization")]]
  inline pose_t get_pose() const {
    return _pose;
  }

  /// @brief Get weak pointer to camera data associated with this frame.
  /// @return Weak pointer to sensor data.
  [[nodiscard("Use returned sensor pointer to query frame camera payload")]]
  inline data_type::SensorData::weak_ptr get_camera_data() const {
    return _cameraData;
  }

  /// @brief Set number of tracked points matched for this frame.
  /// @param numberTrackedPoints_ Number of matched points.
  inline void set_number_tracked_points(const int numberTrackedPoints_) {
    _numberTrackedPoints = numberTrackedPoints_;
  }

  /// @brief Get number of tracked points matched for this frame.
  /// @return Number of matched points.
  [[nodiscard("Use returned tracked-point count for keyframe decisions")]]
  inline int get_number_tracked_points() const {
    return _numberTrackedPoints;
  }

  /// @brief Used to indicate that a point seen by the frame is in the map
  /// already and as map point id equal to pointId (input value).
  /// @param index_ Index of the point in frame feature arrays.
  /// @param pointId_ Map-point ID corresponding to the indexed frame point.
  inline void update_point_id(size_t index_, point_id_t pointId_) {
    _allDetectedImagePointIds[index_] = pointId_;
  }

  /// @brief Used to indicate that a point seen by the frame is in the map
  /// already and as map point id equal to pointId (input value).
  /// @param index_ Index of the point in frame feature arrays.
  /// @param pointId_ Map-point ID corresponding to the indexed frame point.
  inline void update_mappoint_id(size_t index_, point_id_t pointId_) {
    _pointsInMapIds[index_] = pointId_;
  }

  /// @brief Compute the 3D points for each 2D keyfeature.
  /// @note Assumes RGB-D observations are available to recover depth-backed
  /// 3D points for keyframe-upgrade data.
  void setup_upgrade_to_keyframe();

  /// @brief Get 3D points expressed in the camera coordinate frame.
  /// @return Const reference to 3D points in camera frame.
  [[nodiscard("Use returned 3D points for pose estimation and map updates")]]
  inline const point3d_t::vector& get_3d_points_camera_frame() const {
    return _pointsCameraFrame;
  }

  /// @brief Get IDs of map points associated with this frame.
  /// @return Const reference to map-point IDs.
  [[nodiscard("Use returned map-point IDs to access map observations")]]
  inline const point_id_t::vector& get_mappoint_ids() const {
    return _pointsInMapIds;
  }

  /// @brief Get 2D image points used in localization for this frame.
  /// @return Const reference to selected image keypoints.
  [[nodiscard("Use returned keypoints for matching and tracking diagnostics")]]
  inline const point2d_t::vector& get_image_points() const {
    return _pointsImage;
  }

  /// @brief Get descriptors associated with selected frame keypoints.
  /// @return Const reference to descriptor vector.
  [[nodiscard("Use returned descriptors for feature matching")]]
  inline const descriptor_t::vector& get_image_descriptors() const {
    return _pointsImageDescriptors;
  }

  /// @brief Get all detected image keypoints before localization filtering.
  /// @return Const reference to all detected image points.
  [[nodiscard("Use returned detected points for debug and visualization")]]
  inline const point2d_t::vector& get_all_detected_image_points() const {
    return _allDetectedImagePoints;
  }

  /// @brief Get descriptors for all detected image keypoints.
  /// @return Const reference to all detected image descriptors.
  [[nodiscard("Use returned detected descriptors for matching diagnostics")]]
  inline const descriptor_t::vector& get_all_detected_image_descriptors()
      const {
    return _allDetectedImageDescriptors;
  }

  /// @brief Get the neighbors of a point.
  /// @note This function fills the neighbors variable that is passed
  /// by reference, to avoid copying long maps from the caller.
  /// @param keyframeId_ Keyframe ID associated with this frame.
  /// @param neighbors_ Output map of neighbors for each point.
  void get_neighbors(
      const keyframe_id_t& keyframeId_,
      std::unordered_map<point_id_t, neighbor_t::vector>& neighbors_) const;

  /// @brief Check whether frame construction succeeded.
  /// @return `true` if frame has valid feature content, otherwise `false`.
  [[nodiscard("Use returned bool to validate frame creation status")]]
  inline bool check_frame_success() {
    return _createdWithSuccess;
  }

  /// @brief Apply a uniform scale factor to frame 3D points.
  /// @param scale_ Scale factor to apply.
  void apply_scale_to_points3d(const double scale_);

  /// @brief Sets the ID of the reference keyframe of this frame.
  /// Reference keyframe is the keyframe to which this frame has the
  /// most points in common.
  /// @param keyframeId_ Reference keyframe ID.
  inline void set_reference_keyframe_id(const keyframe_id_t keyframeId_) {
    _referenceKeyframe = keyframeId_;
  }

  /// @brief Get the ID of the reference keyframe of this frame.
  /// @return Reference keyframe ID.
  [[nodiscard("Use returned keyframe ID to index reference keyframe data")]]
  inline keyframe_id_t get_reference_keyframe_id() const {
    return _referenceKeyframe;
  }

  /// @brief Sets the ID of the keyframe this frame is upgraded to.
  /// @param keyframeId_ Keyframe ID created from this frame.
  inline void set_upgraded_to_keyframe_id(const keyframe_id_t keyframeId_) {
    _upgradedToKeyframeId = keyframeId_;
  }

  /// @brief Gets the ID of the keyframe this frame is upgraded to.
  /// @return Keyframe ID this frame was upgraded to.
  [[nodiscard("Use returned keyframe ID to check promotion result")]]
  inline keyframe_id_t get_upgraded_to_keyframe_id() const {
    return _upgradedToKeyframeId;
  }

  /// @brief Checks if the frame was upgraded to a keyframe.
  /// @return True if the frame was upgraded to a keyframe, False otherwise.
  [[nodiscard("Use returned bool to branch on keyframe promotion state")]]
  inline bool is_keyframe() const {
    return _upgradedToKeyframeId != keyframe_id_t::UNSET;
  }

  /// @brief Check if the frame has the necessary data to be a keyframe.
  /// @return True if the frame has the necessary data to be a keyframe,
  /// False otherwise.
  [[nodiscard("Use returned bool to validate keyframe-upgrade requirements")]]
  inline bool valid_keyframe_upgrade() const {
    using slam_mer::settings::sharedSettings;
    return (_pointsCameraFrame.size() == _pointsImage.size()) &&
           (_pointsCameraFrame.size() == _pointsImageDescriptors.size()) &&
           (_pointsCameraFrame.size() == _pointsInMapIds.size()) &&
           (_pointsCameraFrame.size() >
            static_cast<size_t>(sharedSettings->minimum_frame_features()));
  }

private:
  /// @brief Check whether detected features meet minimum frame requirements.
  /// @return `true` if frame has enough detected points, otherwise `false`.
  [[nodiscard("Use returned bool for internal frame validity checks")]]
  inline bool __check_frame_success() {
    using slam_mer::settings::sharedSettings;
    const int& numberFeatures = sharedSettings->minimum_frame_features();
    const size_t minimumPoints = static_cast<size_t>(numberFeatures);
    return _allDetectedImagePoints.size() > minimumPoints;
  }

  /// Frame Id
  frame_id_t _id;

  /// Frame Pose in world coordinates
  pose_t _pose = pose_t::Identity();
  /// camera data (can be stereo, monocular, or RGB-D from different datasets)
  data_type::SensorData::shared_ptr _cameraData;
  /// To save the number of keypoints successfully matches with a previous
  /// frame/keyframe/buffer (useful for keyframe-creation decisions)
  int _numberTrackedPoints = 0;
  /// Minimum depth for the 3D points
  double _minDepth = 0.1;
  /// Maximum depth for the 3D points
  double _maxDepth = 10.0;
  /// Flag that indicate if the frame was successfully created
  bool _createdWithSuccess = true;

  /// Id of the reference keyframe
  keyframe_id_t _referenceKeyframe = keyframe_id_t::UNSET;
  /// Note that not all frames are upgraded
  keyframe_id_t _upgradedToKeyframeId = keyframe_id_t::UNSET;

  /// Keep all the detected image points for better accuracy latter
  point2d_t::vector _allDetectedImagePoints;
  /// Image Descriptors
  descriptor_t::vector _allDetectedImageDescriptors;
  /// Ids of Mappoints for the image points that are matched to the map
  point_id_t::vector _allDetectedImagePointIds;

  /// 3D points seen by the frame in its coordinate frame
  point3d_t::vector _pointsCameraFrame;
  /// 2D image points
  point2d_t::vector _pointsImage;
  /// Descriptors of the 2D/3D points
  descriptor_t::vector _pointsImageDescriptors;
  /// Ids of all the detected mappoints in the image
  point_id_t::vector _pointsInMapIds;
};

}  // namespace localization
}  // namespace slam_mer

#endif  // FRAME_H
