/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file covisibility_factors.hpp
/// @brief Custom GTSAM factor definitions for covisibility optimization.
/// @details Declares factor classes and helpers used by adjustment routines to
/// model projection, distance, and odometry constraints in the covisibility
/// graph backend built on top of GTSAM.
/// @author Pedro Miraldo
/// @author Valter Piedade
#ifndef COVISIBILITY_FACTORS_HPP
#define COVISIBILITY_FACTORS_HPP

#include <sim3_factor_derivatives.hpp>
#include <types/id.hpp>
#include <types/output.hpp>

#include <type_traits>

// Necessary GTSAM headers for SLAM
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
// #include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace slam_mer {
namespace covisibility_graph {

/// @class PointDistanceFactor
/// @brief Class for a factor representing the distance between a
/// transformed 3D point from world to camera reference frame and
/// a measured point in the camera reference frame.
/// @details This factor constrains a 3D point (in world coordinates)
/// transformed by a pose, and computes the error as the difference to a
/// measured point in the camera frame. It is useful for enforcing geometric
/// consistency between poses and points in SLAM.
/// @tparam TransformationType The type representing the camera pose
/// (e.g., gtsam::Pose3, gtsam::Similarity3).
template <typename TransformationType>
class PointDistanceFactor
    : public gtsam::NoiseModelFactor3<TransformationType, gtsam::Point3> {

public:
  /// Typedef for the base gtsam class
  using Base = gtsam::NoiseModelFactor3<TransformationType, gtsam::Point3>;
  using Base::evaluateError;
  /// Typedef for the base gtsam class
  using PointDistanceFactorTyped = PointDistanceFactor<TransformationType>;

  /// Typedef for a std::shared_ptr of a PointDistanceFactor
  using shared_ptr = std::shared_ptr<PointDistanceFactorTyped>;
  /// @brief Create a shared point-distance factor.
  /// @param keyframeId_ Keyframe ID associated with the observation.
  /// @param pointId_ Point ID associated with the observation.
  /// @param measured_ Measured 3D point in the camera frame.
  /// @param noiseModel_ Noise model used for this factor.
  /// @return Shared pointer owning the created point-distance factor.
  [[nodiscard("Use returned shared_ptr to keep factor ownership")]]
  static shared_ptr make_shared(
      const keyframe_id_t keyframeId_,
      const point_id_t pointId_,
      gtsam::Point3 measured_,
      const gtsam::SharedNoiseModel& noiseModel_) {
    return std::make_shared<PointDistanceFactorTyped>(
        keyframeId_,
        pointId_,
        measured_,
        noiseModel_);
  }

  /// @brief Clone a point-distance factor with updated graph keys.
  /// @param other_ Source factor to clone.
  /// @param keyframeId_ New keyframe ID used in the cloned factor.
  /// @param pointId_ New point ID used in the cloned factor.
  /// @return Shared pointer owning the cloned point-distance factor.
  [[nodiscard("Use returned shared_ptr to keep factor ownership")]]
  static shared_ptr make_shared(
      const PointDistanceFactor::shared_ptr& other_,
      const keyframe_id_t keyframeId_,
      const point_id_t pointId_) {
    return std::make_shared<PointDistanceFactorTyped>(
        other_,
        keyframeId_,
        pointId_);
  }

  /// A static empty pointer for PointDistanceFactor, useful for
  /// initialization.
  inline static const shared_ptr EMPTY = nullptr;

  /// Measured 3D point in the camera frame
  gtsam::Point3 _measured;

  /// @brief Construct a PointDistanceFactor object
  /// @param keyframeId_ ID of the keyframe where the keypoint was detected.
  /// @param pointId_ ID of the map point associated with the measurement.
  /// @param measured_ The 3D point measurement in the camera frame.
  /// @param noiseModel_ The noise model for the factor.
  PointDistanceFactor(
      const keyframe_id_t keyframeId_,
      const point_id_t pointId_,
      gtsam::Point3 measured_,
      const gtsam::SharedNoiseModel& noiseModel_)
      : Base(
            noiseModel_,
            gtsam::Symbol('k', keyframeId_.key()),
            gtsam::Symbol('p', pointId_.key())),
        _measured(measured_) {}

  /// @brief Construct a PointDistanceFactor object from a previously existing
  /// factor. It basically copies everything from the input factor, and
  /// updates the keys for the keyframe and point.
  /// @param other_ The factor to copy from.
  /// @param keyframeId_ The new keyframe ID.
  /// @param pointId_ The new map point ID.
  PointDistanceFactor(
      const PointDistanceFactor::shared_ptr& other_,
      const keyframe_id_t keyframeId_,
      const point_id_t pointId_)
      : Base(
            other_->noiseModel_,
            gtsam::Symbol('k', keyframeId_.key()),
            gtsam::Symbol('p', pointId_.key())),
        _measured(other_->_measured) {}

  /// @brief Point distance error evaluation function. This is the
  /// function that computes the point distance error between the
  /// observed 3D point and the transformed 3D point that is used inside
  /// gtsam optimization steps.
  /// @param pose_ The camera pose.
  /// @param point_ The 3D point in the world.
  /// @param H1_ Optional Jacobian with respect to the pose.
  /// @param H2_ Optional Jacobian with respect to the point.
  /// @return The distance error vector.
  gtsam::Vector evaluateError(
      const TransformationType& pose_,
      const gtsam::Point3& point_,
      gtsam::OptionalMatrixType H1_,
      gtsam::OptionalMatrixType H2_) const override {

    if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>) {
      return sim3_camera_point(pose_, point_, H1_, H2_) - this->_measured;
    }

    gtsam::Matrix H_point_cam_wrt_Tinv;
    gtsam::Matrix H_point_cam_wrt_point;

    gtsam::Matrix H_Tinv_T;
    TransformationType Tinv = pose_.inverse(H_Tinv_T);

    gtsam::Point3 pointCameraFrame = Tinv.transformFrom(
        point_,
        H1_ ? &H_point_cam_wrt_Tinv : nullptr,
        H2_ ? &H_point_cam_wrt_point : nullptr);

    if (H1_) *H1_ = H_point_cam_wrt_Tinv * H_Tinv_T;
    if (H2_) *H2_ = H_point_cam_wrt_point;

    gtsam::Point3 reprojectionError = -this->_measured + pointCameraFrame;
    return reprojectionError;
  }
};

/// @class ProjectionFactor
/// @brief Class for a factor representing the reprojection of a 3D point
/// onto a 2D image plane using camera intrinsics.
/// @details This factor constrains a 3D point and a camera pose by measuring
/// the difference between the observed 2D pixel coordinates and the projected
/// coordinates of the 3D point onto the image plane using the camera model.
/// @tparam TransformationType The type representing the camera pose
/// (e.g., gtsam::Pose3, gtsam::Similarity3).
template <typename TransformationType>
class ProjectionFactor
    : public gtsam::NoiseModelFactor2<TransformationType, gtsam::Point3> {

public:
  /// Typedef for the base gtsam class
  using Base = gtsam::NoiseModelFactor2<TransformationType, gtsam::Point3>;
  using Base::evaluateError;

  /// Typed alias for the ProjectionFactor class template
  using ProjectionFactorTyped = ProjectionFactor<TransformationType>;

  /// Typedef for a std::shared_ptr of a ProjectionFactor
  using shared_ptr = std::shared_ptr<ProjectionFactorTyped>;
  /// @brief Create a shared projection factor.
  /// @param keyframeId_ Keyframe ID associated with the observation.
  /// @param pointId_ Point ID associated with the observation.
  /// @param intrinsics_ Camera intrinsics used for reprojection.
  /// @param measured_ Measured 2D keypoint coordinates.
  /// @param noiseModel_ Noise model used for this factor.
  /// @return Shared pointer owning the created projection factor.
  [[nodiscard("Use returned shared_ptr to keep factor ownership")]]
  static shared_ptr make_shared(
      const keyframe_id_t keyframeId_,
      const point_id_t pointId_,
      const gtsam::Cal3_S2::shared_ptr& intrinsics_,
      gtsam::Point2 measured_,
      const gtsam::SharedNoiseModel& noiseModel_) {
    return std::make_shared<ProjectionFactorTyped>(
        keyframeId_,
        pointId_,
        intrinsics_,
        measured_,
        noiseModel_);
  }

  /// @brief Make an object for a shared pointer for a ProjectionFactor from a
  /// previously existing factor.
  /// @param other_ The factor to copy from.
  /// @param keyframeId_ The new keyframe ID.
  /// @param pointId_ The new map point ID.
  /// @return Shared pointer owning the cloned projection factor.
  [[nodiscard("Use returned shared_ptr to keep factor ownership")]]
  static shared_ptr make_shared(
      const ProjectionFactor::shared_ptr& other_,
      const keyframe_id_t keyframeId_,
      const point_id_t pointId_) {
    return std::make_shared<ProjectionFactorTyped>(
        other_,
        keyframeId_,
        pointId_);
  }

  /// A static empty pointer for ProjectionFactor, useful for initialization
  inline static const shared_ptr EMPTY = nullptr;

  /// Observed 2D pixel coordinates (measurement)
  gtsam::Point2 _measured;
  /// Camera intrinsics
  gtsam::Cal3_S2::shared_ptr _intrinsics;

  /// @brief Construct a ProjectionFactor object
  /// @param keyframeId_ ID of the keyframe where the keypoint was detected.
  /// @param pointId_ ID of the map point associated with the measurement.
  /// @param intrinsics_ The camera calibration (intrinsics).
  /// @param measured_ The keypoint pixel coordinates.
  /// @param noiseModel_ The noise model for the factor.
  ProjectionFactor(
      const keyframe_id_t keyframeId_,
      const point_id_t pointId_,
      const gtsam::Cal3_S2::shared_ptr& intrinsics_,
      gtsam::Point2 measured_,
      const gtsam::SharedNoiseModel& noiseModel_)
      : Base(
            noiseModel_,
            gtsam::Symbol('k', keyframeId_.key()),
            gtsam::Symbol('p', pointId_.key())),
        _measured(measured_),
        _intrinsics(intrinsics_) {}

  /// @brief Construct a ProjectionFactor object from a previously existing
  /// factor. It basically copies everything from the input factor, and
  /// updates the keys for the keyframe and point.
  /// @param other_ The factor to copy from.
  /// @param keyframeId_ The new keyframe ID.
  /// @param pointId_ The new map point ID.
  ProjectionFactor(
      const ProjectionFactor::shared_ptr& other_,
      const keyframe_id_t keyframeId_,
      const point_id_t pointId_)
      : Base(
            other_->noiseModel_,
            gtsam::Symbol('k', keyframeId_.key()),
            gtsam::Symbol('p', pointId_.key())),
        _measured(other_->_measured),
        _intrinsics(other_->_intrinsics) {}

  /// @brief Sets the calibration for be used by the factor.
  /// @param intrinsics_ camera intrinsics.
  void set_calibration(gtsam::Cal3_S2::shared_ptr intrinsics_) {
    this->_intrinsics = intrinsics_;
  }

  /// @brief Checks if the factor has a calibration.
  /// @return true if calibration is set, false otherwise.
  [[nodiscard("Use returned bool to guard projection factor calibration use")]]
  bool has_calibration() const {
    return this->_intrinsics != nullptr;
  }

  /// @brief Gets the gtsam::Key for the map point associated with this factor.
  /// @return Key of the map point associated with this factor.
  [[nodiscard("Use returned key to index map-point variables in GTSAM")]]
  gtsam::Key get_key() const {
    return this->key2();
  }

  /// @brief Projection error evaluation function.
  /// This is the function that computes the reprojection error between the
  /// observed 2D point and the projected 3D point that is used inside gtsam
  /// optimization steps.
  /// @param pose_ The camera pose.
  /// @param point_ The 3D point in the world.
  /// @param H1_ Optional Jacobian with respect to the pose.
  /// @param H2_ Optional Jacobian with respect to the point.
  /// @return The reprojection error vector.
  gtsam::Vector evaluateError(
      const TransformationType& pose_,
      const gtsam::Point3& point_,
      gtsam::OptionalMatrixType H1_,
      gtsam::OptionalMatrixType H2_) const override {
    using gtsam::Matrix;
    using gtsam::Matrix22;
    using gtsam::Matrix23;
    using gtsam::Point2;
    using gtsam::Point3;

    Matrix Dpc_dpose;
    Matrix Dpc_dpoint;
    Point3 pc;

    if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>) {
      pc = sim3_camera_point(
          pose_,
          point_,
          H1_ ? &Dpc_dpose : nullptr,
          H2_ ? &Dpc_dpoint : nullptr);
    } else if constexpr (requires(
                             const TransformationType& P,
                             const Point3& X,
                             Matrix* A,
                             Matrix* B) { P.transformTo(X, A, B); }) {
      pc = pose_.transformTo(
          point_,
          H1_ ? &Dpc_dpose : nullptr,
          H2_ ? &Dpc_dpoint : nullptr);
    } else {
      Matrix Dpc_dSinv;
      Matrix Dimv_dpose;
      auto pinv = pose_.inverse(Dimv_dpose);

      pc = pinv.transformFrom(
          point_,
          H1_ ? &Dpc_dSinv : nullptr,
          H2_ ? &Dpc_dpoint : nullptr);

      if (H1_) Dpc_dpose = Dpc_dSinv * Dimv_dpose;
    }

    const double x = pc.x(), y = pc.y(), z = pc.z();
    const double invz = 1.0 / z, invz2 = invz * invz;

    Point2 pn(x * invz, y * invz);

    Matrix23 Dpn_dpc;
    Dpn_dpc << invz, 0.0, -x * invz2, 0.0, invz, -y * invz2;

    Matrix22 Duv_dpn;
    Point2 uv = _intrinsics->uncalibrate(
        pn,
        nullptr,
        (H1_ || H2_) ? &Duv_dpn : nullptr);

    if (H1_) *H1_ = Duv_dpn * Dpn_dpc * Dpc_dpose;
    if (H2_) *H2_ = Duv_dpn * Dpn_dpc * Dpc_dpoint;

    return uv - _measured;
  }
};

/// @class OdometryFactor
/// @brief Factor representing an odometry measurement between two keyframes.
/// @tparam TransformationType The type representing the camera pose
/// (e.g., gtsam::Pose3, gtsam::Similarity3).
template <typename TransformationType>
class OdometryFactor : public gtsam::BetweenFactor<TransformationType> {

public:
  using Base = gtsam::BetweenFactor<TransformationType>;
  using Base::evaluateError;
  /// Typedef for a std::shared_ptr of an OdometryFactor
  using shared_ptr = std::shared_ptr<OdometryFactor<TransformationType>>;

  /// Typed alias for the OdometryFactor class template
  using OdometryFactorTyped = OdometryFactor<TransformationType>;

  /// @brief Create a shared pointer to an OdometryFactor.
  /// @param keyframeId1_ Id of Keyframe 1
  /// @param keyframeId2_ Id of Keyframe 2
  /// @param odometry_ The measured relative pose (from keyframe 1 to keyframe
  ///   2)
  /// @param noiseModel_ The noise model for the factor.
  /// @return A shared pointer to the created OdometryFactor.
  [[nodiscard("Use returned shared_ptr to keep factor ownership")]]
  static shared_ptr make_shared(
      const keyframe_id_t keyframeId1_,
      const keyframe_id_t keyframeId2_,
      const TransformationType& odometry_,
      const gtsam::SharedNoiseModel& noiseModel_) {
    return std::make_shared<OdometryFactorTyped>(
        keyframeId1_,
        keyframeId2_,
        odometry_,
        noiseModel_);
  }

  /// @brief Construct a OdometryFactor object
  /// @param keyframeId1_ Id of Keyframe 1
  /// @param keyframeId2_ Id of Keyframe 2
  /// @param odometry_ The measured relative pose (from keyframe 1 to keyframe
  ///   2)
  /// @param noiseModel_ The noise model for the factor.
  OdometryFactor(
      const keyframe_id_t keyframeId1_,
      const keyframe_id_t keyframeId2_,
      const TransformationType& odometry_,
      const gtsam::SharedNoiseModel& noiseModel_)
      : gtsam::BetweenFactor<TransformationType>(
            gtsam::Symbol('k', keyframeId1_.key()),
            gtsam::Symbol('k', keyframeId2_.key()),
            odometry_,
            noiseModel_) {}

  /// @brief Preserve the residual and correct the Sim(3) chart derivatives.
  /// @details Includes the Local derivative at nonzero residual. Pose3 keeps
  /// the pinned GTSAM behavior. Sim(3) uses analytic chart derivatives.
  gtsam::Vector evaluateError(
      const TransformationType& first_,
      const TransformationType& second_,
      gtsam::OptionalMatrixType H1_,
      gtsam::OptionalMatrixType H2_) const override {
    if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>) {
      const auto relative = first_.between(second_);
      const auto error = this->measured().between(relative);
      gtsam::Matrix local;
      const auto residual = sim3_logmap(error, (H1_ || H2_) ? &local : nullptr);
      if (H1_) *H1_ = -local * sim3_adjoint(relative.inverse());
      if (H2_) *H2_ = local;
      return residual;
    } else {
      return Base::evaluateError(first_, second_, H1_, H2_);
    }
  }

  /// @brief Clone without slicing away the corrected derivatives.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<OdometryFactorTyped>(*this);
  }
};

/// @brief Pose prior preserving GTSAM residuals with consistent Sim(3) derivatives.
/// @details Pose3 keeps the native Jacobian. Sim(3) differentiates the native
/// prior residual in its actual retraction rather than assuming identity away
/// from the prior origin. Noise models and mean semantics remain native.
template <typename TransformationType>
class PosePriorFactor : public gtsam::PriorFactor<TransformationType> {
public:
  using Base = gtsam::PriorFactor<TransformationType>;
  using Base::Base;
  using Base::evaluateError;

  /// @brief Evaluate the native prior residual and optional local Jacobian.
  gtsam::Vector evaluateError(
      const TransformationType& pose_,
      gtsam::OptionalMatrixType H_) const override {
    if constexpr (std::is_same_v<TransformationType, gtsam::Similarity3>) {
      if (H_) {
        const auto error = pose_.between(this->prior());
        gtsam::Matrix local;
        sim3_logmap(error, &local);
        *H_ = local * sim3_adjoint(error.inverse());
      }
      return Base::evaluateError(pose_, nullptr);
    } else {
      return Base::evaluateError(pose_, H_);
    }
  }

  /// @brief Clone without replacing the corrected prior by its native base.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<PosePriorFactor>(*this);
  }
};

/// @brief Check whether retrieving a point-distance factor failed.
/// @param factorOut_ Retrieval output to inspect.
/// @return True when retrieval failed, false on success.
template <typename TransformationType>
[[nodiscard("Use returned bool to branch on factor retrieval failures")]]
inline bool __fail_retrieve(
    const output_t<std::shared_ptr<PointDistanceFactor<TransformationType>>>&
        factorOut_) {
  if (factorOut_ == fail_t::LOST_PTR)
    __throw__("Unexpected: Lost pointer for factor");
  return factorOut_ != fail_t::NO_FAIL;
}

/// @brief Throw if point-distance factor retrieval did not succeed.
/// @param factorOut_ Retrieval output to validate.
template <typename TransformationType>
inline void __sanity_retrieve(
    const output_t<std::shared_ptr<PointDistanceFactor<TransformationType>>>&
        factorOut_) {
  if (factorOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Lost pointer for factor");
}

/// @brief Check whether retrieving a projection factor failed.
/// @param factorOut_ Retrieval output to inspect.
/// @return True when retrieval failed, false on success.
template <typename TransformationType>
[[nodiscard("Use returned bool to branch on factor retrieval failures")]]
inline bool __fail_retrieve(
    const output_t<std::shared_ptr<ProjectionFactor<TransformationType>>>&
        factorOut_) {
  if (factorOut_ == fail_t::LOST_PTR)
    __throw__("Unexpected: Lost pointer for factor");
  return factorOut_ != fail_t::NO_FAIL;
}

/// @brief Throw if projection factor retrieval did not succeed.
/// @param factorOut_ Retrieval output to validate.
template <typename TransformationType>
inline void __sanity_retrieve(
    const output_t<std::shared_ptr<ProjectionFactor<TransformationType>>>&
        factorOut_) {
  if (factorOut_ != fail_t::NO_FAIL)
    __throw__("Unexpected: Lost pointer for factor");
}

}  // namespace covisibility_graph

}  // namespace slam_mer

#endif  // COVISIBILITY_FACTORS_HPP
