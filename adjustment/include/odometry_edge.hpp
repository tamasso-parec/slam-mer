/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
///
/// @file odometry_edge.hpp
/// @brief Odometry-edge representation between keyframes in the graph.
/// @details Declares `OdometryEdge`, a lightweight container for keyframe IDs,
/// relative pose constraints, and loop-closure flags used by pose-graph
/// optimization and graph maintenance routines.
/// @author Pedro Miraldo
/// @author Valter Piedade

#ifndef ODOMETRY_EDGE_HPP
#define ODOMETRY_EDGE_HPP

// SLAM-MER includes
#include <types/id.hpp>
#include <types/pose.hpp>
#include <types/queue.hpp>
#include <types/vector.hpp>

namespace slam_mer {
namespace covisibility_graph {

/// @brief Represents a relative-pose constraint between two keyframes.
/// @details Stores both keyframe IDs, the relative pose from keyframe A to
/// keyframe B, and whether the constraint was created by loop closure.
class OdometryEdge {

public:
  /// First keyframe Id
  keyframe_id_t _keyframeA;
  /// Second keyframe Id
  keyframe_id_t _keyframeB;
  /// Relative pose that transforms keyframe A to B
  pose_t _odometry;
  /// Flag indicating if this edge is a loop closure
  bool _loopClosure = false;

  /// Typedef for a std::shared_ptr of OdometryEdge
  using shared_ptr = std::shared_ptr<OdometryEdge>;
  /// Typedef for a vector_t of OdometryEdge
  using vector = vector_t<OdometryEdge>;
  /// Typedef for a queue_t of OdometryEdge
  using queue = queue_t<OdometryEdge>;

  /// @brief Construct an OdometryEdge object.
  /// @param keyframeA_ First keyframe ID of the constraint.
  /// @param keyframeB_ Second keyframe ID of the constraint.
  /// @param odometry_ Relative pose that transforms keyframe A into keyframe B.
  /// @param loopClosure_ True when the edge was created from loop closure.
  OdometryEdge(
      const keyframe_id_t& keyframeA_,
      const keyframe_id_t& keyframeB_,
      const pose_t& odometry_,
      const bool loopClosure_ = false)
      : _keyframeA(keyframeA_),
        _keyframeB(keyframeB_),
        _odometry(odometry_),
        _loopClosure(loopClosure_) {}

  /// @brief Construct an OdometryEdge object from a shared pointer.
  /// @param other_ Shared pointer to the source OdometryEdge.
  OdometryEdge(const OdometryEdge::shared_ptr& other_)
      : _keyframeA(other_->_keyframeA),
        _keyframeB(other_->_keyframeB),
        _odometry(other_->_odometry),
        _loopClosure(other_->_loopClosure) {}

  /// @brief Construct an OdometryEdge object by copy.
  /// @param other_ Source OdometryEdge.
  OdometryEdge(const OdometryEdge& other_)
      : _keyframeA(other_._keyframeA),
        _keyframeB(other_._keyframeB),
        _odometry(other_._odometry),
        _loopClosure(other_._loopClosure) {}

  /// @brief Construct a default OdometryEdge object.
  OdometryEdge()
      : _keyframeA(keyframe_id_t::UNSET),
        _keyframeB(keyframe_id_t::UNSET),
        _odometry(pose_t::Identity()),
        _loopClosure(false) {}

  /// @brief Destroy an OdometryEdge object.
  ~OdometryEdge() {}

  /// @brief Assignment operator for OdometryEdge.
  /// @param other_ Source OdometryEdge.
  /// @return Reference to this edge after assignment.
  OdometryEdge& operator=(const OdometryEdge& other_) {
    if (this != &other_) {
      _keyframeA = other_._keyframeA;
      _keyframeB = other_._keyframeB;
      _odometry = other_._odometry;
      _loopClosure = other_._loopClosure;
    }
    return *this;
  }
};

}  // namespace covisibility_graph
}  // namespace slam_mer

#endif  // ODOMETRY_EDGE_HPP
