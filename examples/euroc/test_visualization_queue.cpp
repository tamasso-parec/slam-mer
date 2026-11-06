/// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
/// SPDX-License-Identifier: AGPL-3.0-or-later
/// @brief Check that deleted-point notifications are consumed, not retried.

#include <interface.hpp>
#include <stdexcept>

namespace slam_mer::interface {
extern set_t<point_id_t>::shared_ptr __newPoints;
extern set_t<point_id_t>::shared_ptr __updatePoints;
extern set_t<point_id_t>::shared_ptr __hidePoint;
extern std::function<output_t<point3d_t>(point_id_t)> ___get_point;
extern std::function<neighbor_t::vector(point_id_t)> ___get_neighbors;

class QueueInterface : public AbstractInterface {
public:
  /// @brief Synchronize queues without starting visualization threads.
  void synchronize() { log_points(); }

private:
  void __log_messages() override {}
  void __log_images() override {}
  void __log_dense_point_cloud() override {}
  void __log_points() override {}
  void __log_cells() override {}
  void __log_camera_location() override {}
  void __log_histogram_counts() override {}
  void __log_keyframe_criteria() override {}
  void __log_covisibility() override {}
};
}  // namespace slam_mer::interface

/// @brief Check missing, duplicate, and deleted visualization notifications.
int main() {
  using namespace slam_mer;
  using namespace slam_mer::interface;
  __newPoints = set_t<point_id_t>::make_shared();
  __updatePoints = set_t<point_id_t>::make_shared();
  __hidePoint = set_t<point_id_t>::make_shared();
  int queries = 0;
  bool exists = false;
  ___get_point = [&](point_id_t) -> output_t<point3d_t> {
    ++queries;
    if (!exists) return fail_t::NOT_FOUND;
    return point3d_t(1, 2, 3, 100, 100, 100);
  };
  ___get_neighbors = [](point_id_t) {
    return neighbor_t::vector{};
  };
  QueueInterface interface;
  const point_id_t id(12345);

  __newPoints->insert_(id);
  interface.synchronize();
  interface.synchronize();
  if (!__newPoints->empty() || queries != 1)
    throw std::runtime_error("Deleted-before-insert point was retried");

  exists = true;
  __newPoints->insert_(id);
  interface.synchronize();
  __newPoints->insert_(id);
  interface.synchronize();
  if (!__newPoints->empty() || queries != 2)
    throw std::runtime_error(
        "Duplicate visualization insertion was not consumed");

  exists = false;
  __updatePoints->insert_(id);
  interface.synchronize();
  interface.synchronize();
  if (!__updatePoints->empty() || !__hidePoint->empty() || queries != 3)
    throw std::runtime_error("Deleted-before-update point was retried");
}
