// Copyright (c) 2017 Marten Bjorkman
// MIT License
// Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
// SPDX-License-Identifier: AGPL-3.0-or-later

#include <Eigen/Dense>
#include <cmath>

#include "cudaSift.h"

int ImproveHomography(
    SiftData& data,
    float* homography,
    int numLoops,
    float minScore,
    float maxAmbiguity,
    float thresh) {
#ifdef MANAGEDMEM
  SiftPoint* mpts = data.m_data;
#else
  if (data.h_data == NULL) return 0;
  SiftPoint* mpts = data.h_data;
#endif
  float limit = thresh * thresh;
  int numPts = data.numPts;
  using matrix8_t = Eigen::Matrix<double, 8, 8>;
  using vector8_t = Eigen::Matrix<double, 8, 1>;
  matrix8_t M;
  vector8_t A, X, Y;
  for (int i = 0; i < 8; i++)
    A(i) = homography[i] / homography[8];
  for (int loop = 0; loop < numLoops; loop++) {
    M.setZero();
    X.setZero();
    for (int i = 0; i < numPts; i++) {
      SiftPoint& pt = mpts[i];
      if (pt.score < minScore || pt.ambiguity > maxAmbiguity) continue;
      float den = A(6) * pt.xpos + A(7) * pt.ypos + 1.0f;
      float dx = (A(0) * pt.xpos + A(1) * pt.ypos + A(2)) / den - pt.match_xpos;
      float dy = (A(3) * pt.xpos + A(4) * pt.ypos + A(5)) / den - pt.match_ypos;
      float err = dx * dx + dy * dy;
      float wei = (err < limit ? 1.0f : 0.0f);  //limit / (err + limit);
      Y << pt.xpos, pt.ypos, 1.0, 0.0, 0.0, 0.0, -pt.xpos * pt.match_xpos,
          -pt.ypos * pt.match_xpos;
      M.noalias() += (Y * Y.transpose()) * wei;
      X.noalias() += Y * pt.match_xpos * wei;
      Y << 0.0, 0.0, 0.0, pt.xpos, pt.ypos, 1.0, -pt.xpos * pt.match_ypos,
          -pt.ypos * pt.match_ypos;
      M.noalias() += (Y * Y.transpose()) * wei;
      X.noalias() += Y * pt.match_ypos * wei;
    }
    Eigen::LDLT<matrix8_t> solver(M);
    if (solver.info() != Eigen::Success) break;
    A = solver.solve(X);
  }
  int numfit = 0;
  for (int i = 0; i < numPts; i++) {
    SiftPoint& pt = mpts[i];
    float den = A(6) * pt.xpos + A(7) * pt.ypos + 1.0;
    float dx = (A(0) * pt.xpos + A(1) * pt.ypos + A(2)) / den - pt.match_xpos;
    float dy = (A(3) * pt.xpos + A(4) * pt.ypos + A(5)) / den - pt.match_ypos;
    float err = dx * dx + dy * dy;
    if (err < limit) numfit++;
    pt.match_error = sqrt(err);
  }
  for (int i = 0; i < 8; i++)
    homography[i] = A(i);
  homography[8] = 1.0f;
  return numfit;
}
