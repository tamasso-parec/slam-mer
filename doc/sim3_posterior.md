<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# Online Sim(3) Gaussian posterior — 2026-11-08

The backend now publishes Sim(3) covariance through the same immutable posterior
API as Pose3. A successful numeric capture retains all seven pose coordinates
and all landmark variables, including pose–pose, pose–landmark, landmark–landmark
and log-scale cross terms. No complete dense covariance is constructed per
update. Query bounds, sparse solves, bounded revision-local caches, ownership,
freshness and failure behavior follow [online_posterior.md](online_posterior.md).

This is the **maintained linear Gaussian**, including priors, active restraints
and robust weights at their last linearization. It approximates the current
optimizer objective locally. It is not the exact nonlinear posterior and makes
no statistical calibration or trajectory accuracy claim. Mixed-age selective
linearizations can differ from a fresh nonlinear linearization at the estimate.

## Coordinates and transport

The patched transformation is `S(P) = s R P + t` with physical translation `t`.
Anchor queries return additive coordinates in the optimizer chart, ordered
`(rotation[3], local translation[3], natural log scale[1])` for each pose and
Euclidean XYZ for each point. They return the full Gaussian optimum `R_factor^-1 d`,
which can differ from the optimizer's wildfire-truncated estimate delta.

For an anchor `A`, the pinned right retraction is `A * E(delta)`, where
`E(w,v,lambda) = (Exp_SO3(w), V(w,lambda) v, exp(lambda))`. The inherited kernel
`V` is not the standard physical-translation Sim(3) exponential kernel. Covariance
transport differentiates that actual function; it does not substitute a textbook
Sim(3) exponential Jacobian or the dependency's inconsistent adjoint.

The estimate query computes

```text
f(e) = Local(estimate, anchor.retract(delta_est + e))
J = Df(0)
mean_est = J (mean_gaussian - delta_est)
P_est = J P_anchor J^T.
```

Left composition by the anchor cancels. Write `Q = Exp_SO3(w)`, `s = exp(lambda)`,
`B[:,j] = (partial V / partial w_j) v`. Since the Local derivative at identity is
identity in this chart, its transport Jacobian is

```text
J = [ J_SO3_right(w)         0               0                     ]
    [ s^-1 Q^T B            s^-1 Q^T V      s^-1 Q^T V_lambda v   ]
    [ 0                     0               1                     ].
```

`sim3_retract_jacobian` evaluates this analytically using the same pinned kernel
coefficients and derivatives as the corrected factors. It also differentiates
the actual small-angle Rodrigues polynomial in the matrix rotation build.
No numerical derivatives or regularization are used in production transport.
The block-diagonal transport multiplies the entire queried joint covariance,
preserving every cross term. Finite differences and Monte Carlo are test oracles.
This remains first-order affine propagation, evaluated at the estimate delta,
not exact nonlinear propagation or automatic recentering at the Gaussian mean.

## Queries and physical quantities

```cpp
auto snapshot = slam->get_posterior();
if (!snapshot) return;
auto joint = snapshot->joint_covariance_at_estimate({poseKey, pointKey});
if (joint.status != GaussianPosterior::Status::available) return;
// dimensions == {7,3}; the covariance is 10x10 in exactly that key order.
const auto& pose = snapshot->estimates().at<gtsam::Similarity3>(poseKey);
const auto& point = snapshot->estimates().at<gtsam::Point3>(pointKey);
gtsam::Matrix hp, hx;
sim3_camera_point(pose, point, &hp, &hx);
gtsam::Matrix h(3,10);
h << hp, hx;
gtsam::Matrix cameraPointCovariance = h * joint.covariance * h.transpose();
```

The helpers are in `sim3_factor_derivatives.hpp`, namespace
`slam_mer::covisibility_graph`. Use the full joint result, including pose–point
cross covariance. A local translation covariance is not a world translation
covariance: at the estimate its physical Jacobian is `[0, sR, 0]`. The scale
Jacobian is `[0,...,0,s]`, so first-order scale variance is `s^2 P_logscale`.
The same joint transformation preserves translation–scale correlations.

For local inverse coordinates the Jacobian is `-sim3_adjoint(S)`. For the relative
transform `U = S1^-1 S2`, the two blocks are `[-sim3_adjoint(U.inverse()), I]`.
These require estimate-local covariance and estimates from the same snapshot.
Native patched Similarity3 point, adjoint and other Jacobian APIs must not be
assumed compatible; the supported helper boundaries are explicitly tested.

The map's pose type stores rotation/physical translation, with scale maintained
separately inside the backend. The snapshot preserves the full Similarity3 value.
It is the coherent source for a covariance consumer. Do not pair a current map
pose or separately obtained estimate with an old snapshot covariance.

## Publication and remaining boundaries

The old Pose3-only capture flag has been removed. Capture checks actual GTSAM
value types and records their coordinate types. Pose3, Similarity3 and Point3
are accepted; other types return `unsupported`. All backend variables are
retained. Missing keys, singular systems, invalid scales and failed queries
remain explicit failures. No arbitrary gauge anchor, damping or covariance floor
is added by reporting. Weak but finite directions retain their large covariance.

Capture still copies existing numeric conditionals, matching anchors, optimizer
delta and estimates, solves once for the Gaussian mean, then publishes after
map commit. It performs no new nonlinear linearization or elimination. Readers
retain update k while update k+1 runs, including after removal of an old point ID.
No mutable optimizer object or shared conditional escapes capture. Retaining
history extends memory use; the provider retains only its newest snapshot.

The shipped GTSAM patch is preserved. Its remaining matrix/alignment/general
Exp/Log API inconsistencies are not repaired or endorsed by this feature.
The supported Gaussian coordinates and physical helper transformations bypass
those inconsistent derivative APIs. Large coordinate noise, extreme scales,
branch switches and the rotation logarithm cut are outside a global smoothness
or accuracy claim. The tested production build uses matrix-backed rotations;
quaternion-backed GTSAM has not been built or validated here. Pose3 behavior is
covered by the existing regressions.

## Tests and validation

All 12 production CTests passed, including five focused Sim(3) posterior groups
and the real Sim(3) map/backend worker. The new posterior executable performs
249 checks, in addition to the existing 2,052 analytic derivative checks and
Pose3/optimizer regressions. All four standalone posterior groups also passed (319 checks).

- `linear`: two nonunit-scale similarities and two landmarks, with an independent
  dense 20-dimensional A,b reference, nonzero RHS/means, every cross-block type,
  caller ordering, symmetry, marginalization versus conditioning, iSAM2 marginal
  agreement, repeated queries, missing/duplicate keys and estimate-coordinate
  transport. Relative covariance tolerances are 1e-11 for the dense reference
  and 1e-10 for block/native checks.
- `geometry`: analytic Retract transport checked at scales 0.4, 1 and 2.5,
  nonzero translation/log-scale deltas, small-angle and large-rotation deltas
  (finite-difference step 1e-6; tolerance 2e-7). Independent inverse, relative
  transform, world translation and uncertain camera-point Jacobians preserve
  cross terms. A 20,000-sample joint Monte Carlo test uses seed 73812026 and a
  5% relative covariance tolerance at small noise. The dense fixture uses seed
  91482026.
- `mean`: a deliberately large wildfire threshold leaves some estimate deltas
  truncated. The full Gaussian optimum is checked against the maintained
  conditionals; its nonzero affine mean after transport must be preserved.
- `incremental`: actual corrected projection/distance/odometry/prior factors,
  robust weights, relinearization, additions, factor replacement, removal and
  replacement of a point ID, revision-local cache changes, concurrent queries,
  old snapshot lifetime and failed updates. Dense maintained-conditionals
  references and native marginals are checked to 1e-8. Fresh nonlinear
  linearizations are identified separately; the comparison at the estimate uses
  estimate-local coordinates on both sides. Independent optimizers on the same
  fixed graph agree to 1e-12 with and without reporting.
- `edge`: seven-dimensional gauge failure, weak anchored directions (1e12
  covariance), unsupported seven-dimensional Euclidean values, and zero/negative
  similarity scales. Dimension alone must not identify a pose type.

The shared backend worker test now runs in both pose modes. It exercises real
map commits, repeated publication, added point IDs, active restraint release and
replacement, concurrent readers, epoch/revision matching and snapshot survival
after backend teardown. Its controlled stationary poses have unit scale; the
separate mathematical tests and live replay cover nonunit scale.

A 100-frame MH01 smoke run observed 669 available snapshots with no failures.
It preceded two 300-frame runs in each of three modes: baseline (capture and
publication skipped only in a disposable instrumented build), capture without
queries, and capture with joint queries. The second repeat reversed mode order.
All six processes completed. Both capture-only runs observed 3,419 available
snapshots in total; both query runs observed 3,439, all available, with zero
observer/query failures. The final query-run poses had scales in approximately
[0.985, 1.227]. Those are final-state bounds, not a per-revision scale audit.

The live reader checks symmetry, positive semidefiniteness, repeated-query
identity, estimate transport, coherent revision identities and retained snapshot
lifetime. It samples publications and can skip revisions. The runs use the real
frontend, inference, map and backend, but are bounded asynchronous experiments.
They do not establish full-sequence/loop-closure behavior or trajectory accuracy.

## Measured costs

Environment: retained Ubuntu 24/GCC 14 production container, Intel i7-13700H,
RTX 4070 Laptop GPU, two OpenMP threads; no CPU affinity or clock control.
The live runs used 64 MiB FAISS temporary memory, 100 Hz backend target,
at-most-20-Hz playback, 100 ms first-frame bootstrap grace and a two-second
end grace. Robust losses and active restraints remained enabled.

An identical-graph benchmark adds 40 nonunit-scale poses and 40 landmarks over
40 updates, with real custom visual factors, priors, odometry and up to 90
regenerated directed restraints. Five serial paired processes give 200 samples.
Timing order alternates, and every update checks estimate equality to 1e-12.
The old snapshot is retained during the next capture. The graph grows during
measurement; first-update allocation and initialization are included.
Nearest-rank distributions, in milliseconds:

| Controlled operation | p50 | p95 | p99 | Maximum |
| --- | ---: | ---: | ---: | ---: |
| Baseline guarded copy/update/estimate | 2.513 | 4.603 | 5.117 | 5.238 |
| Reporting copy/update/estimate + capture | 2.805 | 5.233 | 5.706 | 6.605 |
| Capture alone | 0.341 | 0.580 | 0.695 | 0.829 |
| Cold joint query | 0.319 | 0.461 | 0.490 | 0.694 |
| Cached joint query | 0.00099 | 0.00131 | 0.00222 | 0.00615 |
| Estimate transport using cached joint | 0.0070 | 0.0102 | 0.0112 | 0.0127 |
| Capture age after query/lifetime checks | 0.447 | 0.634 | 0.674 | 0.971 |

Reporting total is the sum of its measured update and capture components;
query time is additional. Cold queries select two poses/two points (20 scalars),
except the first update, which has only one pose (13 scalars). Maximum sparse
factor-array storage was 549,464 bytes per snapshot. Separate single-process
baseline/report runs had peak RSS 617,960/622,936 KiB (difference 4,976 KiB).
These high-water marks include libraries, optimizer storage, allocator behavior,
values, caches and overlapping snapshots; they are not precise heap accounting.
The provider and consumers must still bound retained history.

Pooled live query-run distributions (3,739 successful backend timing rows;
3,439 reader observations):

| Live operation | p50 | p95 | p99 | Maximum |
| --- | ---: | ---: | ---: | ---: |
| Capture | 0.489 | 0.969 | 1.032 | 1.566 |
| Cold joint query | 0.323 | 0.570 | 0.665 | 4.212 |
| Cached joint query | 0.00087 | 0.00173 | 0.00265 | 0.01863 |
| Estimate transport using cached joint | 0.0082 | 0.0139 | 0.0176 | 0.1550 |
| Capture age after reader queries | 3.958 | 6.469 | 7.191 | 8.540 |

Maximum live factor-array storage was 535,712 bytes; the query runs reached 710
backend variables. Baseline/capture-only graphs reached 695/682 variables, so
live mode timing or memory differences do not isolate reporting overhead.
Baseline peak RSS ranged 4,386,916–4,388,864 KiB; capture-only
4,353,672–4,423,312 KiB; query runs 4,384,396–4,405,252 KiB. The asynchronous graphs
and inference initialization differ between runs. Frame-time maxima remained
about 1.75–1.81 seconds. No real-time performance claim is made.

The retained library was rebuilt without timing instrumentation after measuring.
The base `slam-mer:datasets` image was not rebuilt or retagged. Normal image
launches require rebuilding it to use these source changes. No commit or paper
change was made.

## Reproducible commands and artifacts

Use the container/toolchain setup in `production_posterior_validation.md`. Copy
updated adjustment headers, sources, CMake files and tests into `/opt/slam-mer`.
Rebuild/install the complete project and clients; the new transport helper is
part of the `covisibility_graph` library.

```sh
cd /opt/slam-mer
mise exec -- cmake --build build --target install -j 2
mise exec -- cmake --build /opt/slam-posterior-production-tests \
  --target test_backend_posterior test_sim3_derivatives test_sim3_analytic \
  test_sim3_posterior benchmark_sim3_posterior validate_euroc_posterior -j 2
OMP_NUM_THREADS=2 mise exec -- ctest \
  --test-dir /opt/slam-posterior-production-tests --output-on-failure
mise exec -- cmake --build /opt/slam-posterior-standalone -j 2
OMP_NUM_THREADS=2 mise exec -- ctest \
  --test-dir /opt/slam-posterior-standalone --output-on-failure
cd /opt/slam-mer_examples
OMP_NUM_THREADS=2 /opt/slam-posterior-production-tests/test_sim3_posterior
OMP_NUM_THREADS=2 /opt/slam-posterior-production-tests/benchmark_sim3_posterior \
  paired /validation/results/paired.csv
```

The benchmark also accepts `baseline` and `report` for separate RSS measurements.
For live costs, instrument only a disposable backend source copy using the
existing `instrument_backend.py`, build/install it, then run:

```sh
/usr/bin/python3 /opt/slam-mer/adjustment/tests/production/run_replays.py \
  --executable /opt/slam-posterior-production-tests/validate_euroc_posterior \
  --sequence /datasets/EuroC/MH_01_easy \
  --settings /opt/slam-mer/adjustment/tests/production/similarity3.yaml \
  --output /validation/results/sim3-posterior-20261108/replays \
  --frames 300 --repeats 2
/usr/bin/python3 /opt/slam-mer/adjustment/tests/sim3/summarize_posterior_validation.py \
  /validation/results/sim3-posterior-20261108
```

Restore the uninstrumented backend and rebuild/install after measuring. Raw
logs, five paired CSVs, separate RSS runs, smoke/replay results, summary JSON,
source/build manifests and library hash are retained on the host under
`/tmp/slam-production-validation/sim3-posterior-20261108`.
