<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# Production Sim(3) factor derivatives — 2026-11-08

This report records the initial numerical correction. Production now uses the
subsequent [analytic implementation](sim3_analytic_derivatives.md), with new tests
and measurements. The numerical timings below are historical. Subsequent
[Sim(3) posterior support](sim3_posterior.md) now enables covariance queries.

The derivative correction tested in `sim3_derivative_audit.md` was initially promoted
for backend Sim(3) factors. This is a correction to linearization of the existing
objective, not a new residual model or a claim of improved trajectory accuracy.
The earlier experiments demonstrated better controlled convergence, but only a
small objective improvement on one frozen real graph.

## Implementation

`adjustment/include/sim3_factor_derivatives.hpp` provides two small helpers:

- `sim3_camera_point` evaluates the existing patched inverse point action and
  supplies its analytic Jacobians in the actual right-tangent coordinates:
  `J_pose = [skew(pc), -I, -pc]`, `J_point = R^T/s`.
- `sim3_residual_jacobian` takes central differences of an unchanged 7D residual
  under the actual GTSAM retraction, with step 1e-6 for rotation, local translation
  and log scale. This avoids the stale adjoint and the missing/nonzero-error Local
  derivative without changing the dependency's chart.

`PointDistanceFactor<Similarity3>` and `ProjectionFactor<Similarity3>` use the
analytic helper. `OdometryFactor<Similarity3>` differentiates the native between
residual numerically. This also corrects the existing active rigid-window
restraints and loop odometry edges because they use that same factor class.
`PosePriorFactor<Similarity3>` differentiates the native prior residual instead
of assuming an identity derivative away from its origin. Backend initial priors
and the available fixed-window helper now construct this prior class.

A requested Sim(3) between block requires 14 perturbed residual evaluations
(28 when both blocks are requested), plus the nominal value. A prior block costs
14 plus the nominal value. The numerical part has floating-point/step-size error;
it is not an analytic derivative or damping. It has been checked against scalar
objective finite differences and an independent visual numerical oracle on the
tested moderate-angle, nonunit-scale problems. Extreme scales, very large world
coordinates and chart branch cuts require further testing.

The values of residuals, measurements, factor sigmas, robust kernels, scale
parameterization, retraction, factor ownership and window policy are preserved.
Pose3 follows the pinned GTSAM implementation, including its existing between
and prior approximations. Regression tests explicitly compare Pose3 values and
Jacobians at nonzero residual to the native classes. Public no-Jacobian overloads
remain usable, and cloned corrected odometry/prior factors retain their derived
type and derivative behavior.

The shipped `thirdparty/gtsam_similarity3.patch` and GTSAM sources have not been
changed by this promotion. The remaining matrix/adjoint/Exp/Log/alignment API
inconsistencies remain in that dependency. Active backend factor derivatives
bypass the problematic derivative APIs; this is not a complete repair of the
Similarity3 class. Sim(3) covariance remains explicitly unsupported.

## Tests

The historical baseline is retained in `adjustment/tests/sim3/legacy_factors.hpp`.
It overrides only the old visual/between derivative paths and uses unchanged
residual values. This prevents the audit's "legacy" mode from silently becoming
the corrected implementation after the production header changes.

The fixed-objective test now has five modes: historical, visual-only correction,
the original tested candidate, full numerical oracle, and actual production.
It uses 12 deterministic starts with the same objectives and initial values in
all modes. Production must match the scalar objective gradient and the numerical
oracle's final batch cost. This gives 60 batch comparisons plus 60 iSAM2 runs.
Derivative tests cover nonunit scales, nonzero residuals, optional Jacobian
arguments, world-origin invariance, factory use and clone type preservation.
The Pose3/backend worker tests remain part of the same suite.

A new window-replacement test repeatedly removes and regenerates the complete
set of directed restraints on stationary graphs of 3, 6 and 10 poses. It uses the
backend's existing guarded iSAM2 copy pattern, validates the known stationary
solution to 1e-8, checks that replacing restraints does not accumulate Gaussian
information (relative covariance tolerance 1e-5), and records 75 measured updates per mode/window size after five
warmup updates. Execution order alternates between historical and production.

## Controlled window cost

Measured on the existing production GCC 14/CUDA image, Intel i7-13700H,
`OMP_NUM_THREADS=2`, without clock/affinity control. These small pose-only graphs
isolate regenerated-restraint cost; they do not represent a full SLAM deadline.
Times include factor creation, guarded copy/update and estimate calculation.

| Poses / directed restraints | Version | p50 ms | p95 ms | p99 / maximum ms |
| --- | --- | ---: | ---: | ---: |
| 3 / 6 | Historical | 0.0455 | 0.0762 | 0.0836 |
| 3 / 6 | Corrected | 0.0721 | 0.1144 | 0.2895 |
| 6 / 30 | Historical | 0.1336 | 0.1612 | 0.1785 |
| 6 / 30 | Corrected | 0.1715 | 0.2215 | 0.2647 |
| 10 / 90 | Historical | 0.2917 | 0.3564 | 0.4515 |
| 10 / 90 | Corrected | 0.3874 | 0.5051 | 0.5377 |

The numerical derivatives add measurable cost. At 10 poses the median increase
was about 0.096 ms in this fixture. With 75 samples, nearest-rank p99 is the
maximum. Larger windows and denser full graphs remain unmeasured; an analytic
Local derivative could reduce cost in a separate, independently tested change.

## Fresh production replay

Two historical and two corrected 300-frame MH01 runs used the same settings,
models, GPU, 100 Hz backend target, at-most-20-Hz playback and explicit 100 ms
bootstrap grace from the previous harness. Robust losses and restraints stayed
active. The old implementation was archived from the retained validation
container before installation of the new header, then timed using the same
validation-only backend instrumentation. Each process used two OpenMP threads
and the 64 MiB FAISS temporary-memory setting. All four runs completed, and the
corrected runs reported no estimation/publication failures. Their covariance
status remained `unsupported`, as expected for Sim(3).

These are asynchronous runs with different resulting graphs, not an
identical-graph accuracy or causal speed comparison. Pooled nearest-rank timings:

| Operation | Version | p50 ms | p95 ms | p99 ms | Maximum ms |
| --- | --- | ---: | ---: | ---: | ---: |
| Guarded backend update | Historical | 1.776 | 3.033 | 3.598 | 5.292 |
| Guarded backend update | Corrected | 1.685 | 2.940 | 3.333 | 5.014 |
| Frame decode through map counters | Historical | 24.277 | 80.992 | 147.948 | 2857.700 |
| Frame decode through map counters | Corrected | 24.404 | 71.347 | 148.721 | 1776.620 |

There were 3,818 successful historical and 3,633 successful corrected backend
updates, including unchanged updates and teardown; each version processed 600
frames. Maximum graph sizes differed (683 historical versus 593 corrected
variables). Observed poses/landmarks reached 10/673 versus 9/584. Peak RSS ranged
from 4,377,736–4,418,560 KiB historically and 4,364,792–4,367,652 KiB corrected.
These differences do not isolate memory overhead or establish improved tracking.
Initialization/keyframe inference spikes remain, so this is not a real-time
claim. The controlled window benchmark above is the direct derivative-cost test.

## Reproduction and artifacts

Use the production container/toolchain setup from
`production_posterior_validation.md`, copy the updated adjustment sources, and
build/install the complete production project so every template consumer sees
the same factor definitions. Rebuild the external validation executables too:

```sh
cd /opt/slam-mer
mise exec -- cmake --build build --target install -j 2
mise exec -- cmake --build /opt/slam-posterior-production-tests \
  --target test_backend_posterior test_sim3_derivatives validate_euroc_posterior -j 2
OMP_NUM_THREADS=2 mise exec -- ctest \
  --test-dir /opt/slam-posterior-production-tests --output-on-failure
cd /opt/slam-mer_examples
OMP_NUM_THREADS=2 /opt/slam-posterior-production-tests/test_sim3_derivatives \
  window /validation/results/window.csv
```

For repeated runtime checks, use `run_production_comparison.py --label corrected`
with the existing EuRoC validation executable and Sim(3) settings. It runs serial
bounded replays, preserves covariance reporting, and writes per-process logs and
CSV data. The `--label` names a run; it does not change derivatives. Source/library
installation determines which version runs. The same validation-only timing
instrumentation used in the earlier report supplies backend.csv; restore the
uninstrumented source and rebuild after measuring.

Fresh raw data, baseline source archives, build/test logs, settings and summaries
are under `/tmp/slam-production-validation/sim3-promotion-20261108` on the host.
The older November 6 `/tmp` artifacts did not survive the host restart; their
historical results remain documented separately. No trajectory accuracy benefit,
long-sequence/loop-closure result or complete Sim(3) library repair is claimed.
No commits or paper changes were made.

## Final validation state

The uninstrumented corrected project was rebuilt and installed in the retained
validation container. All five production/investigation CTests passed, as did
the four focused posterior groups (315 checks) and the existing EuRoC
visualization-queue test. The 60 deterministic batch and 60 iSAM2 runs had no
numerical failures; the actual production initial graph-gradient discrepancy
was at most 1.20e-10 relative in that fixture. Pose3 comparison tests retain the
native values and Jacobians to 1e-13, including nonzero residuals.

Source/build hashes and checks confirm the installed library has no validation
baseline/timing hook. The original dependency patch still passes its reverse
application check and was not modified. The retained container contains the
corrected installed library and is stopped after validation. The base
`slam-mer:datasets` image was not rebuilt or retagged: normal scripts launching
that image require an image rebuild to use the new source. No Git commit was
created.
