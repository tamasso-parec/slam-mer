<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# Online backend Gaussian posterior

Production build, custom-factor/worker tests and bounded MH01 measurements now
appear in [the production validation report](production_posterior_validation.md).
Sim(3) support and subsequent validation are described in
[sim3_posterior.md](sim3_posterior.md). The initial standalone measurements below
are retained as a separate experiment.

## Contract

`CovisibilityGraph<Pose3>::posterior()` and
`CovisibilityGraph<Similarity3>::posterior()` return an owned, read-only
`GaussianPosterior`, or null before the first successful backend update. It
is also available through the public `Slam::get_posterior()` facade. It
contains **all variables in the backend Bayes tree**, not ordinary tracking
frames or singly observed points moved only by map update code. Keys use the
backend convention `Symbol('k', keyframe_id.key())` and
`Symbol('p', point_id.key())`. Query missing IDs explicitly; do not interpret a
missing point as having zero covariance.

The distribution is the optimizer's **maintained linear Gaussian**:

```
p(delta) proportional to exp(-0.5 * || R delta - d ||^2)
mean = R^-1 d
covariance = (R' R)^-1
```

It includes the Gaussian linearizations of measurements, initial priors,
active rigid-window restraints, and the robust weights frozen when each factor
was last linearized. It preserves even symmetric duplicate rigid-window
factors added by the current backend. No factor is removed or reconstructed
for reporting. Restraints are algorithmic constraints, not new independent
sensor measurements. This is neither the exact nonlinear posterior nor a claim
of statistical calibration, independent measurement provenance, or coverage.

Selective relinearization means this Gaussian can differ from a fresh graph
linearization at the current estimate. In particular, robust weights can have
mixed ages. The snapshot stores the complete current anchors; it does not
invent one common nonlinear linearization time. Its revision identifies the
maintained factorization and anchor set together. No separate per-factor
linearization-age ledger is implemented.

`anchors()` is the copied GTSAM linearization point theta. `estimate_delta()`
is the optimizer delta used for `estimates() = theta.retract(delta_est)`.
The latter is the exact values object passed to map update, stored inside the
snapshot. Wildfire back-substitution can truncate the optimizer delta;
`gaussian_mean()` instead solves the full triangular Gaussian, including its
nonzero right-hand side. These quantities need not coincide. Capture verifies
the estimate/retraction identity and refuses mismatched values.

## Coordinates and queries

`joint_covariance({keys...})` returns the joint marginal, in **caller order**,
with dimensions, Gaussian mean, covariance, epoch, revision, coordinates and
status. One-key queries give marginals. A two-key query contains either kind
of cross block, including landmark-landmark blocks. The underlying factor
retains all correlations, regardless of which blocks have been requested.
Duplicate keys and missing keys return unavailable with a diagnostic message.
An empty selection on an available posterior returns an empty joint Gaussian.
Every result owns its numeric data.

Default coordinates are additive perturbations to delta in the tangent spaces
at `anchors()`. Backend Pose3 values are camera-to-world Twc. GTSAM orders the
six Pose3 coordinates as rotation, then translation; its configured retraction
is authoritative (full Expmap in the validation build). World landmarks use
Euclidean XYZ. Sim(3) coordinates have seven entries: rotation, local
translation, then natural log scale. The patched action is `s R p + t`. Its
actual pinned chart, rather than a standard Sim(3) exponential, is authoritative.
A pose covariance in these coordinates is not the covariance of
its world translation vector, nor a left-perturbed Tcw covariance.

`joint_covariance_at_estimate()` returns a first-order affine transport to
GTSAM local coordinates at the snapshot estimates. For each pose it uses the
analytic chain of GTSAM Retract and Local Jacobians for Pose3, or the
validated analytic derivative of the actual patched chart for Sim(3):

```
f(e) = Local(estimate, Retract(anchor, delta_est + e))
J = Df(0)
mean_out = J (mean_delta - delta_est)
P_out = J P_delta J'
```

Point Jacobians are identity. The block-diagonal transform multiplies the
**entire joint covariance**, including cross blocks. This is a local Gaussian
approximation, not exact nonlinear propagation. Its Jacobian is evaluated at
the optimizer delta, not automatically at the full Gaussian mean. The result
labels these coordinates explicitly. Consumers must check status before using
numeric fields; failed queries have empty numeric output.

Example (error handling abbreviated):

```cpp
auto snapshot = graph->posterior();
if (!snapshot) return;
const gtsam::Key a = gtsam::Symbol('k', firstId.key());
const gtsam::Key b = gtsam::Symbol('k', secondId.key());
auto joint = snapshot->joint_covariance_at_estimate({a, b});
if (joint.status != GaussianPosterior::Status::available) return;
gtsam::Matrix66 ha, hb;
snapshot->estimates().at<gtsam::Pose3>(a).between(
    snapshot->estimates().at<gtsam::Pose3>(b), ha, hb);
gtsam::Matrix jacobian(6, 12);
jacobian << ha, hb;
gtsam::Matrix relativeCovariance =
    jacobian * joint.covariance * jacobian.transpose();
```

Use the same pattern for pose inversion or an uncertain camera-frame point:
obtain compatible Jacobians and propagate the full joint result. For Sim(3),
use the corrected helpers described in `sim3_posterior.md`; the remaining native
Similarity3 Jacobian APIs are not generally compatible with the repository patch.
Dropping cross blocks generally gives the wrong answer. Conditioning on a
pose is a different operation from expressing a point in an uncertain frame.
No downstream covariance feedback, calibration or tracking changes are added.

## Publication and freshness

The optimizer thread captures after a successful guarded update and estimate
calculation, outside `_mutex` (graph ingestion lock). It then updates the map
and atomically replaces the publication slot. This common completion path is
`PosteriorPublication::publish_after_update`. Covariance extraction failures
publish an unavailable posterior for the new revision; they do not undo a valid
estimate or prevent the map callback from running. Failed optimizer updates do
not call this path and leave the previous publication available. An exception
from map commit leaves the previous publication unchanged and propagates, as
in the pre-existing map-update path; it is not treated as covariance failure.
Resource exhaustion that prevents allocating even the snapshot is not recoverable
by this interface.

A backend/provider instance has a process-local unique `epoch()`. Each accepted
update increments its `revision()`, including unchanged-problem updates. A new
provider (including a rebuild with a new graph object) has a different epoch
and initially no publication. Cross-epoch or non-increasing attachments are
rejected. Standalone `capture` defaults to epoch zero for tests/tools; pass the
provider epoch to publish it. Epochs are not persistent IDs across processes.
The current backend has no in-place reset API. A future in-place reset must
also create a new publication epoch, rather than reuse this slot/revision.

Graph revision and factorization revision share the accepted-update identity:
there is exactly one captured factorization for that accepted state. Old
snapshots remain valid during the next update, after failures, after key additions
or replacement of the provider, and after the backend is destroyed. A query
result repeats both identities. Never combine its covariance with a separately
read newer estimate. Use `snapshot->estimates()` from the same owner.

The mutable map is **not transactional**: its writes are individual, and ingestion,
fusion or pending-point handling can also change it. No global map revision is
added, and this API does not claim that arbitrary map reads share the snapshot
revision. Publication follows map commit, but the snapshot's owned backend
estimates are the coherent interface. Merged point IDs are literal historic
backend keys within a snapshot. Resolve current aliases outside the snapshot,
then query explicitly; resolving an alias does not rewrite an old posterior.
The existing manager's removal of merged variables is not redesigned here.

`captured_at()` is a steady-clock timestamp at completion of numeric capture;
`capture_seconds()` measures extraction. Consumers can measure age from that
timestamp, including map commit and scheduling delay. It is not a sensor timestamp
or a guarantee that the snapshot matches the most recent ingested frame.

## Factorization, concurrency and costs

This implementation targets pinned GTSAM commit
`663bdb6ee02ec28dc88da84cb3d22a362aa1512a` (4.3a0). It does not assume the newer
`ISAM2::jointMarginalCovariance` API. Locally inspected sources:

- `nonlinear/ISAM2.{h,cpp}`: maintained Bayes tree, anchors, lazy delta, estimates.
- `inference/BayesTree-inst.h`, `BayesTreeCliqueBase.h`: copy/ownership and caches.
- `linear/GaussianConditional.{h,cpp}`, `JacobianFactor.h`: numeric blocks/noise.
- `nonlinear/Values.cpp`: copying clones Values, rather than aliasing their entries.
- `base/Lie.h`, `geometry/Pose3.cpp`, `geometry/Similarity3.cpp`: charts/Jacobians.

Capture traverses the existing tree iteratively, children before parents, and
copies frontal/separator coefficients and RHS into an owned sparse upper
triangular R. Conditional noise sigmas are whitened explicitly. It performs no
nonlinear factor reconstruction or graph elimination. It solves once for the
full Gaussian mean. The temporary shared conditional handles are only used on
the optimizer thread and do not escape capture. The existing guarded iSAM2 copy
clones clique structure but shares conditional pointers; it also has mutable
separator caches. A copied optimizer is therefore **not used as reader isolation**.
Readers only use owned sparse coefficients and cloned values.

For a selection matrix E, queries solve `R' Y = E` and return `Y'Y`. This gives
marginals over all remaining variables, not inverse diagonal Hessian blocks.
No factorization, dense full inverse or graph traversal is performed per query.
The solve currently traverses the entire sparse factor; it is not a minimal
Steiner-tree recovery. This trades possible query speed for a small, auditable
adapter to the existing pin. A faster selected-ancestor algorithm is a separate
optimization requiring its own tests and measurements.

If N is scalar graph dimension, F is the sparse factor nonzero count, and q is
selected scalar dimension, capture uses O(F + N) numeric storage/work plus
key-index lookups, sparse assembly and Values copies. Query work is approximately
O(F q + N q^2), with O(N q) temporary solve storage and O(q^2) output. The temporary
triplet representation and cloned/retracted values increase capture peak memory.
`factor_bytes()` counts sparse numeric/index arrays, **not** complete snapshot
memory (Values, maps, caches, allocator overhead and publication peaks excluded).

Each query is limited to 256 scalars and a 64 MiB dense solve buffer. Larger
requests explicitly return unavailable; callers can query all block pairs in
smaller selections. These limits bound allocation, not wall time. Up to 32
anchor-coordinate results are cached within a snapshot, with an 8 MiB charged
numeric/key budget; map/string allocation overhead is additionally bounded by
the entry/key limits. Eviction clears the cache. Estimate-coordinate transforms
reuse these base results. Revisions never share caches. Concurrent cold queries
may duplicate solves, but cache mutation is mutex-protected and solves only read
immutable data. Per-query workspace limits do not bound aggregate memory across
arbitrarily many reader threads.

Publication uses a short mutex only to exchange shared ownership. Queries do
not take map, ingestion or optimizer locks. During capture the old publication
and new snapshot coexist. Consumers retaining older snapshots extend that
memory lifetime; the provider retains only its latest snapshot. Applications
must bound their own history. The existing guarded-update copy and optimization
still run under the ingestion lock; this change only moves/reporting work outside
that lock and does not redesign ingestion scheduling.

Gauge/rank failures from optimization remain explicit optimizer failures.
Capture rejects zero/nonfinite triangular diagonals, invalid conditional
sigmas, constrained conditionals, mismatched key sets and nonfinite means.
Queries reject nonfinite results. No damping, pseudo-inverse, covariance floor
or arbitrary gauge prior is introduced. These checks are not a condition-number
certificate: weak finite geometry may produce very large finite covariances,
and numerical near-singularity remains subject to GTSAM's factorization checks.
Sparse index overflow is an explicit extraction failure.

## Similarity3 support and boundaries

The [analytic factor correction](sim3_analytic_derivatives.md) and the
[validated chart transport](sim3_posterior.md) now allow complete seven-dimensional
Sim(3) Gaussian queries, including every pose/scale/landmark cross block. Value
types are checked explicitly, rather than inferred from dimension or selected
by a boolean mode flag. Pose3, Similarity3 and Point3 are supported; other variable
types publish `unsupported` with matching values and a diagnostic. Invalid
numeric extraction publishes `unavailable`. A successful Sim(3) update now uses
the same available/empty/failure rules as Pose3.

The original Similarity3 patch remains unchanged. Its remaining matrix, alignment
and derivative API inconsistencies are outside the query contract. The snapshot
uses owned Similarity3 rotation/physical translation/scale values, the actual
retraction and maintained numeric factorization; it does not use those unrelated
APIs to manufacture a physical covariance. The public map still stores physical
rotation/translation separately from backend scale. Query the snapshot's full
values when interpreting its covariance. This is not a general repair or
endorsement of every operation in the patched Similarity3 class.

## Reproducible validation

Build the *current patched checkout* into an isolated directory:

```sh
cmake -S thirdparty/gtsam -B /tmp/slam-posterior-gtsam \
  -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
cmake --build /tmp/slam-posterior-gtsam -j 3
cmake -S adjustment/tests -B /tmp/slam-posterior-tests \
  -DGTSAM_DIR=/tmp/slam-posterior-gtsam -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/slam-posterior-tests -j 2
ctest --test-dir /tmp/slam-posterior-tests --output-on-failure
/tmp/slam-posterior-tests/benchmark_gaussian_posterior
/tmp/slam-posterior-tests/benchmark_gaussian_posterior --baseline-only
/tmp/slam-posterior-tests/benchmark_gaussian_posterior --report-only
```

The standalone targets need only pinned GTSAM, Eigen, Boost and Threads, and
can use C++17. Production still requires the repository's C++23 toolchain and
other dependencies. If ccache is configured, it needs a writable cache directory.
The test executable has four independently runnable groups (`linear`, `pose`,
`edge`, `publication`). Tests use explicit tolerances and fixed random seed 9182.
They cover dense independent linear mean/covariance, every cross-block type,
ordering/symmetry, native marginals, separate maintained/fresh Gaussian references,
robust projection factors, relinearization, factor additions/removal, unchanged
updates, finite-difference analytic coordinate/relative/inverse/frame Jacobians,
Monte Carlo retraction propagation, missing/duplicate IDs, singular and weak
geometry, cache eviction/bounds, owned lifetime, failure publication, epoch/rebuild
and concurrent readers. The completion-path test runs the helper invoked by the
actual backend, using a small map-commit callback. It does **not** run the full
CovisibilityGraph worker or real map/fusion code.

The bounded benchmark replays 80 deterministic updates ending with 80 poses,
120 landmarks and 9,600 visual observations, with a first pose prior, odometry
and weak landmark priors. It includes the existing whole-iSAM2 guarded-update
copy in both timing paths. Paired mode checks every update's estimate agreement
to 1e-12, with extraction only in one path. It retains an old snapshot through
the next update. It uses standard GTSAM projection factors, not the repository's
custom factors or rigid-window scheduler. Baseline runs first within each pair,
so cache/ordering effects and system scheduling affect timing comparisons.

## Validation results and remaining work

The initial host-only validation could not compile production: Rerun was absent
and only GCC 11 was installed. The subsequent production follow-up used the
existing Docker image with GCC 14, Rerun and GPU dependencies, successfully built
the full libraries and example, and exercised custom factors, the actual backend
worker/map, and bounded asynchronous MH01 replays. See
[production_posterior_validation.md](production_posterior_validation.md) for exact
commands, timings, the bootstrap issue, and remaining production coverage.
No full-sequence, loop-closure, statistical calibration or real-time claim is made.

Measurements on 2026-11-06, Intel Core i7-13700H (20 logical CPUs), GCC 11.4,
Release/O3, system Eigen, GTSAM TBB disabled. Single bounded replay, 80 samples
per timing row; no CPU affinity or frequency control. The empirical p99 equals
the maximum at this sample count. These numbers are descriptive, not deadlines.

| Paired replay operation | p50 ms | p95 ms | p99 / maximum ms |
| --- | ---: | ---: | ---: |
| Baseline copy + update + estimate | 10.921 | 24.580 | 28.015 |
| Reporting copy + update + estimate + capture | 11.941 | 26.214 | 30.679 |
| Capture alone | 1.841 | 2.919 | 3.447 |
| Cold joint: one pose, two landmarks | 0.787 | 1.274 | 1.543 |
| Cached joint | 0.00184 | 0.00314 | 0.00451 |
| Capture age at first query completion | 0.808 | 1.312 | 1.575 |

The age row measures only this synchronous harness; it excludes real map commit,
frame ingestion and worker scheduling. Capture excludes covariance queries.
Cached times include returning an owned result copy. Maximum factor-array storage
was 2,909,116 bytes per snapshot. Paired replay peak process RSS was 260,392 KiB,
including both optimizers and their guarded copies. Separate processes measured
baseline peak RSS 171,604 KiB and reporting peak RSS 178,932 KiB (difference
7,328 KiB); these include allocator high-water marks and are not a precise
snapshot heap allocation measurement. In the separate reporting-only run, cached
query p50/p95/maximum were 0.00197/0.00574/0.0581 ms, illustrating timing
variability even for cached copies. The reporting run held the previous
snapshot while creating the next one. User-retained history costs additional
memory beyond these measurements.

All four Release CTest groups passed, with 313 checks in the combined executable.
The paired replay passed fixed-graph estimate equality at every update. The
full production configure command below was attempted and stopped at the missing
`rerun_sdkConfig.cmake` dependency; production compilation was not reached:

```sh
cmake -S . -B /tmp/slam-posterior-production -DWITH_CUDA=OFF \
  -DWITH_ALIKED=OFF -DVISUAL_PLACE_LOCALIZATION=OFF -DDEPTH_INFERENCE=OFF
```

Additional sanitizer validation (instrumented test/adapter, linked to the same
unsanitized Release GTSAM) used:

```sh
cmake -S adjustment/tests -B /tmp/slam-posterior-asan \
  -DGTSAM_DIR=/tmp/slam-posterior-gtsam -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS=-O1 -g1 -fsanitize=address,undefined -fno-omit-frame-pointer -DEIGEN_MALLOC_ALREADY_ALIGNED=1' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
cmake --build /tmp/slam-posterior-asan --target test_gaussian_posterior -j 2
ctest --test-dir /tmp/slam-posterior-asan --output-on-failure
```

`EIGEN_MALLOC_ALREADY_ALIGNED=1` matches this Release library's Eigen allocation
ABI. Without it, instrumented translation units selected Eigen's fallback
aligned allocator while the library used `free`, causing boundary bad-free
reports before this code could be assessed. This setting must match the library
build, not be blindly copied to a different GTSAM configuration. LeakSanitizer
also required execution outside the ptrace-based sandbox; leak checking was
not disabled. This checks the new code and tests, not all internal GTSAM memory
accesses or a ThreadSanitizer-instrumented production run.

The final rebuilt sanitizer executable passed all four groups with the same
313 checks. Release and sanitizer checks do not replace production integration
or workload-specific performance validation.
