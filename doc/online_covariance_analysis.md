<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# Online covariance: unseen_v_slam to SLAM-MER

Analysis date: 2026-11-06. This is a source inspection and implementation plan,
not an implementation or measured performance result. SLAM-MER baseline:
`b65f743`, GTSAM `663bdb6ee02ec28dc88da84cb3d22a362aa1512a` with the existing
Similarity3 patch. The sibling repository has substantial working-tree changes;
the findings below describe its current files rather than only its last commit.
Neither estimator was changed or run for this analysis.

## Recommendation

Yes: synchronous online covariance is feasible here. Reuse iSAM2's elimination
and query machinery rather than porting the custom g2o Schur/Woodbury engine.
Transfer the publication, coordinate, validity, history-ownership and validation
contracts. Start with SE(3), report-only covariance after each successful backend
update, and explicitly distinguish this from covariance for every tracked frame.

The numerical query is the relatively easy part. The central questions are
which factors define the uncertainty, which coordinate/linearization it belongs
to, how it reaches concurrent consumers, and whether an online budget can be met.
GTSAM does not automatically make a misspecified measurement model calibrated.

## 1. What exists in unseen_v_slam

### Synchronous publication, not a shutdown-only Hessian inverse

`CMakeLists.txt` enables covariance extraction and synchronous extraction by
default. `LocalMapping.cc` invokes
`LocalBundleAdjustmentWithCovarianceExtraction`. At the end of a completed
visual LBA, `Optimizer.cc` commits map estimates, invalidates the old posterior,
and refreshes or directly publishes a posterior under the map's update lock.
Snapshots contain values, not live KeyFrame/MapPoint pointers.

The ordinary `PosteriorMap.cpp` refresh reconstructs the local visual model at
the committed estimates: local/covisible variable cameras, external observing
cameras treated as exact, accepted positive-depth observations and nominal
measurement whitening. Its acceptance tests mirror mono/stereo/body gates and
association removal. It does not simply invert the last damped LM system or
retain final Huber weights. Thus it is an accepted-observation conditional
Gaussian, not automatically the posterior of every factor used by the optimizer.

Experimental historical-prior modes instead capture the actual accepted graph
and publish its pose information, including historical priors and released
cameras. The ordinary synchronous boundary-prior recipe still publishes the
conditional visual model; separate augmented diagnostics include the actual
boundary priors. These quantities must not be conflated.

### Numerical machinery

`IncrementalPosterior.{hpp,cpp}` groups whitened Jacobians by landmark. For
pose/landmark information blocks A, B and C it forms

    S = A - B C^-1 B^T
    P_pose = S^-1
    P_landmark = C^-1 + C^-1 B^T P_pose B C^-1
    P_pose,landmark = -P_pose B C^-1

Rank-aware QR handles deficient landmark geometry; such a landmark can still
contribute pose information without receiving a finite point marginal.
For at most 3000 pose scalars (500 SE(3) poses), the implementation materializes
the dense pose inverse using Cholesky solves. Larger systems use sparse LDLT
and selected right-hand-side solves. Small dirty-landmark replacements can use
Woodbury updates; structural changes, large replacement sets, missing update
factors, periodic rebuilds and numerical failures trigger refactorization.
Forced LBA rebuilds use the Schur fast path, so the name IncrementalPosterior
does not imply that every publication is a cheap inverse update.

It supports pose marginals, pose cross blocks, landmark marginals,
pose-landmark and landmark-landmark blocks, and landmark covariance conditioned
on a reference pose. This is more substantial than separate 6x6/3x3 inverses.
The conditional point covariance subtracts
P_point,ref P_ref,ref^-1 P_ref,point. That is conditioning on a pose; it is
different from propagating uncertainty into coordinates attached to an
uncertain reference frame.

### Serving and downstream consumers

`PosteriorMap.cpp`, `CovarianceSnapshot.hpp` and `PoseCovarianceHistory.hpp`
provide revision/generation identities, invalidation, map/history epochs,
published snapshots and last-variable-pose history. Queries distinguish current
and pending state. Calibration attaches only to the matching revision. Tracking
can read the last publication without rebuilding it, with explicit age and
conditioning/fallback modes. This is synchronous with backend publication,
while tracking and local mapping still execute asynchronously.

`RelativePoseCovariance.hpp` declares world-to-camera Tcw, left perturbations,
and rotation-first tangents. Relative covariance uses the joint pose covariance,
including cross terms, with analytical propagation and Monte Carlo tests.
`Optimizer.cc` has opt-in predictive tracking based on published landmark
uncertainty, plus separate fixed-boundary inflation and boundary-pose prior
paths. Independent per-observation weighting is an approximation when map
landmarks share pose uncertainty; it is not full joint predictive inference.

### Historical reuse is a separate problem

There are two materially different mechanisms:

* The synchronous marginal-prior recipe releases selected boundary poses and
  uses their stored marginal covariance with externally chosen scale/floors
  and gates. It drops cross-pose correlations and can reuse old measurements.
* `PoseMarginalPrior.{hpp,cpp}` implements exact, conservative and inflated
  historical-factor modes. These retain a Gaussian factor's H, nonzero RHS b,
  tangent anchors, consumed observation IDs and retired-track identities.
  Previous history plus retiring tracks is marginalized; retained live factors
  are not included again. Complete-track retirement can discard later useful
  observations, so this is a real accuracy/information tradeoff.

`PRIOR_REUSE_VALIDATION.md` describes sequential linear tests against a batch
reference, unchanged-window repetition, ID-replacement suppression and a simple
counterexample showing that finite covariance inflation cannot generally cure
double counting. It also describes nonlinear simulations; implemented tests
and protocols are not proof of universal nonlinear consistency.

### What is research rather than a solved online capability

Null calibration, cluster sandwiches, revision scaling, external calibration,
pose-equivalent nuisances and temporally correlated residual models coexist.
They are distinct experimental modes, with compatibility restrictions.
External scale/floor/drift calibration is not GT-free online learning.
The pose-equivalent construction reproduces a prescribed scaled covariance
plus independent pose floors while preserving a fixed-graph MAP; its published
real-sequence evidence still reports incomplete calibration and scheduling
variability. The October online signal/regularization plans report unsuccessful
promotion gates, not an established adaptive live policy.

The temporal feasibility audit specifically found repeated observations with
changed linearizations in successive captures. It requires chronological
factor ownership and pre-assimilation predictions before claiming online
learning. Later fixed-geometry residual fitting is useful model research, but
is not an end-to-end online covariance implementation.

## 2. SLAM-MER integration points and obstacles

### Existing optimizer

`adjustment/src/covisibility_graph.cpp` owns iSAM2 inside `__optimize`.
`GraphOptimizationManager::run_optimization` copies iSAM2, tries an update,
and replaces the original only on success. The backend then calls
`calculateEstimate()` and `__map_update()`. This gives a natural publication
boundary, but map writes currently occur individually; a coherent state and
covariance publication needs an explicit transaction/revision contract.

The graph contains custom 2D projection and 3D camera-point distance factors,
odometry/loop factors, a tight initial-pose prior, and temporary rigid-window
factors. Projection/distance/odometry factors can use robust losses. Their
noise models currently come from scalar or diagonal settings.

The active path is `__optimize_rigid_window`, not the available exact-fixed
window helper. It adds strong relative-pose factors centered on the current
estimate for connected localization-buffer keyframes and removes previous
clamps on the next update. These are algorithmic restraints, not independent
new measurements. Current code iterates both endpoints of connected pairs;
factor identity and possible symmetric duplication must be audited.
The fullGraphOptimization argument is not consulted in the inspected manager
body, so callers must not assume it gives a clamp-free covariance update.

Therefore the native marginal is uncertainty of the graph including these
restraints. It must be labeled accordingly. A measurement-only covariance
requires a separately defined factor selection and elimination; merely
subtracting marginal matrices or hiding clamp metadata cannot produce it.

Image pixels, inferred/stereo-derived 3D points and image-derived odometry may
share source observations. Treating their factors as independent can produce
overconfidence. Trace their provenance before interpreting statistical coverage.

### Native GTSAM version

The pinned `ISAM2.h/.cpp` implements `marginalCovariance(key)` using the Bayes
tree. It does not have `ISAM2::jointMarginalCovariance`. The inherited two-key
Bayes-tree `joint` API documents limitations; do not assume arbitrary queries
are covered. `Marginals.h` supports joint marginals, including construction
from a Gaussian graph, and is a useful validation/fallback path. Reconstructing
and eliminating a full graph every publication would sacrifice much of the
incremental benefit and should not be the default online design.

Current upstream GTSAM documents newer direct incremental joint queries and
Steiner-tree recovery (PR 2486). Its reported 3–4 ms pair queries concern a 2D
pose-graph example, not a runtime guarantee for this dense 3D visual graph.
Evaluate a pinned upgrade separately from the first implementation. Preserve
the working Docker build and audit the local Similarity3 patch before changing
the dependency. On the present pin, implement selected joint recovery from
the existing Gaussian factorization with a batch reference, or initially limit
reporting to native marginals and declare joint products unavailable.

### Linearization and coordinates

iSAM2 selectively relinearizes factors. Its maintained Gaussian is not
necessarily a fresh Hessian evaluated entirely at `calculateEstimate()`.
Publish its linearization anchors/delta and transport requested covariances
through the actual retraction when needed. Validate against the same cached
Gaussian first, then separately measure discrepancy from a fresh undamped
linearization at the committed estimate. Forcing a full relinearization each
frame changes the computational question and must be benchmarked separately.

SLAM-MER's custom factors use camera-to-world poses (inversion maps world
points into the camera); ordinary localization inverts PoseLib's Tcw output
to Twc. GTSAM's manifold/retraction convention must be the authority for the
new implementation. Derive adapters rather than copying g2o left-Tcw formulas.
For standard right-retracted Twc, inversion gives a left-Tcw perturbation
with opposite sign; joint blocks and output coordinate transforms still need
the full Jacobians. Test inversion, between, frame changes and translation
outputs numerically and by Monte Carlo.

Both Pose3 and Similarity3 are supported. Similarity3 has seven tangent
dimensions, scale correlations, a local convention patch, and map export that
stores pose rotation/translation separately from backend scale. Audit compose,
inverse, Exp/Log, matrix conversion and factor Jacobians at nonunit scales
before enabling its covariance. A 6x6 slice is not automatically covariance
of the published physical pose; use the output map's Jacobian.

### Backend versus every tracked frame

Backend keyframe covariance is not current non-keyframe covariance.
`localization/src/localization.cpp::__pose_estimation` calls the separate
PoseLib path in `utils/src/pose_estimation.cpp`; there is no existing covariance
handoff there. A second online component is needed for per-frame uncertainty.
Single-observation map points can also be moved during `__map_update` without
being variables in iSAM2. They need an explicit unavailable or propagated
initialization covariance, not a fabricated backend marginal.

## 3. Proposed implementation sequence and acceptance gates

### Stage 0 — freeze semantics and baseline

Record settings, dependency revisions/patches, hardware, input hashes and thread
configuration. Inventory actual active factors, observation source IDs, merges,
retirements, rigid restraints, gauge anchors and loop updates. Confirm whether
data-derived factors reuse raw measurements. Capture graph statistics and
backend/frame timing before enabling covariance.

Specify products: pose marginal, selected relative poses, selected landmark
marginals, pose-landmark blocks, and later frame predictive covariance.
Initial mode: `Pose3`, report-only, backend Gaussian including active restraints,
explicit gauge and linearization metadata. No imported TUM calibration constants.

Gate: a small saved graph has known factor ownership, gauge and coordinate
semantics; supported and unavailable products are explicit.

### Stage 1 — synchronous backend publication

Introduce `adjustment/include/covariance_snapshot.hpp`,
`adjustment/include/covariance_provider.hpp` and corresponding implementation.
Use existing naming/doc conventions. The optimizer thread alone queries and
mutates its iSAM2/Bayes-tree caches. Consumers read immutable values.

After a successful update, obtain estimates and a bounded query set, compute
requested covariance, and publish a shared revision with the map state.
Include graph/map epoch, update/frame watermark, timestamps, ordered keys,
pose type/dimension, means/anchors, tangent convention, graph/robust mode,
anchor/restraint identities, availability and query timing. Retain a distinct
linearization revision if factors have mixed relinearization ages.

Do not hold the graph-ingestion mutex during expensive queries. Keep map commit
and publication coherent using a revision/transaction protocol; failed covariance
must not accidentally roll back a valid optimizer update. Publish unavailable
for that revision and preserve old results only with their original revision.
Reject stale asynchronous attachment and clear epochs on reset/rebuild.
Point fusion/reassociation must invalidate affected IDs/caches.

Gate: snapshots match one accepted backend state; no mixed-revision queries,
no live optimizer pointers, no fake zero covariance, and fixed-input solves
are unchanged by report-only extraction. Live trajectories can differ through
scheduling, so validate identical-graph equivalence separately.

### Stage 2 — joint products and covariance model audit

Implement and validate a version-specific joint-query adapter. Start with
current/reference pose pairs and pose-point pairs; cache within one revision.
Compare the pinned adapter and, separately, an upstream upgrade against dense
small-system solves. Never invert only a variable's diagonal Hessian block:
that gives conditional information, not its marginal.

For a relative function with Jacobians Ji and Jj, retain

    P_relative = Ji Pii Ji^T + Jj Pjj Jj^T
               + Ji Pij Jj^T + Jj Pij^T Ji^T.

For a reference-frame point, propagate the full pose-point joint covariance.
Expose point-given-reference conditioning as a separate named quantity.

Keep two diagnostic models when useful: maintained backend Gaussian (including
restraints and frozen robust weights), and accepted nominal measurement graph
at a specified state. The latter must have an explicit gauge and factor policy;
without restraints it may be singular, and at a restrained estimate it need not
be at its own optimum. Initial fresh/batch computations are validation tools,
not production per-frame work.

Gate: pose, point, cross, relative and conditional products match independent
linear references; relinearization, robust-weight, restraint and gauge effects
are reported instead of silently mixed.

### Stage 3 — per-frame online covariance and consumers

After PoseLib tracking, first publish a pose-only local covariance with the
matched map treated as fixed, clearly labeled conditional. Next incorporate
the relevant map covariance through selected joint blocks. For a prediction
before assimilation, residual covariance follows from the joint uncertain
state/map Jacobian plus measurement noise; it is not generally independent
R + J_point P_point J_point^T per feature. Cross-landmark terms can be large.

A small temporary tracking Gaussian connected to a joint map prior can produce
a current-frame marginal without inserting the same observations twice into
the persistent backend. Its factor ownership and correlation with map history
must be explicit. Use reference-relative coordinates to control gauge effects.
Budgeted low-rank/shared-pose approximations are possible but need comparison
to the small dense joint reference.

Begin with reporting/visualization. Only then test uncertainty-aware refinement,
gating or keyframe selection. Prediction tests must score a datum before it is
used in the prediction. In-sample residual cancellation is not calibration.
Keep the current nominal tracking behavior as the fallback when covariance is
missing, stale, singular or over budget, with counters for each reason.

Gate: actual frame-level results are causal and revision-consistent, unavailable
cases remain visible, and runtime plus accuracy changes are measured separately.

### Stage 4 — bounded memory and principled history

If the retained iSAM2 graph meets the target duration/memory budget, keep it:
it already retains historical factors. Do not add its marginal back as another
prior alongside those factors. If long-duration operation requires fixed lag,
choose retained separators and retire factors exactly once, preserving joint
Gaussian information, RHS and anchors rather than independent unary marginals.

GTSAM's `marginalizeLeaves` requires leaf keys and freezes linearization points
involved in its linear marginal. Fixed-lag smoothing therefore needs ordering,
separator/fill-in control, landmark lifecycle and late-loop-closure policies.
An old pose absent from the active smoother cannot accept an ordinary loop
factor without an explicit archive/global-backend design. Compare retained
full-graph and bounded-lag architectures before choosing one.

The sibling's consumed-observation ledger and epoch/ticket rules are useful
templates; porting complete-track retirement literally would unnecessarily
discard observations if the new design can retain pose-landmark separators.

Gate: sequential linear mean AND covariance match a unique-factor batch
reference; repeating a window without new information does not add information;
fusion, reset, re-entry and retirement cannot resurrect consumed data.

### Stage 5 — online noise modeling, after the foundation

Keep this a separate research stage. Start with nominal measurement noise and
evaluate predictive innovations on unseen arrivals. Any adaptive model needs
identifiable parameters, bounded causal sufficient statistics, correct elapsed
time and an explicit motion/model assumption. A shared frame latent or temporal
state can represent correlation within a GTSAM graph, but does not by itself
identify an amplitude or distinguish sensor bias from free pose error.

Use externally frozen calibration only as an explicitly labeled comparison.
Do not transplant scale 158, pose floors or gates from ORB-SLAM: the front end,
depth model, graph and factor noise differ. Gaussian/robust factor covariance,
sampling uncertainty, bias and empirical coverage remain separate claims.

Gate: synthetic parameter recovery and causal held-out predictive scoring,
followed by sequence-held-out real-data evaluation, before adaptive weights
affect the estimator. No claim of universally calibrated online uncertainty.

## 4. Validation and performance protocol

* Numerical: tiny anchored Pose3 graphs with points and nonzero residuals;
  independent dense solves for all requested blocks; singular/weak geometry;
  changes in robust weights and factor removal; joint propagation Monte Carlo.
* Geometry: finite-difference custom factor Jacobians; nonunit-scale Sim(3)
  compose/inverse/Exp/Log/matrix tests before enabling seven-dimensional output.
* History/concurrency: no-new-data repetition, delayed publication, failed
  optimization/query, reset, map correction, point fusion and loop closure.
* Statistical: repeated synthetic noise trials with known generating model,
  full joint relative-pose NEES/coverage, bias/RMSE, covariance failures included;
  then real sequences with correct camera/body GT extrinsics and timestamps.
  Alignment gauge and scale must be handled consistently in error/covariance.
* Performance: p50/p95/p99 and maxima for update, map commit, each query family,
  publication age and frame latency; also memory, graph/clique sizes, queue
  backlog, coverage, missed deadlines and loop-closure spikes. Measure cold and
  cached queries. Include the existing whole-iSAM2 copy in the baseline cost.

Choose a target frame/backend rate on the intended hardware before setting
numeric budgets. For a rate f the total relevant critical path must fit 1/f;
allocate a smaller explicit covariance share after measuring baseline headroom.
Bound the number and dimensions of requested blocks; do not request covariance
of the entire map each cycle. A query-count cap alone is not a worst-case time
bound. If queries cannot be cancelled mid-solve, deadline control needs measured
admission rules and explicit missed-deadline behavior. A background estimator
with an unbounded queue is not evidence of online operation.

Use matched, reproducible graph replays for numerical and timing comparisons,
then repeated full live sequences for scheduling/accuracy effects. Begin with
an MH01 integration prefix, then a full sequence, and only broaden after the
correctness/runtime gates pass. Freeze settings before confirmation sequences;
GT is used for evaluation, not the runtime covariance provider.

## 5. Source map

Sibling sources under `/home/tom/Documents/PhD/unseen_v_slam/`:

* `include/Covariances/{IncrementalPosterior,CovarianceSnapshot,PosteriorStats,
  PoseCovarianceHistory,RelativePoseCovariance,PoseMarginalPrior,SyncPosePrior}.hpp`
* `src/Covariances/{IncrementalPosterior,PosteriorMap,PoseMarginalPrior,
  PoseEquivalentNuisance}.cpp`, `src/{Optimizer,LocalMapping}.cc`
* `docs/{COVARIANCE_AUDIT,CONSERVATIVE_PRIORS,PRIOR_REUSE_VALIDATION,
  POSE_EQUIVALENT_ONLINE,REAL_TEMPORAL_FEASIBILITY,ONLINE_NUISANCE_PROGRESS,
  ONLINE_COVARIANCE_SIGNAL_PLAN,ONLINE_COVARIANCE_REGULARIZATION_PLAN}.md`

SLAM-MER sources: `adjustment/src/covisibility_graph.cpp`,
`adjustment/include/covisibility_factors.hpp`,
`adjustment/src/factors_noise_model.cpp`, `localization/src/localization.cpp`,
`utils/src/pose_estimation.cpp`, `slam/src/slam.cpp`, and the pinned
`thirdparty/gtsam/gtsam/{nonlinear,geometry,inference}` implementations.

Upstream primary references checked on the analysis date:

* [Incremental fast covariance with iSAM2](https://gtsam.org/2026/04/06/incremental-fast-covariance.html)
* [iSAM2 reference, including leaf marginalization and linearization restrictions](https://gtsam.org/doxygen/4.0.0/a03643.html)

The local headers determine what is available in this checkout; newer upstream
documentation is not evidence that those APIs exist in the pinned dependency.
