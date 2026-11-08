<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# Sim(3) derivative audit and isolated repair experiments

**Status update, 2026-11-08:** the tested factor-boundary correction is now used
by production Sim(3) factors. See [implementation and fresh validation](sim3_derivative_implementation.md).
The text below records the November 6 investigation, when candidates were still
test-only. Current tests preserve the old derivatives in an explicit historical
fixture and include a fifth mode exercising the actual production implementation.
The subsequent [Sim(3) posterior support](sim3_posterior.md) enables covariance
queries; the unavailable status discussed below is historical.

Date: 2026-11-06. This investigation preserves the repository's applied
`thirdparty/gtsam_similarity3.patch`, existing estimator defaults, Pose3 behavior
and Sim(3) covariance-unavailable status. Experimental derivative replacements
live under `adjustment/tests/sim3`; they are not selected by the production
backend. The goal is to isolate optimization derivative errors without changing
residual definitions, noise models, robust losses, measurements or retraction.

## Actual path and attribution

The patched group acts as `P = s R p + t`, with physical translation t. The
backend's projection and distance factors compute the inverse action:

```
pc = R' (P - t) / s
r_distance = pc - measured_camera_point
r_projection = uncalibrate(pc.xy / pc.z) - measured_pixel
```

For the actual right retraction, the first derivatives of this inverse action
at zero perturbation are

```
J_pose  = [ skew(pc), -I, -pc ]
J_point = R' / s
```

The current distance factor instead chains `inverse(H)` and `transformFrom(H)`.
The patched forward action uses `R p` for its log-scale column rather than
`s R p`. The inherited inverse Jacobian uses `-AdjointMap(S)`, but that adjoint
still contains `s [t]x R` and `-s t`; the new physical-translation convention
requires `[t]x R` and `-t` in those locations. Combining both errors gives:

```
current distance rotation block = skew(pc) + (1/s - 1) skew(R' t)
current distance translation block = -I
current distance log-scale column = -s pc
```

The rotation error also breaks **world-origin invariance**: adding the same
translation c to every world point and pose translation leaves pc, its residual
and the correct right-tangent Jacobian unchanged. The current rotation Jacobian
changes by `(1/s - 1) skew(R' c)`. A focused test verifies both that erroneous
change and invariance of the repair using c = (20,-5,2). Consequently a small
scale discrepancy can have a larger derivative effect farther from the chosen
world origin. This is a mathematical risk, not a measured long-sequence result.

Thus replacing only the forward-action scale column would leave other errors.
Perspective projection cancels the erroneous scale column because its
Jacobian annihilates pc. It does not generally cancel the rotation error. This
is why an error in `transformFrom` must be traced through the complete factor
rather than assumed to affect every factor component identically.

Odometry and the active rigid-window restraints use `OdometryFactor<Similarity3>`,
which inherits the pinned GTSAM `BetweenFactor`. Its `between` derivative uses
the stale adjoint. Separately, the default `BetweenFactor` omits the final Local
Jacobian at nonzero residual. That omission is an upstream approximation, not
introduced by this repository's patch. The native prior similarly supplies an
identity Jacobian for its local-coordinate residual; this is also an upstream
approximation. The experiments separate these approximations from the patch's
convention errors. Enabling `GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR` alone is not a
repair: this pin's Similarity3 `Logmap` derivative is unimplemented and throws
when requested.

## Group consistency versus a usable optimization chart

The current `Expmap`/`Logmap` remain inverse chart functions. Their naming does
not establish that they are the exponential/logarithm of the newly patched
group. A smooth local retraction need not be the true group exponential for an
optimizer to work correctly. The derivative candidates therefore differentiate
**the implemented residual and retraction**, preserving the objective, instead
of changing the chart and attributing all differences to the scale column.

Other incomplete convention migrations remain: `matrix()` and its constructor
retain the earlier homogeneous representation, and `Align` still divides its
estimated physical translation by scale. These are additional consistency
problems, but source inspection found no active backend call to GTSAM's
`Similarity3::Align`; new backend values are constructed from scale-one Pose3
matrices. They are not used to explain away the measured active-factor errors.
A complete library repair still needs a coordinated audit of matrix conversion,
Exp/Log, adjoints, alignment, factors and map export.

## Candidates and independent checks

Four graph variants share identical nonlinear residuals and noise models:

| Mode | Visual derivatives | Between/restraint and pose-prior derivatives |
| --- | --- | --- |
| 0: Current | Existing implementation | Existing implementation |
| 1: Visual only | Analytic inverse-action Jacobians above | Existing implementation |
| 2: Repaired candidate | Analytic inverse-action Jacobians | Central differences of actual residuals under actual retraction |
| 3: Numerical oracle | Central differences of visual residuals | Same numerical derivatives as mode 2 |

Numerical derivatives use step 1e-6. The analytic visual candidate is checked
against that independent oracle and against the closed-form identities above.
A separate finite-difference derivative of the **scalar graph objective** checks
`-b` from Gaussian linearization, including robust weights. It is not merely a
comparison between two versions of the same factor Jacobian implementation.

The direct trace covers scales 0.4, 1 and 2.5, zero/nonzero translations and
nontrivial rotations. The second pose in the between-factor trace has scale
1.6, so even the row whose first pose has scale 1 can have a nonunit relative
scale. At scale 0.4 with nonzero translation, the current projection rotation
Jacobian differs from finite differences by Frobenius norm 127.3; the repaired
one differs by about 4.5e-8. The distance pose-Jacobian error falls to about
7.1e-10. The point-coordinate and distance local-translation derivatives already
agree; those are not claimed to be broken.

## Deterministic optimization experiment

The fixture has three poses with truth scales 0.7, 1.4 and 2.1, eight landmarks,
a tight first-pose prior, weak point priors, odometry and visual factors. Seed
8462026 generates six initializations of a noiseless Gaussian problem and six
of a problem with deterministic small distance discrepancies and Huber weights.
Initial conditions are shared across all four modes. Batch LM uses the same
100-iteration cap and convergence tolerances. Candidate and reference objective
values at the initial state must agree; modes 2 and 3 must agree on the final
objective within 1e-7. These tests do not retune any factor sigmas between modes.

The fixture also compares iSAM2 with the production threshold 0.01, relinearize
skip 1, default Gauss-Newton/wildfire behavior and 40 unchanged-problem updates
after initial insertion. Failures are recorded explicitly. Batch convergence
results are not presented as iSAM2 results or as real-sequence accuracy.

Observed batch results (six starts per noise setting; median and maximum):

| Problem / mode | Initial gradient relative error, median | LM iterations, median / max | LM solve ms, median / max |
| --- | ---: | ---: | ---: |
| Noiseless / current | 0.1209 | 23.5 / 25 | 3.406 / 3.711 |
| Noiseless / visual only | 0.000234 | 5.5 / 6 | 0.857 / 0.953 |
| Noiseless / repaired | 1.03e-10 | 5 / 6 | 0.886 / 0.965 |
| Noisy Huber / current | 0.1652 | 20 / 29 | 5.028 / 5.800 |
| Noisy Huber / visual only | 0.000244 | 5 / 6 | 0.834 / 1.004 |
| Noisy Huber / repaired | 8.68e-11 | 5 / 6 | 0.879 / 1.080 |

The numerical-oracle gradients and final objectives agree with the repaired
candidate. In the noisy batch problem, median final objective changes only from
0.276525454 to 0.276519798. The strong result here is derivative correctness and
convergence speed, not a large change in the optimum. The noiseless current
solver already reaches very small pose error. Timing uses this tiny fixture,
fixed mode order and uncontrolled clocks; cache warmup can affect comparisons.
The scalar finite-difference gradient diagnostics are excluded from solve timings.

With iSAM2, all 48 cases completed without numerical failure. After initial
insertion and 40 unchanged-problem updates:

| Problem / mode | Final objective median | Final objective maximum |
| --- | ---: | ---: |
| Noiseless / current | 0.638030 | 1.287547 |
| Noiseless / visual only | 4.95e-7 | 1.05e-5 |
| Noiseless / repaired | 2.04e-6 | 4.94e-5 |
| Noisy Huber / current | 0.855472 | 1.152793 |
| Noisy Huber / visual only | 0.276524 | 0.276527 |
| Noisy Huber / repaired | 0.276522 | 0.276572 |

The fully numerical oracle reproduces the repaired candidate's iSAM2 outcomes.
Selective relinearization/wildfire thresholds prevent a guarantee of stationary
nonlinear optima; numerical gradient norms remain nonzero. The visual-only
variant can finish closer than the full repair in an individual bounded iSAM2
run because relinearization decisions differ. A mathematically correct Jacobian
does not guarantee uniformly lower cost after a fixed number of thresholded
incremental updates. Most of these updates reuse cached linearization, so this
is not a runtime model for continuously regenerated window restraints.

## Frozen real backend graph

A validation-only hook copied one accepted graph and its estimate during a
100-frame MH01 Sim(3) run. All active nonlinear factors, including duplicate
window restraints and their robust noise models, were copied into each variant.
The copy had **180 variables, 556 scalars and 907 factors**. The four copied
problems were optimized after the backend worker stopped; no result was fed back
into the estimator or map. This avoids attributing asynchronously different
trajectories to a derivative-only change.

At exactly the same current estimate and objective 955.361980924:

| Mode | Relative graph-gradient error | Objective after bounded batch trial | Final finite-difference gradient norm |
| --- | ---: | ---: | ---: |
| Current | 0.005589 | 897.963950318 | 34.35 |
| Visual only | 0.001667 | 897.958343386 | 14.16 |
| Repaired | 1.29e-8 | 897.958016510 | 11.47 |
| Numerical oracle | 1.29e-8 | 897.958016510 | 11.47 |

All variants hit the 80-iteration cap. These are **not converged optimality
claims**. Corrected analytic visual derivatives and the numerical oracle agree
on the final objective, and the corrected initial gradient matches the scalar
objective derivative. However, the repaired-versus-current final objective
improvement is only about 0.00066%. Most of the reduction from 955 to 898 also
occurs with the existing derivatives when given extra batch iterations. It
would be misleading to attribute that whole reduction to the repair.

This real graph confirms an active derivative defect and a small optimization
benefit under the bounded trial. It does not establish an appreciable improvement
in real trajectory accuracy, tracking robustness or online convergence rate.
The experiment changes Jacobians only inside copied diagnostic graphs; the live
run still uses the original estimator.

## Interpretation and implementation boundary

There is a practical path to a better optimizer: correct the visual inverse-action
Jacobians first, then make the between/prior derivatives consistent with the
chosen chart. The analytic visual repair is small and does not require changing
the shipped GTSAM patch or measurement/noise definitions. Numerical between/prior
derivatives are useful validation oracles, not yet an online performance promise.

The isolated tests demonstrate improved gradient correctness and, in the
synthetic problems, substantially faster convergence. They do not justify claiming
that changing the default estimator will improve EuRoC accuracy. A production
change should first add matching analytic Local derivatives or measure the
numerical alternative, validate mixed/nonunit-scale cases and factor lifecycle,
and compare identical recorded graph updates plus paired longer trajectories.
Lower cost in the chosen measurement model is not automatically better ground
truth accuracy.

The underlying Similarity3 library consistency issues remain separate work.
These experiments do not enable Sim(3) covariance, modify the dependency patch,
or change Pose3 behavior. Existing production defaults remain available as the
baseline. No paper or calibration constants were changed, and no commits were
created.

## Reproduction

Use the existing production container described in
`production_posterior_validation.md`. Copy `adjustment/tests` from the workspace
into its `/opt/slam-mer/adjustment/tests` directory and build:

```sh
cd /opt/slam-mer
mise exec -- cmake --build /opt/slam-posterior-production-tests \
  --target test_sim3_derivatives -j 2
OMP_NUM_THREADS=2 mise exec -- ctest \
  --test-dir /opt/slam-posterior-production-tests --output-on-failure
```

The added CTests are `sim3_derivative_trace` and `sim3_fixed_objective`.
The executable can also write raw CSVs directly:

```sh
cd /opt/slam-mer_examples
OMP_NUM_THREADS=2 /opt/slam-posterior-production-tests/test_sim3_derivatives \
  trace /validation/results/sim3-factor-trace.csv
OMP_NUM_THREADS=2 /opt/slam-posterior-production-tests/test_sim3_derivatives \
  optimize /validation/results/sim3-optimization.csv
```

To repeat the frozen real-graph experiment, use the staging script **only on the
disposable source copy**, review its generated diff, rebuild, and run the same
bounded sequence with `SLAM_MER_SIM3_TRIAL` set:

```sh
cd /opt/slam-mer
/usr/bin/python3 /validation/source/adjustment/tests/sim3/stage_frozen_trial.py \
  --reference /validation/source/adjustment/src/covisibility_graph.cpp \
  --target /opt/slam-mer/adjustment/src/covisibility_graph.cpp
mise exec -- cmake --build build --target install -j 2
cd /opt/slam-mer_examples
OMP_NUM_THREADS=2 SLAM_MER_FAISS_TEMP_MEMORY_MB=64 \
  SLAM_MER_SIM3_TRIAL=/validation/results/sim3-frozen-graph.csv \
  /opt/slam-posterior-production-tests/validate_euroc_posterior \
  /datasets/EuroC/MH_01_easy /validation/results/similarity3.yaml 100 \
  /validation/results/sim3-frozen-run
```

`similarity3.yaml` is the tracked production test `pose3.yaml` with only
`Map.Covisibility.UseSimilarity` set to true. The hook captures once the graph
has at least 150 variables; actual contents vary with asynchronous scheduling.
Candidates within that run receive the identical captured graph and state.
Capture allocates copies and can itself affect scheduling, so that run is not
an unbiased online timing experiment. Restore the original backend source and
rebuild after the experiment. Raw data and logs are retained under
`/tmp/slam-production-validation` on the host.

## Final state

The production backend source was restored and rebuilt after the frozen-graph
hook experiment. Existing backend-factor/worker tests and both new Sim(3) tests
pass. The applied GTSAM patch is unchanged. Only diagnostic candidates, tests,
staging tooling and this report were added for this investigation. Raw CSVs are
`sim3-factor-trace.csv`, `sim3-optimization.csv`, `sim3-frozen-graph.csv`, with
summary `sim3-summary.json` under the host results directory. The private
container is stopped and retained after validation.
