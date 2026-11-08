<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# Analytic Sim(3) factor derivatives — 2026-11-08

Subsequent [Sim(3) covariance support](sim3_posterior.md) builds on this correction.
This report records the derivative-validation stage.

Production Sim(3) factors now use analytic derivatives throughout. This replaces
only the numerical odometry/restraint/prior derivatives introduced in the
[initial correction](sim3_derivative_implementation.md). The visual derivatives
were already analytic. Finite differences remain in the independent test oracle.
Residual values, noise models, robust losses, the optimizer's chart and estimator
configuration are preserved. Better trajectory accuracy has not been established.

## Derivation and implementation

The repository's patched action is `P_world = s R P_camera + t`. At the origin
of its actual right chart, a perturbation has `dR = R [dw]`, `dt = s R dv` and
`d(log s) = d sigma`. The physical-translation adjoint is

```text
Ad(R,t,s) = [ R       0      0 ]
            [ [t]R   sR    -t ]
            [ 0      0      1 ].
```

The pinned logarithm returns `L(T) = (w, rho, lambda)`, where `lambda = log(s)`
and `rho = V(w,lambda)^-1 t`. Its kernel is `V = p I + q W + r W^2`, `W=[w]`.
The derivative follows from `V rho = t`:

```text
d rho = V^-1 (dt - dV rho)
J_L = [ D                    0          0                 ]
      [ -V^-1 A D            V^-1 sR   -V^-1 V_lambda rho ]
      [ 0                    0          1                 ],
```

where `A[:,j] = (partial V / partial w_j) rho`, and `D` differentiates the actual
rotation logarithm. With radial derivatives `q_rad = (dq/dtheta)/theta`, etc.,

```text
dV = q [dw] + r ([dw]W + W[dw])
     + (w.dot(dw)) (q_rad W + r_rad W^2) + V_lambda d lambda.
```

`adjustment/src/sim3_factor_derivatives.cpp` explicitly differentiates the pinned
GTSAM `Similarity3.cpp` coefficients and branch conditions, including the small
log-scale polynomial and the exact derivatives of the small-angle polynomial
coefficients. Stable convergent analytic series evaluate the scale derivatives
of `p`, `beta` and `mu` for `abs(lambda) < 0.1`; these are scalar coefficient
evaluations, with no perturbed states or residual evaluations. Eighteen terms
are used. The dependency's residual values are still returned directly.

The matrix-backed rotation Logmap has a separate near-pi approximation whose
reported GTSAM Jacobian is not its exact derivative. The helper differentiates
that branch's returned value too, including its largest-diagonal selection.
It does not change Pose3 or the dependency. The quaternion-backed compile path
keeps the native rotation Logmap derivative; that alternate build configuration
was inspected but has not been built or tested in this validation.

For a between residual, `U = first^-1 second`, `E = measured^-1 U`,

```text
H_first = -J_L(E) Ad(U^-1),    H_second = J_L(E).
```

The native prior residual is `-L(pose^-1 prior)`, not `L(prior^-1 pose)`.
Its derivative is `J_L(E) Ad(E^-1)` for `E = pose^-1 prior`. The prior's native
value evaluator remains in use to preserve its mean semantics. No zero-residual
approximation, numerical differentiation, damping or regularization is used in
these production factor derivatives. A between factor evaluates one logarithm
and a fixed amount of small-matrix work instead of 28 perturbed residuals when
both Jacobian blocks are requested.

## Validation

All six production CTests passed. The new `sim3_analytic` test makes 2,052 checks:
identity and pure-scale limits, nonzero translations, both small-scale branches,
both sides of the SO(3) small-angle threshold, all three near-pi diagonal
branches and a mixed axis, positive/negative rotations, physical-adjoint
conjugation, native residual equality, optional Jacobians, both between blocks,
prior derivatives and simultaneous two-pose directions. Random trials use seed
3872026. Independent five-point differences use steps 1e-5 or 1e-6, with
relative tolerances 1e-8 for the adjoint and 5e-7–3e-6 for Logmap/factor checks.
The worst observed relative differences were 1.06e-7 for Logmap and 3.06e-9
for factors. Native residual comparisons require exact equality.

The existing scalar-objective gradient, optimizer, clone/factory and Pose3
regressions passed as well (production initial-gradient discrepancy at most
1.20e-10 relative in that fixture): 60 deterministic batch and 60 iSAM2 runs, with the
production batch optimum checked against the numerical oracle to 1e-7. The
actual map/backend publication test passed. The window test validates stationary
estimates to 1e-8 and checks removal/replacement does not accumulate information
(relative marginal-change tolerance 1e-5); this internal diagnostic does not
enable the Sim(3) posterior query API.

One uninstrumented 300-frame MH01 production replay completed, observing 1,714
snapshot revisions and zero observer/query failures. Sim(3) covariance remained
`unsupported` throughout. Peak process RSS was 4,401,536 KiB. This bounded check
is not a trajectory accuracy comparison or a long-sequence validation.

## Measured update cost

Five serial benchmark processes ran identical stationary graphs with historical,
analytic and numerical derivatives interleaved in rotating order. Each produced
75 measured updates per mode/window size after five warmups: 375 samples per
row below. Times include factor creation, the backend's guarded iSAM2 copy,
factor removal/replacement and estimate calculation. Covariance verification is
outside the timed interval. Values are nearest-rank percentiles, in milliseconds.

| Poses / directed restraints | Derivatives | p50 | p95 | p99 | Maximum |
| --- | --- | ---: | ---: | ---: | ---: |
| 3 / 6 | Historical | 0.0468 | 0.0649 | 0.0816 | 0.1139 |
| 3 / 6 | Analytic | 0.0482 | 0.0693 | 0.1064 | 0.1182 |
| 3 / 6 | Numerical | 0.0719 | 0.0893 | 0.1169 | 0.1380 |
| 6 / 30 | Historical | 0.1328 | 0.1596 | 0.1764 | 0.2040 |
| 6 / 30 | Analytic | 0.1358 | 0.1558 | 0.1688 | 0.6723 |
| 6 / 30 | Numerical | 0.1725 | 0.2064 | 0.2348 | 0.2457 |
| 10 / 90 | Historical | 0.2954 | 0.4117 | 0.4919 | 0.5993 |
| 10 / 90 | Analytic | 0.3097 | 0.4346 | 0.4852 | 0.5205 |
| 10 / 90 | Numerical | 0.4215 | 0.5303 | 0.5772 | 0.6369 |

At 90 restraints the analytic median was about 26.5% below the numerical version,
and 4.8% (0.0143 ms) above the historical, inconsistent derivatives. This is a
controlled fixture comparison; it does not establish the cost on a full moving
SLAM graph or prove a real-time deadline. The 0.6723 ms analytic outlier at six
poses is retained. Measurements used the previous GCC 14 production image,
Intel i7-13700H and two OpenMP threads, without clock/affinity control.

## Reproduction and remaining limits

Use the production container setup from `production_posterior_validation.md`.
Copy the changed adjustment CMake file, headers, source (including the new
`sim3_factor_derivatives.cpp`) and tests into its disposable source checkout.
Rebuild/install the full project before rebuilding clients:

```sh
cd /opt/slam-mer
mise exec -- cmake --build build --target install -j 2
mise exec -- cmake --build /opt/slam-posterior-production-tests \
  --target test_backend_posterior test_sim3_derivatives test_sim3_analytic \
  validate_euroc_posterior -j 2
OMP_NUM_THREADS=2 mise exec -- ctest \
  --test-dir /opt/slam-posterior-production-tests --output-on-failure
cd /opt/slam-mer_examples
OMP_NUM_THREADS=2 /opt/slam-posterior-production-tests/test_sim3_derivatives \
  window /validation/results/window.csv
```

Window CSV modes are 0 historical, 1 production analytic, 2 numerical oracle.
The existing `run_production_comparison.py --label corrected` runs the installed
analytic version; the label itself does not select derivatives. Raw logs, five
window CSVs, the bounded replay, manifests and summaries are under
`/tmp/slam-production-validation/sim3-analytic-20261108` on the host.

Derivatives are branch-local. The pi cut is not differentiable; branch boundaries
and extreme scales/coordinates are not covered by a global smoothness claim.
The pinned kernel's cancellation near its small-scale switch can also limit
finite-difference agreement. The original Similarity3 patch remains intact,
including its remaining matrix/alignment/Exp/Log API inconsistencies. At this stage Sim(3) covariance was still unsupported; see the subsequent
[posterior support](sim3_posterior.md) for its current contract and validation. The validated container has the installed
analytic implementation; the base `slam-mer:datasets` image requires rebuilding
to use the new source. No commit or paper change was made.
