<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

# Production posterior validation — 2026-11-06

A subsequent [Sim(3) derivative audit](sim3_derivative_audit.md) traces the known
patch defects and compares isolated optimizer repairs. It does not change the
production defaults at the time or enable Sim(3) covariance. The factor
correction was subsequently [promoted on November 8](sim3_derivative_implementation.md);
Sim(3) covariance was unsupported at that stage. The subsequent
[Sim(3) posterior implementation](sim3_posterior.md) now enables it with validated
chart transport and its own production measurements.

This report follows the initial standalone validation in `online_posterior.md`.
It distinguishes production compilation, numerical checks, backend lifecycle
checks, and bounded asynchronous sequence measurements. It does not validate
statistical calibration or establish a real-time guarantee.

## Environment and source control

The existing `slam-mer:datasets` image (`150f7ae984d7`) provided Ubuntu 24.04,
GCC 14.2, CUDA 12.8, the Rerun SDK, model weights and installed dependencies.
The host CPU was an Intel i7-13700H; the GPU was an RTX 4070 Laptop GPU with
8 GiB. A disposable container mounted the checkout and sibling dataset directory
read-only, with results written to `/tmp/slam-production-validation` on the host.
No system toolchain or sibling repository was modified. No commits were made.

A file/hash comparison found that the image's first-party implementation matched
the checkout apart from the new posterior code. The current adjustment sources
were copied into the container and built/installed. The production profile kept
ALIKED/LightGlue, depth inference, visual place localization, CUDA and both
`Pose3`/`Similarity3` instantiations enabled. Pinned GTSAM was
`663bdb6ee02ec28dc88da84cb3d22a362aa1512a`, with TBB and Pose3 Expmap enabled.
The existing Similarity3 patch was preserved. The raw source manifest,
build settings, dataset hashes and logs are in the results directory.

`Slam::get_posterior()` now forwards read-only snapshot ownership from the backend.
It does not return live optimizer state. Callers must include
`gaussian_posterior.hpp` to query the returned object. A concurrent caller must
finish before destroying the owning `Slam` object; a snapshot already acquired
can outlive that object. The EuRoC loader was moved unchanged into
`examples/euroc/dataloader_euroc.hpp` so the normal runner and validation harness
share calibration, image conversion and timestamp handling.

## Checks performed

The full production libraries built and installed, and the normal EuRoC runner
rebuilt after the shared-loader change. The new production-linked tests use the
actual installed `covisibility_graph` library and custom factors, not copied
factor implementations or substituted optimizer/map objects.

`backend_factors` checks the custom projection and camera-point distance
Jacobians at 24 deterministic, nontrivial poses (seed 6429; central difference
step 1e-6, relative tolerance 1e-7). A tiny anchored robust graph checks the
complete joint covariance and nonzero Gaussian mean against an independently
assembled dense reference (1e-8), then checks native iSAM2 marginals after
relinearization. Identical fixed-graph updates with and without report-only
extraction/queries check estimate equality to 1e-12.

`backend_worker` creates the real map and threaded Pose3 backend, inserts an
anchored odometry edge and a landmark with two observations, and waits for
published results. It checks repeated updates, new landmark IDs, map values,
active restraint release/reinstatement, concurrent query/revision identities,
and snapshot lifetime after the actual backend/map are destroyed. The map
comparison uses a zero-residual stationary problem, avoiding an assertion that
arbitrary concurrent map reads are transactional. It does not simulate point
fusion or a failed nonlinear optimizer update.

The headless sequence runner uses the actual `Slam` facade, frontend, inference,
localization, map and backend worker. It does not start visualization workers.
It reads snapshots from a separate thread, selecting the first/latest backend
poses and first/last landmarks in each observed revision (up to 18 scalars).
It checks repeated-query equality, symmetry, positive semidefiniteness,
compatible identities, analytic coordinate transport and readable old snapshots.
It records skipped revisions rather than claiming to query every publication.
The library's real interface owner is linked without starting its threads to
supply the production logging sink.

## Bootstrap issue found by the first smoke test

The first harness run used a 20 Hz backend and failed at frame 2 with an empty
localization buffer, before any backend posterior was published. Initial
inference consumed the playback interval, so there was no wait before the next
frame requested asynchronously ingested descriptors. The ordinary example uses
100 Hz. The final harness uses that frequency and an explicit 100 ms grace
period after the first frame. This is a test-harness allowance, not an ingestion
barrier or a production scheduling fix. The bootstrap grace is excluded from
frame timing columns. Inference teardown is guarded on exceptions so an initial
frontend error does not also trigger a static CUDA teardown failure.

The corrected uninstrumented 100-frame MH01 smoke test observed 661 available
revisions, reaching four backend poses and 232 backend landmarks, with no query
failures. Capture p50/p95/p99/maximum was
0.270/0.294/0.378/0.452 ms; cold selected-joint queries were
0.168/0.262/0.286/0.331 ms. These graph sizes describe backend variables, not
all map points or tracked frames.

## Measurement method

The bounded workload is the first 100 calibrated cam0 frames of
`EuroC/MH_01_easy`, paced to at most 20 Hz without dropping slow frames, followed
by a fixed two-second grace period. That grace is not a drained-backend barrier;
unchanged-problem updates continue. The tracked `pose3.yaml` starts from the
example defaults and monocular settings, selects SE(3), disables loop processing,
and keeps robust factors and active rigid-window restraints. GPU feature/depth
models remain enabled. `OMP_NUM_THREADS=2` and the FAISS temporary memory setting
is 64 MiB. No CPU/GPU clock control or affinity is imposed.

Six serial runs compare two repetitions each of:

- **Baseline:** estimation and map commits, without posterior capture/publication.
- **Reporting:** posterior capture/publication, with metadata observation only.
- **Queries:** reporting plus cold/cached joint and estimate-coordinate queries.

The order is baseline/reporting/queries, then reversed. All use the same binary
and settings. `instrument_backend.py` modifies only the **disposable source copy**
to add timing/logging and an environment-selected baseline branch. Its generated
diff is saved for review. Both branches preserve estimation and map operations;
the baseline skips only the posterior call. The workspace production source has
no diagnostic baseline switch or timing I/O.

Backend CSV rows record successful updates, ingestion-lock wait, guarded update
(including the pre-existing whole-iSAM2 copy), estimate calculation, complete
publication, numeric capture and map commit. Publication includes capture plus
map commit and pointer replacement; it is not an extra term to add to those
components. `factor_slots` counts allocated nonlinear factor slots, including
holes left by restraint removal, rather than active measurement count. Timing
logging occurs after the timed operations, and can still affect scheduling.

Frames record `Slam::add` and decode-through-map-counter wall time, excluding
playback sleep. Cold/cached query timings include owned result copies. Snapshot
age before/after queries includes map commit and reader polling delay relative
to numeric capture completion. Peak process RSS includes inference, maps,
logging queues, allocators and snapshots; it cannot isolate snapshot heap cost.

These are asynchronous replays, not identical fixed graphs. Scheduling changes
can change keyframes, point counts and estimates. Comparisons are descriptive
and must not be read as proof that extraction changes or improves estimation.
The separate fixed-graph numerical test is the estimate-invariance check.

## 100-frame results

All six runs exited successfully. Reporting observed 1,337 available snapshots
across its two runs; query mode observed 1,339. There were no unavailable
posteriors, query failures or revision regressions in these runs. The baseline
intentionally had no publication. Each mode has 200 frame samples. Successful
backend-update samples number 1,650 baseline, 1,652 reporting and 1,653 query mode,
including the final grace period and worker teardown. Query observation stops
before pipeline teardown, so its snapshot sample count is smaller.

The table pools the two runs per mode using nearest-rank percentiles. All values
are milliseconds; maxima are retained rather than treated as outliers.

| Operation | Mode | p50 | p95 | p99 | Maximum |
| --- | --- | ---: | ---: | ---: | ---: |
| Guarded backend update | Baseline | 1.111 | 1.510 | 1.984 | 2.711 |
| Guarded backend update | Reporting | 1.192 | 1.541 | 1.973 | 2.622 |
| Guarded backend update | Queries | 1.062 | 1.435 | 1.729 | 2.472 |
| Map commit | Baseline | 0.568 | 0.691 | 0.828 | 1.793 |
| Full publication (capture + map + swap) | Reporting | 0.958 | 1.098 | 1.375 | 1.553 |
| Numeric capture | Reporting | 0.366 | 0.402 | 0.546 | 0.647 |
| Numeric capture | Queries | 0.299 | 0.387 | 0.466 | 0.558 |
| Cold selected joint | Queries | 0.190 | 0.309 | 0.377 | 0.498 |
| Cached selected joint | Queries | 0.00086 | 0.00157 | 0.00232 | 0.00341 |
| Transport to estimate coordinates | Queries | 0.00785 | 0.01597 | 0.02150 | 0.03296 |
| Snapshot age before query | Queries | 3.202 | 5.445 | 5.659 | 5.816 |
| Snapshot age after query checks | Queries | 3.479 | 5.732 | 5.991 | 6.226 |
| Frame decode through map counters | Baseline | 20.122 | 108.039 | 216.830 | 1845.700 |
| Frame decode through map counters | Reporting | 20.275 | 110.335 | 216.937 | 1796.350 |
| Frame decode through map counters | Queries | 20.251 | 111.776 | 216.900 | 1793.520 |

The large first-inference and subsequent keyframe/inference frame times remain
in the frame distributions. Baseline already exceeds the 50 ms sensor interval
in its tail. This validates neither a 20 Hz end-to-end deadline nor a large-graph
100 Hz backend deadline. The small differences in guarded-update medians should
not be interpreted as an extraction benefit: the asynchronous graphs differ.

Maximum backend variable counts were 326/322/320 for baseline/reporting/queries.
Observed reporting/query graphs reached five poses and 317/315 landmarks; the
map held as many as 1,023 points, many outside the backend Gaussian. Maximum
owned sparse factor arrays were 196,096 bytes in reporting mode. This prefix
therefore exercises real production factors and scheduling, but only a small
backend pose graph.

Peak process RSS ranges (two processes per mode) were:

| Mode | Minimum peak RSS KiB | Maximum peak RSS KiB |
| --- | ---: | ---: |
| Baseline | 4,287,112 | 4,325,156 |
| Reporting | 4,341,296 | 4,342,180 |
| Queries | 4,310,088 | 4,343,256 |

Inference and allocator state dominate these approximately 4.1 GiB processes.
These differences do not establish a precise marginal snapshot memory cost.
The sparse-array counter excludes Values, caches, map nodes and temporary copies;
old snapshot ownership was retained through subsequent publications.

## 300-frame growth check

One additional baseline/reporting/query run each completed successfully on the
first 300 frames. Reporting observed 1,715 available revisions and reached ten
backend poses with 662 landmarks (672 variables); query mode observed 1,706
available revisions and reached nine poses with 498 landmarks. There were no
query/posterior failures. The different graph sizes are direct evidence that
these asynchronous runs are not an identical-graph experiment.

| Operation | Mode | p50 ms | p95 ms | p99 ms | Maximum ms |
| --- | --- | ---: | ---: | ---: | ---: |
| Guarded backend update | Baseline | 1.654 | 2.411 | 2.867 | 4.434 |
| Guarded backend update | Reporting | 1.746 | 3.218 | 3.880 | 5.344 |
| Full publication | Reporting | 1.487 | 2.656 | 2.874 | 4.074 |
| Capture | Reporting | 0.499 | 0.841 | 0.939 | 1.339 |
| Cold selected joint | Queries | 0.273 | 0.447 | 0.535 | 0.640 |
| Cached selected joint | Queries | 0.00083 | 0.00139 | 0.00216 | 0.00526 |
| Snapshot age after query checks | Queries | 3.911 | 6.285 | 6.757 | 7.099 |
| Frame decode through map counters | Baseline | 23.125 | 58.503 | 125.947 | 1827.770 |
| Frame decode through map counters | Reporting | 22.618 | 74.976 | 145.251 | 1802.380 |
| Frame decode through map counters | Queries | 22.859 | 58.535 | 139.935 | 1797.410 |

Reporting retained at most 440,560 bytes of sparse factor arrays per snapshot.
Peak process RSS was 4,365,948/4,403,688/4,379,360 KiB for
baseline/reporting/queries. The baseline map reached 2,109 points and reporting
2,565 points, so neither frame latency nor total RSS differences isolate the
causal cost of extracting covariance. Larger pose graphs, loop closures and
longer retained histories remain unmeasured.

## Sim(3) compatibility check

A 100-frame run with only `UseSimilarity` changed to true completed and observed
670 revisions. All had status `unsupported`, with zero available covariance
results; estimation was not disabled. Final backend scales included approximately
0.979 and 1.154, so this was not merely a unit-scale configuration test. These
estimates are not a validation of Sim(3) geometry or uncertainty. The earlier
nonunit-scale Jacobian discrepancy still blocks covariance support.

This run exposed a diagnostic-only counter error: an empty sparse factor reported
four bytes for a nonexistent outer-index array. `factor_bytes()` now returns
zero when no factor is stored, with empty/unsupported regression checks. This
change does not modify estimates, factors or covariance results. Timing tables
above predate only this empty-factor counter correction; available Pose3 factor
storage/timing code is unchanged.

## Final checks and remaining limits

The final **uninstrumented** production sources rebuilt and installed after all
timing runs. Both production CTests passed, including fixed-graph estimate
invariance at 1e-12. All four standalone groups passed against the installed
production GTSAM/TBB build, with **315 checks**, and the normal example's
`visualization_queue` test passed. The ordinary EuRoC executable rebuilt too.
The standalone test targets now use the same Linux transitive-RPATH convention
as production; without it, an installed GTSAM's `libcephes-gtsam.so.1` dependency
was not found at process startup. No fake dependency or library substitution was
used. GPU production tests were not run under ASan or ThreadSanitizer; the earlier
host sanitizer experiment remains separate.

All 2,227 GTSAM `.h`/`.hpp`/`.cpp` files compared equal between the production
container and the pinned patched checkout. The final first-party source
manifest also has no differences between the built container copy and the
workspace. The dataset manifest digest for cam0 `sensor.yaml`, `data.csv` and
the first 300 PNG hashes is
`aebaf056313fa792b2f5a772d022659bcaaed0d51f317df68221bf1c883d7fef`.
The source, environment and input manifests are saved with the raw results.

Remaining validation is explicitly bounded: no full sequence, long-duration
memory growth test, loop closure, actual point fusion/alias replacement, induced
failed nonlinear update, or in-process backend reset/rebuild was exercised in
the production worker. Missing/new keys and publication failure/reset identities
have focused coverage, but that is not a production fusion/reset test. The
current backend has no explicit in-place reset API. Sim(3) covariance still
requires the separate geometry/Jacobian repair and validation described in
`online_posterior.md`. No noise calibration, tracking feedback or paper changes
were made.

## Reproduction

Use the existing image or a production image built with this repository's
Dockerfile. The commands below assume the same host layout and an unused
container name; keep the source and dataset mounts read-only.

```sh
mkdir -p /tmp/slam-production-validation
docker run -d --name slam-mer-posterior-validation --gpus all \
  --mount type=bind,source=/home/tom/Documents/PhD/slam-mer,target=/validation/source,readonly \
  --mount type=bind,source=/home/tom/Documents/PhD/unseen_v_slam/datasets,target=/datasets,readonly \
  --mount type=bind,source=/tmp/slam-production-validation,target=/validation/results \
  slam-mer:datasets sleep infinity
```

Inside the disposable container, copy only the changed first-party sources over
the matching image checkout, then build and validate:

```sh
cp -a /validation/source/adjustment/. /opt/slam-mer/adjustment/
cp /validation/source/slam/include/slam.hpp /opt/slam-mer/slam/include/
cp /validation/source/slam/src/slam.cpp /opt/slam-mer/slam/src/
cp /validation/source/examples/euroc/dataloader_euroc.hpp /opt/slam-mer/examples/euroc/
cp /validation/source/examples/euroc/main_euroc.cpp /opt/slam-mer/examples/euroc/
cd /opt/slam-mer
mise exec -- cmake --build build --target install -j 2
mise exec -- cmake -S adjustment/tests/production \
  -B /opt/slam-posterior-production-tests -GNinja -DCMAKE_BUILD_TYPE=Release \
  -DSlamMer_DIR=/opt/slam-mer/build/install/lib/cmake/SlamMer \
  -DSLAM_MER_DIR=/opt/slam-mer/build/install
mise exec -- cmake --build /opt/slam-posterior-production-tests \
  --target test_backend_posterior validate_euroc_posterior -j 2
OMP_NUM_THREADS=2 mise exec -- ctest \
  --test-dir /opt/slam-posterior-production-tests --output-on-failure
mise exec -- cmake --build /opt/slam-mer_examples/build \
  --target slam-mer_euroc visualization_queue_test -j 2
mise exec -- ctest --test-dir /opt/slam-mer_examples/build --output-on-failure
mise exec -- cmake -S adjustment/tests -B /opt/slam-posterior-standalone \
  -GNinja -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_DIR=/opt/slam-mer/thirdparty/gtsam/build/install/lib/cmake/GTSAM
mise exec -- cmake --build /opt/slam-posterior-standalone \
  --target test_gaussian_posterior -j 2
mise exec -- ctest --test-dir /opt/slam-posterior-standalone --output-on-failure
```

A clean production smoke test needs no instrumentation:

```sh
cd /opt/slam-mer_examples
OMP_NUM_THREADS=2 SLAM_MER_FAISS_TEMP_MEMORY_MB=64 \
  /opt/slam-posterior-production-tests/validate_euroc_posterior \
  /datasets/EuroC/MH_01_easy \
  /opt/slam-mer/adjustment/tests/production/pose3.yaml 100 \
  /validation/results/smoke
```

For comparable backend timings, instrument only the container copy, review the
generated diff, rebuild, then invoke the serial replay controller:

```sh
cd /opt/slam-mer
/usr/bin/python3 /validation/source/adjustment/tests/production/instrument_backend.py \
  --reference /validation/source/adjustment/src/covisibility_graph.cpp \
  --target /opt/slam-mer/adjustment/src/covisibility_graph.cpp
# Review /opt/slam-mer/adjustment/src/covisibility_graph.validation.diff.
mise exec -- cmake --build build --target install -j 2
/usr/bin/python3 /validation/source/adjustment/tests/production/run_replays.py \
  --executable /opt/slam-posterior-production-tests/validate_euroc_posterior \
  --sequence /datasets/EuroC/MH_01_easy \
  --settings /opt/slam-mer/adjustment/tests/production/pose3.yaml \
  --output /validation/results/replays --frames 100 --repeats 2
# Repeat with --output /validation/results/replays-300 --frames 300 --repeats 1.
/usr/bin/python3 /validation/source/adjustment/tests/production/summarize_validation.py \
  /validation/results/replays/baseline-1 /validation/results/replays/baseline-2 \
  /validation/results/replays/reporting-1 /validation/results/replays/reporting-2 \
  /validation/results/replays/queries-1 /validation/results/replays/queries-2
# Restore the actual production source after measurement.
cp /validation/source/adjustment/src/covisibility_graph.cpp \
  /opt/slam-mer/adjustment/src/covisibility_graph.cpp
mise exec -- cmake --build build --target install -j 2
```

The baseline switch is honored only by the generated instrumentation. Do not
ship that instrumented source as the production implementation. Use
`/usr/bin/python3` for these tools to avoid unrelated mise-shim trust prompts in
the examples working directory. Raw CSVs, individual logs, per-run summaries,
`runs.json` and pooled JSON summaries are retained under the host results path.

The disposable `slam-mer-posterior-validation` container was stopped after the
final checks and retained for inspection. Its built libraries and test binaries
remain inside it; `docker start slam-mer-posterior-validation` resumes it. The
raw host artifacts remain readable without starting the container. No image or
Git commit was created.
