# MPC RC-car demonstration: implementation checklist

The primary deliverable is a reproducible autonomous RC-car MPC demonstration,
with Laguerre MPC preferred and full-input LTV/nonlinear MPC as baselines.
Historical CSVs and videos in `results/` must not be overwritten.

## Remaining weaknesses: planned work, not implemented

**Paused at the user's request for progress reporting and first GitHub
publication.** No recovery/controller changes have been implemented.

Scope: simulation only; retain Laguerre as the educational controller and
full-input LTV as the matched baseline. Keep current-position measurements and
direction-free motion bounds; do not add obstacle route/velocity forecasts,
training, ROS2, external worlds, or hardware.

The reference remains the immutable
[32-run matrix](./results/published/robustness/summary.csv).
Both controllers passed 9/16 missions. All executed portions met safety criteria,
but that is not proof of safety after termination. Do not relax the clearance,
progress, or no-fallback criteria to improve this score.

### Failure evidence and working hypotheses

- Latency: 77 QP exit flags of -2 per controller; the first Laguerre rejection
  was at 27.20 s. Larger objects: one -2 rejection per controller, with the
  first Laguerre rejection at 23.20 s. Faster crossers: 237 -2 rejections per
  controller, first Laguerre rejection at 19.95 s, followed by a stall.
  These establish QP rejection, not physical impossibility. Planner/MPC
  compatibility, conservative bounds, and restart geometry need diagnosis.
- Missed detections: no verified control at 35.00 s; relevant object age was
  17.75 s in the Laguerre run. Combined uncertainty: no verified control at
  39.65 s; object age was 19.40 s. Growing stale reachable regions are a
  hypothesis to investigate, not grounds for deleting memory.
- Unknown classes: stalled despite fresh relevant measurements. Classifying
  every object as potentially movable can legitimately prevent passage.
- Long marking loss: stalled at 14.45 s with line age 12.50 s, before the
  injected loss ends at 20 s. This is an expected safe-abort candidate, not
  evidence that driving through unobserved road should be enabled.
- The watchdog case produced no interventions. Recovery from a genuine
  deadline overrun remains unverified.

### Phase A: diagnose before changing control (first implementation milestone)

- [ ] Add explicit reasons for planning rejection, yielding, invalid rollout,
      missing stopping reserve, visibility rejection, and no verified control.
- [ ] Tag QP constraint families and log relevant margins, observation IDs/ages,
      current/previous inputs, reference, warm start, and transition state.
- [ ] Capture deterministic fixtures immediately before the first latency,
      larger-object, faster-crosser, and stale-memory failures.
- [ ] Compare full-input feasibility against Laguerre feasibility in those
      fixtures; a failed QP alone does not establish an unavoidable situation.
- [ ] Use offline diagnostic constraint relaxations only to locate conflicts.
      Never apply diagnostically relaxed controls to the plant.

Gate: every failing case has an evidence-backed rejection chain, a reproducible
focused test, and a classification of implementation defect, conservative
information limit, or infeasible/unsupported mission. Preserve unresolved labels.

### Phase B: align planning, MPC, and safe restart

Depends on Phase A. Target latency and the single larger-object rejection first;
then apply the same verified changes to faster crossings.

- [ ] Build entry paths from the actual pose, heading, speed, and previous
      steering/acceleration, rather than accepting geometric clearance alone.
- [ ] Check that an accepted path has a dynamically reachable initial segment
      and compatible braking/holding terminal condition under the actual limits.
- [ ] Share timestamp-aware obstacle bounds across planning, QP constraints,
      nonlinear validation, and braking. Evaluate time-indexed reachable discs
      to avoid applying the largest future disc to every prediction instant.
      These remain direction-free bounds, not directional motion predictions.
- [ ] Select speed from reachable curvature, visibility, and stopping reserve;
      do not merely switch from a stopped reference to nominal speed.
- [ ] Preserve completing the stop before resumption; add hysteresis and
      condition-triggered retries so an unchanged infeasible state does not
      cause repeated tracking/braking cycles or blind QP retries.
- [ ] Keep road, footprint, obstacle, actuator and rate limits hard. Any
      reference/basis/trust-region adjustment still needs nonlinear and
      stopping validation before application.

Gate: original latency and larger-object cases complete with unchanged progress
and clearance criteria and zero fallback for both controllers. Faster-crossing
fixtures either recover under the same criteria or expose a documented
information/feasibility limit; do not promise passage before that is established.
All nine previously successful cases per controller must remain successful.

### Phase C: make uncertain memory less crude, not less safe

Depends on Phase A; integrate with the shared bounds from Phase B.

- [ ] Distinguish positively tracked, possibly missed, outside-view, and
      confirmed-absent states, retaining acquisition timestamps and uncertainty.
- [ ] Investigate conservative reachable regions refined by admissible
      observations instead of indefinitely growing a single disc. Negative
      information may exclude space only when the sensor assumptions justify it.
- [ ] Do not reuse complete-scan free-space certificates when misses are possible,
      delete objects merely because they are old, or infer guaranteed static
      behavior just because recent measured displacement is small.
- [ ] Preserve Bernoulli-miss tests and their probabilistic limitations.
      Separately test an explicitly declared maximum detection-gap assumption
      if deterministic stopping guarantees are desired; it is a different
      sensor contract, not a conclusion from the existing experiments.
- [ ] Test disappearance versus a temporary miss, reappearance near the car,
      long out-of-view intervals, correlated misses, and false clearances.

Gate: the stale-memory failures are explained and addressed under a stated
sensor contract. Every memory exclusion has auditable observation evidence.
Any deterministic claim must survive adversarial tests within its declared
bounds; Bernoulli tests report failures and uncertainty without such a claim.

### Phase D: recover when possible and abort honestly when not

Depends on Phases B and C.

- [ ] Separate intentional yielding, degraded-sensing stop, blocked road,
      recovery attempt, stalled mission, and safe abort in logs and video.
- [ ] Resume only after fresh usable markings, sufficiently informative object
      observations, a feasible rollout, and a verified stopping reserve.
- [ ] In designated recoverable fixtures, target resumption within two seconds
      after the required observable conditions are restored, without chatter.
- [ ] Keep long marking loss and persistent ambiguous blockage as explicit
      failed missions when passage cannot be justified. Test safe abort
      separately without relabeling it successful mission completion.
- [ ] Test fresh-line return after both short and long losses in independent
      recovery fixtures; never use knowledge of the injected end time to drive.

Gate: distinguish recoverable failures from legitimate information-limited
aborts, while preserving the original mission metrics and reporting both.
Do not require 16/16 mission passes by weakening the assumptions or acceptance.

### Phase E: timing and actual overrun behavior

Profile after recovery behavior stabilizes; preserve both controller variants.

- [ ] Re-profile actual optimizer calls and sensor/planning/validation stages;
      reuse invariant work and safe warm starts without skipping safety checks.
- [ ] Target online p99 <=40 ms at the 50 ms sample period, leaving headroom;
      report full-loop p99, worst observed time, and every deadline miss.
      This is a desktop performance goal, not a hard-real-time guarantee.
- [ ] Add deterministic overrun-injection tests that exercise the watchdog.
      Model held-input motion during an emulated actuation delay, rather than
      assuming the car freezes while computation overruns.
- [ ] Verify delayed-command handling, braking validity after the delay,
      explicit failure when no checked action remains, and recovery afterward.
- [ ] Keep injected-delay safety tests separate from wall-clock performance
      collection; compare both controllers on identical delay schedules.

Gate: watchdog intervention is actually exercised and audited. State precisely
which delay bounds were tested; do not turn an iteration cap or desktop timing
percentile into a real-time or hardware-safety claim.

### Phase F: held-out validation and demonstration

Depends on the preceding behavioral fixes; optimize iteration cost first.

- [ ] Run focused failure fixtures during development, then the unchanged
      32-run matrix once coherent changes pass, retaining old results.
- [ ] Predeclare a held-out matrix with additional noise seeds, track starts,
      crossing phases/speeds, latency, miss patterns, and obstacle sizes.
      Include compound disturbances; do not select favorable reruns.
- [ ] Separate feasible-mission targets from fault-response tests. Report
      safety margins, completion/progress, fallback, recovery time, oscillation,
      and actual-call/online/full-loop timing for both controllers.
- [ ] Audit observation causality, bounds, matched exogenous streams, and all
      applied controls again; run full regressions after final code changes.
- [ ] Export one previously failing recoverable case and one clearly labeled
      information-limited abort, with dashboard links to every retained run.

Final gate: no regression in existing successful cases or safety margins,
measured recovery improvements on predeclared cases, and explicit remaining
limits. Additional seeds provide empirical evidence, not universal safety proof.

## Simulation consolidation: current request (steps 1 through 4 only)

- [x] Add seeded bounded sensor errors, acquisition latency, marking loss,
      missed detections, classification uncertainty, and separate plant mismatch.
- [x] Preserve observation timestamps/memory and check continuous stopping reserve.
- [x] Certify stopping inside the last complete sensor footprint before excluding
      out-of-view memories; do not silently forget reachable unseen objects.
- [x] Add replan throttling, invariant QP caching, iteration caps, stage profiling,
      and an opt-in measured deadline watchdog.
- [x] Define a paired 16-case, 32-run Laguerre/full-input-LTV robustness matrix.
- [x] Execute sensor/stopping/replay regressions and full controller regressions.
- [x] Execute and retain every matrix result, including stalled/failed cases.
- [x] Generate a paired dashboard and update sensing-aware visualization.
- [x] Document measured safety/progress/timing outcomes and comparison limits.
- [ ] Step 5: ROS2, external simulator integration, or hardware work. Intentionally
      NOT authorized; this remains a pure simulation project.

The original 40-second kidney regression initially exposed unbounded stale
reachable discs blocking a clear currently observed road. The fix uses complete
negative range observations plus an explicit future stopping-tube visibility
certificate, not a memory timeout. The original nine boundary tests now pass,
including kidney progress above one lap and stable stopping/resumption.
Full MATLAB validation passed all 51 tests (including the nonlinear tracking
baseline); Code Analyzer reported zero messages. All 10 Python tests pass,
including actual GIF rendering, remembered removed objects, report failure
preservation, and excluding waiting ticks from optimizer medians.

The completed 32-run matrix was collected sequentially, with separate controller
warm-up and definitions saved in advance. Both controllers passed 9/16 missions;
all 32 executed portions met the safety criteria, with no contacts or departures.
Latency, long marking loss, missed detections, unknown classes, faster crossers,
larger objects, and combined uncertainty failed mission criteria. These failures
are retained and visible, not recast as successful avoidance.

Mean per-run called-optimizer medians: Laguerre 9.03 ms versus LTV 4.63 ms,
despite eight versus twelve variables. Full-loop deadline misses: 213/11,887
versus 3/11,886 ticks. Online misses: 53 versus 1. No watchdog intervention
occurred in the watchdog pair; that pair is not evidence that the watchdog
handles an actual overrun or guarantees hard real-time.

An audit checked all 23,773 ticks, 81,099 physical-world rows, 47,006 memory
rows, and 1,016,888 line pairs. It verified observation causality, noise bounds,
matched world/indexed noise streams, matching source snapshots, progress
thresholds, and an accepted stopping reserve for every applied action.

- [Dashboard](./results/published/robustness/dashboard.html)
- [Full summary](./results/published/robustness/summary.csv)
- [Paired comparisons](./results/published/robustness/comparison.csv)
- [Noisy Laguerre video](./results/videos/noisy_sensing_chicane.mp4):
  first predeclared noisy case, not a seed retry; 40 s, 1.025 laps, 3.92 cm
  planned-path RMS, 15.11 cm minimum conservative clearance, no contact/fallback.
  H.264, 1560 x 840, 400 frames, 10 fps. Visualization was generated after timing
  collection; captions and legend were moved off the track after a spot check.

## Priority 1: trustworthy tracking

- [x] Use a discrete-time nonlinear prediction model and retain solver warm starts.
- [x] Generate position, heading, and speed references by arc length.
- [x] Share dynamics, physical dimensions, tuning, and constraints across controllers.
- [x] Restore explicit obstacle-free Laguerre and ordinary LTV/nonlinear entry points.
- [x] Separate geometric track CTE, planned-path error, speed error, and progress.
- [x] Verify tracking regressions in MATLAB before proceeding to avoidance.

## Priority 2: feasible, footprint-aware avoidance

- [x] Plan smooth, reachable detours around all relevant obstacles.
- [x] Enforce footprint clearance, road boundaries, speed, and input-rate bounds.
- [x] Check nonlinear predictions and inter-sample motion, not only linear constraints.
- [x] Log solver failures and execute an explicitly checked braking fallback.
- [x] Handle static and appearing obstacles without using hidden obstacles.
- [x] Detect and report unavoidable spawn events rather than claiming success.
- [x] Validate collision-free static, appearing, and multiple-obstacle scenarios.

## Priority 3: reproducibility and visualization

- [x] Replace duplicated simulations with a shared scenario runner.
- [x] Save each run in a new directory with configuration, environment, seed,
      matching track/obstacles, state/control/solver logs, metrics, and status.
- [x] Make geometry and plot limits consistent with the saved configuration.
- [x] Support self-contained run directories in the Python visualizers.

## Priority 4: tests and comparison

- [x] Add regression tests for dynamics, prediction matrices, references,
      Laguerre modes, constraints, clearance, stopping, and invalid input.
- [x] Run end-to-end tracking and obstacle tests with measurable assertions.
- [x] Add controlled controller/basis/horizon/speed/seed comparison sweeps.
- [x] Record computation percentiles, missed deadlines, safety, and completion.
- [x] Run MATLAB code analysis and Python syntax/unit tests.

## Final documentation

- [x] Document commands, assumptions, acceptance criteria, and known limitations.
- [x] Report measured new results without implying simulation certifies hardware safety.
- [x] Record remaining work explicitly here so it survives chat compaction.

## Execution record

- Initial assessment: unobstructed LTV/Laguerre tracking is strong, but the
  historical obstacle runs collide. The old nonlinear model uses a discrete
  state update with MATLAB's continuous-time default. The newest old obstacle
  controller imposes an uncontrollable first-position half-space at 99/300
  saved frames. These are regression targets, not acceptable fallback behavior.
- Priority 1 regression suite passed in MATLAB R2024a: all three controllers
  completed the same 3-second tracking case with approximately 0.69 cm RMS
  geometric CTE and 0.003 m/s speed RMSE. Prediction lifting, arc-length
  spacing, heading wrap, nonlinear time domain, and basis tests also passed.
- Priority 2: 15-second static demonstrations passed for full-input and
  Laguerre MPC; a 15-second appearing-obstacle demonstration passed with no
  collisions, no fallback, and over 20 cm conservative body/obstacle clearance.
  An intentionally unavoidable exposure is correctly reported as a failure.
- Priority 3: immutable run directories include source snapshots and SHA-256
  hashes, environment/configuration, realized exposure times, logs, metrics,
  plan history, and terminal state. Replay reproduces saved trajectories.
- Final validation: all 18 MATLAB regression tests passed across the complete
  and focused regression runs; both Python tests and Python syntax checks
  passed. MATLAB Code Analyzer reports zero messages. Failure tests cover
  blocked braking, unreachable curvature, unsafe exposures, unfulfilled
  exposures, and refusing to advance the plant without a checked control.
- Full benchmark completed: 23/23 cases met simulation safety/completion
  criteria, with zero collisions and zero fallback steps. This does not imply
  all configurations met accuracy targets or the 50 ms timing deadline.
  Results: `results/benchmarks/tp8a64d7e4_0f69_4515_8604_75722b53a271/summary.csv`.
- Latest recommended static demonstration:
  `results/runs/20261027_205650_laguerre_static_tp99d39036_70bf_4131_b7f9_318ddfd12ba5`.
  Four Laguerre modes, decay 0.7, prediction/control horizons 10/5:
  15 seconds, approximately 1.37 laps, 2.27 mm RMS planned-path error,
  21.06 cm minimum conservative obstacle clearance, 22.4 ms p99 full-step
  computation, and zero missed deadlines. The bundle includes its source,
  summary plot, and a verified 15-second H.264 video (150 frames at 10 fps).
- Comparison finding: full-input LTV-MPC is faster than Laguerre MPC for these
  small problems despite using more variables. Nonlinear MPC is an offline
  baseline, not a 20 Hz controller on the measured desktop.

## Optional next phase: hardware-oriented work (not implemented)

- [ ] Add actuator lag, steering calibration, localization noise, and model mismatch.
- [ ] Test uncertainty/friction margins and adversarial obstacle exposures systematically.
- [ ] Profile larger problems before claiming computational benefits from Laguerre modes.
- [ ] Add hardware-in-the-loop validation, real-time scheduling, and an independent
      physical emergency stop before operating a real vehicle.

## Independent random-world request

The user selected an independent random world, not interactive obstacle entry.
The existing controlled appearing scenario must remain clearly distinguished
from this new mode. Do not select or retry a live random seed to obtain a
favorable video.

- [x] Generate fresh-seeded stochastic world events independently of the car,
      reference, controller, feasibility, and simulation outcome.
- [x] Separate finite-range detections and remembered observations from world
      ground truth; expose no future schedule or hidden locations to control.
- [x] Remove car-relative placement and deferred spawns from the new mode.
- [x] Replan from detections, monitor physical truth separately, and report
      stopping, late detection, and collisions without claiming avoidance.
- [x] Support seeded diagnostic replay without making live runs deterministic.
- [x] Test world independence, observation causality, sensor timing, and reactions.
- [x] Export one fresh, unselected random-world performance video and record
      its actual outcome and limitations.

### Independent-world execution record

- All 25 MATLAB tests passed; Code Analyzer reported zero messages. All three
  Python tests passed, including sensor rendering and distinct spawn/detection
  states. A 900-tick audit verified every known ID was already detected and no
  future/hidden obstacle entered the control map; replans occurred exactly at
  sensor discoveries.
- The first fresh, unselected random-world seed was 312882142. No live retry
  was performed. The 45-second run completed 4.62 laps with four world spawns,
  three sensor discoveries, zero contacts, zero fallback, and zero missed
  50 ms deadlines. Minimum conservative clearance was 19.09 cm, planned-path
  RMS error 4.87 mm, and p99 full-step computation 17.42 ms.
- Obstacle 1 spawned at 6.20 s and was detected at 9.95 s; obstacle 2 spawned
  at 31.75 s and was detected at 33.65 s; obstacle 3 spawned and was detected
  at 34.80 s. Obstacle 4 spawned at 43.50 s and remained unknown at the end.
  Nothing was postponed to make avoidance easy.
- Run bundle:
  `results/runs/20261027_224224_laguerre_random_tpf6bdda37_04ac_4ab0_97ea_4d0553679c27`.
  Video verified: H.264, 1560 x 840, 900 frames at 20 fps, exactly 45 seconds.
  A convenient copy is at
  `results/videos/results_ltv_laguerre_random_world.mp4`.
  Extracted frames at 7.0, 10.2, and 44.0 seconds were visually checked:
  spawned-but-unknown, detected/replanning, and a final still-unknown obstacle
  are correctly distinguished. Those temporary inspection frames were removed.

## Denser appearing/disappearing world request

- [x] Add independently sampled private lifetimes without changing the persistent
      random-world defaults or exposing future removal times.
- [x] Clear remembered obstacles only when the sensor observes their absence;
      replan on identity changes, including equal-count replacements.
- [x] Validate expiry, stale-map causality, replay, physical monitoring, and
      actual appearing/disappearing video rendering.
- [x] Record a fresh denser run without selecting a favorable seed, audit its
      actual births/removals/observations, and export a real-time video.
- [x] Record measured results and limitations here and in the README.

### Transient-world execution record

- The 29-test MATLAB regression suite passed, followed by a passing additional
  regression for the actual equal-count replacement at 14.30 s: 30 tests total.
  MATLAB Code Analyzer reported zero messages. Four Python tests passed,
  including real GIF export through active, removed-but-remembered, cleared,
  and terminal-hold frames.
- First fresh transient seed: 273210986. Settings were a 60-second requested
  run, two-second mean independent arrivals, at most 60 total births, and
  independent uniform 6-12 second lifetimes. The short 10/5 Laguerre horizons
  and original 1.2 m/s target were retained. No favorable live seed was selected.
- The run recorded 17 births, 12 removals, 16 discoveries, and 11 sensor-confirmed
  absences. It terminated at 31.85 s as `unsafe_spawn` when object 17 appeared
  within required conservative clearance. No further command was applied;
  no physical contact was confirmed. This is not a completed 60-second mission
  or a demonstrated physical emergency stop. The original recording is retained.
- A 638-tick audit checked every known ID against discovery/absence-observation
  times and every physical occupancy/count against actual birth/removal times.
  It verified all 24 successful replans, including equal-count map replacements
  at 14.30 and 27.80 s. For 313 ticks, a removed object correctly remained in
  sensor memory. Object 1 disappeared at 9.75 s and was cleared at 14.30 s.
- There were zero solver failures and fallback steps; 25 complete steps missed
  50 ms and p99 full-step computation was 75.43 ms. This denser scenario is not
  certified hard real-time.
- Run bundle:
  `results/runs/20261027_231146_laguerre_random_tp55742b82_eb34_4452_bbb4_5a55327add21`.
  Video verified: H.264, 1560 x 840, 698 frames at 20 fps, 34.90 seconds,
  including a three-second terminal-observation hold. The convenient copy at
  `results/videos/results_ltv_laguerre_transient_world.mp4` has the same SHA-256
  as the run-bundle video. Actual frames at 10.20, 14.45, and 34.00 seconds
  were visually checked for stale memory, confirmed absence, and the terminal
  safety status. Temporary inspection images were removed.
- A focused permanent-static check also verified that the new cumulative
  telemetry counts existing obstacles correctly without changing behavior.

## Live moving-obstacle demonstration

- [x] Keep Laguerre successively linearized MPC and the same oval/vehicle model.
- [x] Add an independent moving-world actor and finite-range position sensing;
      give control no obstacle speed, route, or future positions.
- [x] Replan from changed measured coordinates, hold last observations when
      out of range, and separately monitor moving-world truth between ticks.
- [x] Save per-tick truth and measured positions; replay them faithfully in
      video, distinguishing live objects from stale remembered positions.
- [x] Validate sensing, independence, moving contact, saved replay, and video.
- [x] Record and audit a moving-world run, export a video, and document its
      measured outcome and limitations without claiming general moving safety.

### Moving-world execution record

- The complete 34-test MATLAB suite passed, with zero MATLAB Code Analyzer
  messages. Five Python tests passed, including actual moving GIF export,
  exact physical/remembered coordinates, and rejection of missing history.
  The contact test checks an obstacle moving into a stationary vehicle that
  was initially clear, rather than only testing a static overlap.
- Recorded run:
  `results/runs/20261028_115214_laguerre_moving_tp11823130_e31f_47b9_ab3a_3051f49ad905`.
  This is a deterministic demonstration, not a randomly selected easy seed.
  Three radius-0.12 m objects travel at 0.18 m/s counterclockwise, independently
  of the car. Their routes/speeds are private to the world, not control inputs.
- Completed 25 seconds and 2.30 laps, with zero contacts, solver failures,
  prediction rejections, and fallback steps. Minimum conservative clearance:
  21.48 cm to moving objects and 12.57 cm to road boundaries. Planned-path
  RMS error: 3.93 cm. No controller training was performed.
- Median solve: 11.98 ms; p99 solve: 16.70 ms; zero solve deadlines missed.
  Complete-loop p99: 56.27 ms, with 43/500 missed 50 ms loop deadlines.
  Replanning every changed observation is not yet a hard-real-time guarantee.
- Audited all 500 control ticks and 1,500 world samples: sensor-only updates,
  no future measurements, frozen out-of-range positions, known IDs/counts,
  and exactly 500 measured-geometry replans. Each object travelled 4.49 m
  over the recorded frames; 483 ticks included stale remembered positions.
- Video:
  `results/videos/results_ltv_laguerre_moving_world.mp4`.
  Verified H.264, 1560 x 840, 500 frames at 20 fps, exactly 25 seconds.
  The convenience copy and run-bundle copy have identical SHA-256 hashes.
  Frames at 5 and 18 seconds were visually inspected for motion, vehicle,
  sensor coverage, and distinct actual versus stale remembered objects.
  Temporary inspection images were removed.
- Limitations: ideal sensing/model; slow same-direction movers on the oval;
  zero-obstacle-motion assumption within each horizon; no crossing/adversarial
  motion or robust moving-obstacle guarantee. Nonlinear truth validation does
  not turn the successively linearized controller into full nonlinear MPC.

## Boundary-sensed tracks, crossing objects, and linearization safeguards

User choices: simulated local paired-boundary measurements, not camera images;
live positions with isotropic admissible speed bounds, not directional forecasts.

- [x] Support oval, kidney, chicane, and custom paired closed boundaries.
- [x] Derive an open local corridor/reference from sensed left/right lines;
      keep the full track out of planning, MPC, and candidate validation.
- [x] Add static, temporary, and transversely crossing objects with independent
      world motion; supply only current measured positions and class speed bounds.
- [x] Add proactive yielding, checked jerk-limited braking ending at zero
      acceleration, and live-measurement-based resumption.
- [x] Add bounded optimization steps, additional relinearization passes,
      measured affine-versus-nonlinear error thresholds, and explicit rejection.
- [x] Validate open geometry, local sensing causality, line-based road clearance,
      crossing motion, yielding/resumption, guard thresholds, replay, and video.
- [x] Record matching controller settings on different tracks, audit all
      measured inputs and outcomes, and export crossing-obstacle videos.
- [x] Record outcomes and limitations; do not claim camera perception,
      arbitrary-track safety, prediction-free robustness, or hard real-time.

### Boundary/crossing execution record

- All 43 MATLAB tests passed; Code Analyzer reported zero messages. Six Python
  tests passed, including real video/GIF rendering with changing active-object
  IDs and exact measured left/right line overlays.
- Both final recordings used identical 0.8 m/s target, 12/6 horizons, four
  Laguerre modes, enabled steering/acceleration trust bounds, six maximum
  relinearizations, and 1.5 cm / 0.03 rad affine/nonlinear error limits.
  Initialization, sensor, and independent world use the physical track;
  planner/MPC/nonlinear candidate checks receive only the sensed open corridor
  and current in-range object measurements with class motion bounds.
- Chicane final run:
  `results/runs/20261028_121518_laguerre_boundary_crossing_tp588889fb_c066_4693_9676_36e3a3a3b60a`.
  Completed 40 s, 1.031 laps, 11.87 cm minimum conservative obstacle clearance,
  34.55 cm road clearance, 2.60 cm planned-path RMS error. Zero contacts,
  solver failures, prediction rejections, and fallback steps. Yielded at
  3.45 s and resumed at 10.95 s after a complete stop (150 yield/wait ticks).
  Full-loop p99 40.83 ms; one of 800 full-loop deadlines missed.
- Kidney final run:
  `results/runs/20261028_121542_laguerre_boundary_crossing_tpca9a7759_9077_404e_a55b_fcf4cf317f5d`.
  Completed 40 s, 1.131 laps, 11.88 cm minimum conservative obstacle clearance,
  34.55 cm road clearance, 2.82 cm planned-path RMS error. Zero contacts,
  solver failures, prediction rejections, and fallback steps. Yield/resume
  pairs: 3.10/10.95 s and 38.60/39.15 s (168 yield/wait ticks).
  Full-loop p99 39.43 ms; two of 800 full-loop deadlines missed.
- The first kidney development recording was retained:
  `results/runs/20261028_121156_laguerre_boundary_crossing_tp43bf91e5_8e87_4394_983e_9c3199c57335`.
  It terminated at 6.05 s when the conservative braking certificate was lost.
  Expanding early yielding from the nominal path to the marked lane fixed the
  stopping reserve. A later unsaved deterministic diagnostic exposed an
  infeasible mid-braking handover; completing braking before resumption fixed
  that case. The 40-second kidney regression exercises both corrections.
  No random seed selection or obstacle trajectory edits were used.
- Audited all 1,600 control ticks, 5,440 world rows, and 68,563 local line
  pairs for sensing range, current-only measured coordinates/IDs, class speed
  bounds, and accepted model-error thresholds. Accepted affine/nonlinear
  position and heading errors remained below the specified limits.
- Videos:
  `results/videos/results_ltv_laguerre_boundary_chicane.mp4` and
  `results/videos/results_ltv_laguerre_boundary_kidney.mp4`.
  Both verified H.264, 1560 x 840, 400 frames, 10 fps, exactly 40 seconds;
  both convenience copies match their run-bundle SHA-256 hashes.
  Chicane frames at 7/12 s and a kidney frame at 20.5 s were visually checked
  for actual transverse crossing, stopped/waiting and resumed vehicle motion,
  locally sensed boundary overlays, and the static-obstacle detour.
  Temporary inspection images were removed.
- Limitations: ideal paired-line sensing and pose, reliable marking/class
  association, bounded obstacle speeds, no directional motion forecasts,
  conservative reachable discs, and no guarantee for arbitrary custom tracks,
  marking loss, late spawns, adversarial movers, or hardware timing.
