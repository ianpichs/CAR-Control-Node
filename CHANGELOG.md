# Changelog

All notable changes to this project are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
Versioning: [Semantic Versioning](https://semver.org/) — MAJOR.MINOR.PATCH

---

## [0.3.7] — 2026-05-11 UTC  ← BEST RESULT

### Changed
- **Reverted to v0.2.9 Q/R, max_steer_rate 4.0 → 1.3, lookahead 60 → 40**: the Q
  weight exploration (v0.3.4–0.3.6) confirmed that extreme weights broke tracking
  without improving the C2 transition. Reverted to v0.2.9 baseline and set
  lookahead_steps=40 (4.5 m / 0.4 s preview) — a conservative step back from the
  lookahead=60 breakthrough to reduce the C1 pre-steer regression while retaining
  most of the C2 benefit.

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 1.0 | 5.0 | 8.0 | 1.0 | 1.0 | 1.3 | 40 |

### Results
- C1 entry peak: ~0.758 m inside (best C1 recorded this session)
- SS: ~−0.47 m inside bias (slight regression vs v0.2.9 ~0m; lookahead pre-steer
  pulling car inside during circular tracking)
- C2 transition peak: ~0.518 m (**best C2 recorded — 14× improvement from 7.5 m baseline**)
- **Overall: best result of the entire tuning programme**

---

## [0.3.6] — 2026-05-11 UTC

### Changed
- q_steer 1.0 → 50 (exploring whether high steer penalty forces earlier correction
  at circle transitions, with all other weights from v0.3.5)

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 50.0 | 100.0 | 5.0 | 8.0 | 50.0 | 1.0 | 4.0 | 60 |

### Results
- Degraded — q_steer is DARE-insensitive in this regime (K[steer] barely changes).
  Confirmed prior finding from v0.2.3. Prompted revert to v0.2.9 weights.

---

## [0.3.5] — 2026-05-11 UTC

### Changed
- q_e_psi 1.0 → 100, q_e_y 100 → 50 (attempting heading-dominant correction)

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 50.0 | 100.0 | 5.0 | 8.0 | 1.0 | 1.0 | 4.0 | 60 |

### Results
- Degraded — extreme heading weight drove car off-track at C2 transition.

---

## [0.3.4] — 2026-05-11 UTC

### Changed
- q_e_y 4.0 → 100 (attempting to force more aggressive lateral correction at transitions)

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 100.0 | 1.0 | 5.0 | 8.0 | 1.0 | 1.0 | 4.0 | 60 |

### Results
- Degraded — extreme q_e_y causes oscillation at this operating point. Confirms
  q_e_y ceiling ≈ 4.0 for current damping levels (q_vy=5.0, q_r=8.0).

---

## [0.3.3] — 2026-05-11 UTC  ← BREAKTHROUGH

### Changed
- **lookahead_steps 0 → 60** (6.75 m / 0.6 s preview at 11.25 m/s): following the
  finding that max_steer_rate=4.0 still hit the steering angle limit (±0.400 rad)
  rather than the rate limit, switched focus back to feedforward preview. 60 steps
  spans the circle boundary — kappa_ref flips sign mid-horizon, giving the LQR
  ~0.6 s warning before the geometric C1→C2 transition.

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 1.0 | 5.0 | 8.0 | 1.0 | 1.0 | 4.0 | 60 |

### Results
- C1 entry: ~3.3 m (regression — car pre-steered early due to kappa step on straight
  approach being visible 6.75 m out)
- SS: small
- C2 transition: ~5.0 m (**best C2 to date; first time car ever turned early for C2**)
- Key finding: lookahead feedforward is the primary lever for the C2 spike, not
  steer rate or Q/R weights. The early-turn behaviour was observed for the first time.

---

## [0.3.2] — 2026-05-11 UTC

### Changed
- **max_steer_rate 1.3 → 4.0 rad/s**: raising to the DUT25 physical maximum to
  remove what was believed to be the rate-limiting constraint at circle entry.

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 1.0 | 5.0 | 8.0 | 1.0 | 1.0 | 4.0 | 0 |

### Results
- No improvement. Log confirmed x0.steer pinned at ±0.400 rad during both C1 entry
  and C1→C2 transition — the binding constraint is the **steering angle limit**, not
  the rate. At κ = 1/7.5 m⁻¹ the steady-state steer_ref ≈ 0.200 rad; transient
  overshoot saturates the remaining 0.200 rad of margin regardless of actuator speed.
- C1 and C2 peaks unchanged from v0.3.1. Kept max_steer_rate=4.0 for subsequent
  experiments (no cost since angle is binding).

---

## [0.3.1] — 2026-05-11 UTC

### Changed
- Reverted Q/R to v0.2.9 best params for gain-scheduling baseline run

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 1.0 | 5.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- C1 entry: ~2.4 m
- SS: ~−0.5 m inside bias
- C2 transition: ~7.5 m

---

## [0.3.0] — 2026-05-11 UTC

### Added
- **Gain scheduling via lookup table**: DARE is now solved at startup for each
  entry in `vx_schedule` (default [3, 5, 7, 9, 11.25] m/s). At runtime K is
  linearly interpolated from the two bracketing entries using the current
  measured speed from AccRequest. Speeds below the minimum schedule entry clamp
  to the lowest K.
- `vx_schedule` ROS parameter (list of floats) to configure the schedule points.
- `_current_vx` cached from AccRequest and used for feedforward references
  (r_ref, vy_ref) and trajectory propagation — previously all used the fixed
  `vx_op=11.25` even during the acceleration phase.

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 20.0 | 100.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.27] — 2026-05-11 UTC

### Changed
- q_vy 15.0 → 1.0, q_r 16.0 → 1.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 20.0 | 100.0 | 1.0 | 1.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.26] — 2026-05-11 UTC

### Changed
- q_e_y 4.0 → 20.0, q_e_psi 8.0 → 100.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 20.0 | 100.0 | 15.0 | 16.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.25] — 2026-05-11 UTC

### Changed
- q_e_psi 1.0 → 8.0, q_r 8.0 → 16.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 8.0 | 15.0 | 16.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.24] — 2026-05-11 UTC

### Changed
- q_e_psi 0.0 → 1.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 1.0 | 15.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.23] — 2026-05-11 UTC

### Changed
- Reverted to v0.2.16 (q_e_y 10.0→4.0, r_steer_rate 0.25→1.0)

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 0.0 | 15.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.22] — 2026-05-11 UTC

### Changed
- r_steer_rate 1.0 → 0.25

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 10.0 | 0.0 | 15.0 | 8.0 | 1.0 | 0.25 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.21] — 2026-05-11 UTC

### Changed
- q_vy 20.0 → 15.0 (reverted to v0.2.18)

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 10.0 | 0.0 | 15.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.20] — 2026-05-11 UTC

### Changed
- q_e_y 20.0 → 10.0, q_vy 15.0 → 20.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 10.0 | 0.0 | 20.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.19] — 2026-05-11 UTC

### Changed
- q_e_y 10.0 → 20.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 20.0 | 0.0 | 15.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.18] — 2026-05-11 UTC

### Changed
- q_e_y 2.0 → 10.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 10.0 | 0.0 | 15.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.17] — 2026-05-11 UTC

### Changed
- q_e_y 4.0 → 2.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 2.0 | 0.0 | 15.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.16] — 2026-05-11 UTC

### Changed
- q_vy 10.0 → 15.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 0.0 | 15.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.15] — 2026-05-11 UTC

### Changed
- q_vy 5.0 → 10.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 0.0 | 10.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.14] — 2026-05-11 UTC

### Changed
- Reverted to v0.2.9 base (q_e_y=4.0, q_r=8.0, r_steer_rate=1.0), q_e_psi 1.0 → 0.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 4.0 | 0.0 | 5.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- TBD

---

## [0.2.13] — 2026-05-11 UTC

### Changed
- q_r 8.0 → 10.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 2.0 | 1.0 | 5.0 | 10.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- C1: ~2.5m, SS: ~0m, C2: ~8.5m — worse than v0.2.9 on both metrics

---

## [0.2.12] — 2026-05-11 UTC

### Changed
- Reverted to v0.2.9 base (q_vy 7.0→5.0, r_steer_rate 0.5→1.0), q_e_y 4.0 → 2.0

### Parameters
| q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | max_rate | lookahead |
|-------|---------|------|-----|---------|---------|----------|-----------|
| 2.0 | 1.0 | 5.0 | 8.0 | 1.0 | 1.0 | 1.3 | 0 |

### Results
- C1: ~3.0m, SS: noisy, C2: ~7.5m — q_e_y too low; post-C2 oscillations, worse than v0.2.9

---

## [0.2.7] — 2026-05-11 UTC

### Changed
- **r_steer_rate 1.5 → 1.0**: lower control effort penalty allows the LQR to use
  more of the available 1.3 rad/s steer rate budget. At 0.5 rad/s the rate limit was
  always the binding constraint so r_steer_rate had little effect; at 1.3 rad/s the
  LQR may be voluntarily under-steering due to the higher penalty. R4 historical data
  (from log measurements) showed r_steer_rate=1.0 improved SS offset vs 1.5 — testing
  whether C1/C2 entry peaks also improve now that precise plot comparison is available.

### Results (from Foxglove plot)
- C1 peak: +2.1m (improved from +2.5m baseline)
- SS offset: ~0m (improved from −0.5m — effectively eliminated)
- C2 peak: −7.5m (improved from −8.0m)
- steer[0] actively using ±0.4 rad range throughout — LQR no longer conservative
- New best run. SS offset elimination suggests it was a steering effort issue, not
  a feedforward model mismatch as previously diagnosed.

---

## [0.2.11] — 2026-05-11 UTC

### Changed
- **q_vy 5.0 → 7.0**: more lateral velocity damping to test whether the elevated
  damping level allows q_e_y to be raised above its current 4.0 ceiling without
  inducing oscillation. q_vy has been fixed at 5.0 since R2 and not re-evaluated
  at the new operating point (r_steer_rate=0.5, max_steer_rate=1.3).

### Results (0.2.11 — q_vy=7.0, from plot)
- C1: ~1.7m (improved from ~2.2m — best C1 recorded this session)
- SS: ~0m (held)
- C2: ~−8.0m (regressed from −7.5m)
- Tradeoff: higher vy damping corrects the C1 lateral velocity buildup more
  aggressively but conflicts with the C2 yaw rate reversal dynamics.
  C1 −0.5m better, C2 −0.5m worse — net wash on total error.
- Decision pending: whether to keep q_vy=7.0 and explore q_e_y above 4.0,
  or revert to q_vy=5.0 (cleaner C2).

### Session parameter evolution summary (2026-05-11)

| Version | q_e_y | q_vy | q_r | r_steer | max_rate | C1 | SS | C2 |
|---------|-------|------|-----|---------|----------|----|----|----|
| R2 baseline (logs) | 4.0 | 5.0 | 4.0 | 1.5 | 0.5 | 2.3m | −0.47m | 7.3m |
| 0.2.7 (r_steer 1.0) | 4.0 | 5.0 | 4.0 | 1.0 | 1.3 | 2.1m | ~0m | 7.5m |
| 0.2.9 (q_r 8.0) | 4.0 | 5.0 | 8.0 | 1.0 | 1.3 | 2.0m | ~0m | 7.5m |
| **0.2.9 ← session best** | **4.0** | **5.0** | **8.0** | **1.0** | **1.3** | **2.0m** | **~0m** | **7.5m** |
| 0.2.10 (r_steer 0.5) | 4.0 | 5.0 | 8.0 | 0.5 | 1.3 | ~2.2m | ~0m | ~7.5m |
| 0.2.11 (q_vy 7.0) | 4.0 | 7.0 | 8.0 | 0.5 | 1.3 | 1.7m | ~0m | 8.0m |

---

## [0.2.10] — 2026-05-11 UTC

### Changed
- **r_steer_rate 1.0 → 0.5**: continuing the trend that lower control effort penalty
  unlocks more of the available 1.3 rad/s steer bandwidth. 1.5→1.0 produced the
  largest single gain of the session (C1 2.5→2.1m, SS −0.5→0m, C2 8.0→7.5m).
  Testing whether further reduction continues the trend or introduces oscillation
  from over-aggressive steering.

### Results (0.2.9 — q_r=8.0, from plot)
- C1: ~2.0m (marginal improvement from 2.1m, within noise)
- SS: ~0m (held)
- C2: ~7.5m (unchanged)
- Conclusion: q_r is DARE-insensitive in this regime; kept at 8.0 (no regression).

### Results (0.2.10 — r_steer_rate=0.5, from plot)
- C1: ~2.2m (within noise of 2.0m — marginal, trend flattening)
- SS: ~0m (held)
- C2: ~7.5m (unchanged)
- Conclusion: r_steer_rate trend is exhausted. 1.5→1.0 was the impactful step;
  1.0→0.5 yields diminishing returns. Effort penalty no longer a limiting factor.

---

## [0.2.9] — 2026-05-11 UTC

### Changed
- **q_e_y reverted 8.0 → 4.0**: q_e_y=8.0 produced C1 oscillation (~3m with ringing)
  and C2 regression to −8.5m. q_vy=5.0 and q_r=4.0 cannot damp the more aggressive
  lateral corrections at this steer rate — same relationship observed in original rounds.
  q_e_y=4.0 is confirmed as the ceiling for current damping levels.
- **q_r 4.0 → 8.0**: yaw rate build-up lag is the primary driver of C2 entry peak.
  Higher q_r makes the LQR prioritise correcting yaw rate error more aggressively,
  which should reduce the time the car takes to reach r_ref during the C1→C2 reversal.
  At max_steer_rate=1.3 the LQR has sufficient bandwidth to act on this; at 0.5 rad/s
  the rate limit would have prevented any benefit.

### Results (0.2.8 — q_e_y=8.0, from plot)
- C1: ~3m with oscillation (regression from 2.1m)
- SS offset: ~0m (held from r_steer_rate change)
- C2: ~−8.5m (regression from −7.5m)
- Conclusion: q_e_y=8.0 is too aggressive for current damping; 4.0 is the ceiling.

---

## [0.2.8] — 2026-05-11 UTC

### Changed
- **q_e_y 4.0 → 8.0**

---

## [0.2.6] — 2026-05-11 UTC

### Changed
- **lookahead_steps set to 0 (disabled)**: all tested values (20, 25, 40) produced
  net negative results on the skidpad. Root cause: the skidpad path has an abrupt
  straight-to-circle curvature transition (0 → 1/7.5 m⁻¹ step) with no transition
  curve. Lookahead feedforward works best with gradual curvature ramps; on the
  skidpad it causes the car to pre-steer on the straight approach, creating a lateral
  error that compounds with the circle-entry transient. Returning to nearest-waypoint
  curvature feedforward (lookahead_steps=0) for clean R2 baseline measurement.

### Next
- Re-establish clean R2 baseline with the Foxglove error plot (not available during
  original R1–R6 tuning rounds). Explore q_r / q_vy interaction at max_steer_rate=1.3
  — those weights were tuned at 0.5 rad/s and the optimal damping may differ now.

---

## [0.2.5] — 2026-05-11 UTC

### Changed
- **lookahead_steps reduced 40 → 25** (2.8 m / 0.25 s preview): 40 steps caused
  oscillatory behaviour on the straight approach to C1 — the car began pre-steering
  4.5 m before the geometric entry, building a lateral error on the straight that the
  LQR then had to fight simultaneously with the circle entry transient. 25 steps is a
  conservative step back toward the 20-step baseline that showed clean single-spike C1
  behaviour, with slightly more preview than 20 to retain the SS-offset improvement.

---

## [0.2.4] — 2026-05-11 UTC

### Added
- **Lookahead curvature feedforward** (`lookahead_steps` parameter): the curvature
  feedforward (steer_ref, r_ref, vy_ref) now reads from a point `lookahead_steps`
  waypoints ahead of the nearest waypoint rather than the nearest waypoint itself.
  This gives the LQR preview of upcoming path curvature so it begins pre-steering
  before the geometric circle entry, reducing the circle-entry lateral error transient.
  Waypoint spacing = vx_op × dt_traj = 0.1125 m; default 40 steps = 4.5 m / 0.4 s
  of preview at skidpad speed. The e_y and e_psi error states continue to use the
  nearest waypoint — only the feedforward references use the lookahead index.
- `lookahead_steps` exposed as a ROS parameter (default: 40) in both launch files.
  Adjust without recompiling by changing the launch file value and rebuilding
  mission_control only (no change to controller_node.py required).

### Changed
- Log line now prints `preview=Nwp` showing the effective lookahead distance in
  waypoints, confirming the parameter is active at runtime.

### Notes
- Q/R weights remain at R2 best: q_e_y=4.0, q_e_psi=1.0, q_vy=5.0, q_r=4.0,
  q_steer=1.0, r_steer_rate=1.5
- max_steer_rate=1.3 rad/s (simulator physical limit)
- Observed improvement at lookahead=40: SS offset reduced to ~0m during steady
  circular tracking; C2 peak slight reduction (~6.5m vs ~7–7.5m at lookahead=0).
  C1 peak (~2.5m) unchanged — dominated by yaw rate build-up lag, not steer delay.

---

## [0.2.3] — 2026-05-11 UTC

### Changed (tuning)
- **Confirmed best Q/R parameters (R4)**: after six systematic one-change-at-a-time rounds,
  the optimal weights are:
  `q_e_y=8.0, q_e_psi=1.0, q_vy=5.0, q_r=4.0, q_steer=1.0, r_steer_rate=1.0`
  Both launch files (`controllers.launch.xml` and `lqr_pid_controller.launch.py`) set to these values.

### Findings
- **DARE insensitivity to q_steer**: Raising q_steer from 1.0 → 4.0 (Round 5) produced only a
  0.5% change in K[steer] (K[steer] ≈ 15.53 is already the dominant eigenvalue). Performance
  regressed: C1 3.40m, C2 9.19m vs R4 baseline 2.56m / 7.66m. q_steer is a dead knob in this
  operating regime. Reverted to 1.0.
- **q_e_psi floor = 1.0**: Lowering q_e_psi from 1.0 → 0.5 (Round 6) produced only 0.1% change
  in K[e_psi]. Car failed to complete track — C2 error reached −13.7m with heading errors ±112°
  as reduced heading correction allowed oscillations to grow compoundingly. 1.0 is the minimum
  stable value. Reverted to 1.0.
- **max_steer_rate is the primary bottleneck**: At circle entries u_raw reaches 5–20 rad/s but
  is clamped to 0.5 rad/s. C1 peak (~2.56m) and C2 peak (~7.66m) are both driven by steer rate
  saturation, not by Q/R weights. Q/R tuning has reached its ceiling.
- **Steady-state offset (~−0.35m)**: Caused by feedforward model mismatch; not eliminable via
  Q/R without integral action.

### Notes
- Tuning round history (all at request_interval=0.004s, N=100, Tf=1.0s, vx_op=11.25 m/s):

| Round | q_e_y | q_e_psi | q_vy | q_r | q_steer | r_steer | C1 peak | SS offset | C2 peak | Completed |
|-------|-------|---------|------|-----|---------|---------|---------|-----------|---------|-----------|
| R1 | 7.0 | 1.0 | 2.0 | 2.0 | 1.0 | 1.5 | ~2.9m | ~−0.47m | diverged | No |
| R2 | 4.0 | 1.0 | 5.0 | 4.0 | 1.0 | 1.5 | ~2.3m | ~−0.47m | ~7.3m | Yes |
| R3 | 8.0 | 2.0 | 5.0 | 4.0 | 1.0 | 1.0 | ~3.1m | ~−0.35m | ~8.1m | Yes |
| **R4** | **8.0** | **1.0** | **5.0** | **4.0** | **1.0** | **1.0** | **~2.56m** | **~−0.35m** | **~7.66m** | **Yes ← BEST** |
| R5 | 8.0 | 1.0 | 5.0 | 4.0 | 4.0 | 1.0 | ~3.40m | ~−0.35m | ~9.19m | Yes |
| R6 | 8.0 | 0.5 | 5.0 | 4.0 | 1.0 | 1.0 | — | — | −13.7m | No |

- Next experiment: raise `max_steer_rate` from 0.5 → 1.0 rad/s (single-parameter change).
  Simulator physically allows ~1.3 rad/s; DUT25 actual ~4.0 rad/s.

---

## [0.2.2] — 2026-05-11 UTC

### Changed
- **Control frequency: 40 Hz → 250 Hz**: `request_interval` for skidpad_manager
  overridden to 0.004 s in the `mode=lqr` launch block. skidpad_manager's
  pose_callback already fires at 250 Hz (driven by `/se/vehicle/pose`); the
  previous 0.025 s interval was the only bottleneck. The LQR now receives fresh
  x0 state and recomputed waypoints every 4 ms instead of every 25 ms.
  At 11.25 m/s this reduces the arc traveled between corrections from 28 cm to
  4.5 cm, cutting per-step error accumulation by 6×.
- **MultiThreadedExecutor**: `main()` now uses `rclpy.executors.MultiThreadedExecutor`
  so that `_on_accel_request` callbacks are not starved by `_on_opt_request`
  at the higher 250 Hz rate.

### Notes
- MPC mode is unaffected: the `request_interval` override exists only inside the
  `mode=lqr` launch group; MPC loads `skidpad_mpc_python_params.yaml` which
  retains `request_interval: 0.025`.
- `max_steer_rate` remains 0.5 rad/s for this release. The simulator enforces
  ~1.3 rad/s at the front wheel (5.24 rad/s column ÷ ~4:1 ratio); the
  `full-24` measured value is 4.0 rad/s. Rate increase is deferred until
  250 Hz baseline performance is evaluated.

---

## [0.2.1] — 2026-05-10 22:30 UTC

### Fixed
- **Path curvature feed-forward**: LQR error state now subtracts steady-state
  reference values for r, steer, and vy before applying K. Without this, K[steer]
  and K[r] commanded opposing the turn at the moment circular motion began (u_ss ≈
  −3.2 rad/s against the right circle), causing immediate divergence into random
  spins. Reference values derived from path curvature κ computed locally from
  adjacent waypoints in each OptRequest:
    r_ref = vx · κ,  steer_ref = −(LF+LR) · κ,  vy_ref = 0.2429 · vx · r_ref
  Handles both skidpad circles and straights automatically (κ≈0 on straights).
- **Corrected cornering stiffness**: Cf=Cr=20 000 N/rad replaced with values
  interpolated from the DUT25 measured tyre stiffness curve (same data used by
  LPVMPC.get_tyre_stiffness): Cf=18 877 N/rad (front, 379.6 N/tyre load),
  Cr=24 293 N/rad (rear, 503.3 N/tyre load). This corrects a 17 % underestimate
  of rear stiffness that caused model/plant mismatch in the K matrix.

### Added
- `_compute_path_curvature(ref_idx, msg)` method: estimates signed path curvature
  κ = dθ/ds from heading angle difference between adjacent waypoints, used for the
  feed-forward reference computation above.

---

## [0.2.0] — 2026-05-10 17:00 UTC

### Added
- Dynamic lateral bicycle model with DUT25 vehicle parameters:
  m=180 kg, Iz=294 kg·m², lf=0.872 m, lr=0.658 m, Cf=Cr=20 000 N/rad
  (linearized at vx=11.25 m/s skidpad operating point)
- 5-state LQR error vector: [e_y, e_psi, vy, r, steer] — replaces the
  3-state kinematic model; now includes explicit lateral velocity and yaw
  rate dynamics driven by tyre cornering stiffness
- N-step trajectory propagation (N=100, Tf=1.0 s, dt=10 ms) using
  nonlinear Euler integration — produces the 101-element OptResult.steer
  array required by skidpad_manager's time-interpolated latency compensator
- Gain-scheduled longitudinal PID with three speed-range gain sets matching
  DUT25 conventional/config/longitudinal_pid_parameters.yaml exactly
- Quadratic speed feed-forward term (FF_A·v² + FF_B·v + FF_C) derived from
  DUT25 measured vehicle data in speed_ff_term_calc.py
- All Q/R weights and PID gains exposed as individual ROS 2 parameters for
  runtime tuning without recompilation
- Full inline documentation: module-level docstring covers system model,
  ROS interface, how skidpad_manager uses OptResult, and tuning guide

### Changed
- ROS interface completely replaced to match DUT25 skidpad pipeline:
    Subscribes: /controllers/opt_requests (OptRequest), /controllers/accel_request (AccRequest)
    Publishes:  /controllers/opt_results (OptResult), /controllers/long (PIDErrors)
- LQR control output changed from steering angle (δ) to steering rate (dδ/dt);
  steering angle is now a state that integrates the control input
- Control architecture changed from timer-driven to callback-driven:
  each OptRequest triggers an immediate LQR response (microsecond latency
  vs 25 ms timer); each AccRequest triggers an immediate PID response
- Vehicle model discretization timestep changed from DT=0.05 s (20 Hz) to
  DT_TRAJ=0.01 s (100 Hz) to match the manager's interpolation time axis
- package.xml version bumped to 0.2.0; dependencies updated (removed
  nav_msgs, geometry_msgs, std_msgs; added controller_msgs)

### Fixed
- Entry point bug in setup.py: module path was lqr_pid_controller.lqr_pid_controller
  (non-existent); corrected to lqr_pid_controller.controller_node

### Removed
- Kinematic bicycle model (wheelbase-only parameterisation, valid only at low speed)
- Generic nav_msgs/Odometry + geometry_msgs/PoseStamped subscriber interface
- Standalone throttle [0, 1] output (replaced by DUT25-compatible PIDErrors)
- Standalone steering angle output (replaced by steering_rate + steer sequence in OptResult)

---

## [0.1.0] — 2026-02-26

### Added
- Initial implementation: discrete LQR steering + PID throttle
- 3-state kinematic bicycle model: state = [e_lat, e_yaw, ė_lat],
  control = steering angle δ; linearized using wheelbase L and speed v
- DARE solver (scipy.linalg.solve_discrete_are) for optimal gain computation
- Zero-Order Hold (ZOH) discretization via scipy.signal.cont2discrete
- Fixed-rate timer control loop decoupled from topic publish rates
- ROS 2 parameter system for vehicle parameters and cost weights

### Fixed
- Integral windup: added explicit clamp to PID accumulator
- Triple-callback publishing: moved control output to timer; callbacks
  only cache incoming data
- Logger API: corrected to ROS 2 Python throttled logging syntax
  (warning(..., throttle_duration_sec=N))
