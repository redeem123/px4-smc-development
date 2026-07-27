# Type 3 evidence plan and Type 4 PC-PISTA research plan

Evidence cutoff: **2026-07-27**

## Status and claim boundary

This document separates two projects that must not be conflated:

- **Track A — existing-controller evidence.** `MC_RATE_CTRL_T=3` remains the
  legacy ASTSMC controller. Its proper-implicit scalar super-twisting kernel is
  prior art. Track A may support replication, experimental characterization,
  and implementation findings, but not a new-algorithm claim.
- **Track B — theorem-led research.** A new controller is reserved as
  `MC_RATE_CTRL_T=4`, provisionally named **PC-PISTA**
  (Priority-Constrained Proper-Implicit Super-Twisting Allocation). Its only
  candidate algorithmic contribution is to place a constrained allocation
  optimality system inside the same implicit generalized equation as the
  multivariable STA state.

The current evidence does **not** establish:

- a validated final Type-3 card or a robust feasible Type-3 tuning region;
- Type-3 superiority or broad non-inferiority to PID or Mode 2;
- controlled flight validation of the allocator-aware 2026-07-24 Type-3 image;
- a complete physical identification of inertia, effectiveness, or actuator
  dynamics;
- novelty, well-posedness, finite-time convergence, or a measurable hardware
  benefit for the proposed Type-4 controller.

The corrected Type-3 paired SITL campaign failed the declared paired gates,
particularly roll/pitch tracking and torque variation
(`astsmc_mode3_final_validation.md`). Later real-airframe bring-up supersedes
that report's historical “SITL-only” status, but includes serious adverse and
rejected trials. The allocator-aware image currently has build, upload,
parameter-readback, and props-off evidence; controlled flight remains pending
(`uav985_real_flight_readiness.md:573-591`).

This plan is not a flight authorization.

### Document-control labels

- **ACTIVE** — current plan or conclusion.
- **HISTORICAL** — accurate for its date but superseded by later evidence.
- **SUPERSEDED** — retained to make corrections auditable; do not execute.
- **UNVERIFIED HYPOTHESIS** — a proposition that must pass a stated gate.

---

## 1. Prior art and the exact candidate gap

### 1.1 Four ingredients are already taken

| Ingredient | Prior art | Active conclusion |
|---|---|---|
| Proper-implicit scalar STA, without and with actuator saturation | Seeber & Andritsch, *Automatica* 173 (2025), Eq. (17)–(18), arXiv:2406.16094 | Type 3 implements a published scalar kernel. |
| Multivariable Euler-implicit sliding-mode generalized equations, solution algorithm, and hardware demonstration | Mojallizadeh et al., *ISA Transactions* 147 (2024), 140–152, DOI `10.1016/j.isatra.2024.01.031` | Multivariable implicit solving is not the contribution. |
| Dynamic constrained control allocation as a quadratic program | Härkegård, *Journal of Guidance, Control, and Dynamics* 27(6) (2004), 1028–1034, DOI `10.2514/1.11607` | A weighted constrained allocator is not the contribution. |
| Allocation-aware anti-windup using realized/allocation-deficit information | Hanus/anti-windup and control-allocation literature; PX4 PID saturation gating; proposition-level full-text check still required | The broad anti-windup idea is not the contribution. The narrower one-command-delayed Type-3 state update is a repository implementation whose exact external precedent remains an open literature-ledger item. |

Additional narrowing prior art includes implicit SMC hardware experiments,
variational/vector super-twisting, adaptive/barrier STA, and STA combined with
control allocation in fault-tolerant settings. These must be proposition-level
checked in the literature ledger before submission.

### 1.2 What Type 3 actually is

`RateControl::implicitSuperTwistingStep()` at
`src/lib/rate_control/rate_control.cpp:1173-1204` is locally faithful to Seeber
& Andritsch Eq. (18) under the PX4 sign mapping

```text
q = -v,    r = -u,
```

because PX4 uses `s = rate_sp - rate`, so positive residual torque enters
`ds/dt` with the opposite sign. Local scalar-kernel fidelity does not transfer
the paper's theorem to the full Type-3 loop.

The full implementation additionally contains:

- variable and rejected timing intervals;
- jerk-limited reference shaping and raw-reference blending;
- nominal/residual torque partitioning and gyroscopic compensation;
- scheduled gains and dynamic authority;
- actuator lag, slew, and command projection;
- trim learning, selective state release, state recovery, and quiet anchoring;
- variation regularization;
- priority allocation and one-command-delayed allocator feedback.

Those mechanisms are practical wrappers outside the scalar theorem and require
individual ablation. They must not be presented as one proven extension.

### 1.3 Current allocator conditioning is delayed, not coupled

The current Type-3 path already does the engineering correction an earlier
version of this document proposed:

- on a fresh roll/pitch allocation miss, allocator-modeled allocated torque
  becomes the prior actuator observation
  (`rate_control.cpp:1391-1396`);
- the later integral-state update is conditioned toward the modeled allocated
  residual torque (`rate_control.cpp:1550-1593`);
- feedback is one-command delayed and freshness/epoch gated;
- yaw retains its separate authority-governor path;
- the current command and the scalar Equation-18 helper are not overwritten.

This is a delayed external anti-windup/state-conditioning wrapper. It is **not**
a simultaneous solution of Equation (17)/(18) with the allocator.

### 1.4 The one candidate gap

**UNVERIFIED HYPOTHESIS.** No publication identified by the documented searches
as of 2026-07-27 places a priority-constrained allocation optimality system
inside the same proper-implicit multivariable STA generalized equation and then
establishes a materially equivalent well-posedness and infeasible-allocation
closed-loop result.

The candidate Type-4 distinction is therefore exactly one composition step:

> solve the implicit STA state and constrained allocation together so the state
> update uses the same-sample modeled realizable torque associated with the
> actuator vector returned by that solve.

Composition by itself is not enough. The contribution must be a nonroutine
result for the coupled operator—most plausibly a practical closed-loop bound
under allocation infeasibility with explicit finite-priority-weight constants.

### 1.5 Torque terminology

For Type 4, define the full modeled control wrench

```text
w_model,k = A_k f_k = [tau_model,k; F_model,k],
```

where the torque block `B_tau,k` and force block `B_F,k` satisfy

```text
tau_model,k = B_tau,k f_k,    F_model,k = B_F,k f_k.
```

`w_model,k` is the **modeled realizable wrench**. Its components are modeled
allocated torque and modeled allocated force, not measured or physically
achieved quantities. `A_k` is an estimated effectiveness map, while ESC, motor,
propeller, airframe, battery, and aerodynamic dynamics separate the command
from physical response.

The body-rate STA state couples to `tau_model,k`; thrust/force remains an
allocation objective and constraint that competes for the same actuators.
Reserve “physical torque/force” for measured or inferred airframe response.
Apply this terminology consistently in equations, messages, diagnostics,
plots, and publication text.

### 1.6 Scope of `a509.pdf`

Bouabdallah and Siegwart's 2005 paper supplies quadrotor-model, cascaded-control,
and attitude-angle SMC lineage relevant primarily to Mode 2. Its experiments
used a mechanically constrained 3-DOF bench, not free flight. It is not prior
art for proper-implicit STA, Type 3, or allocator-coupled implicit control.

The existing measured result that the integral-free paper-strict Mode-2 surface
cannot hold the real airframe's trim is a Track-A experimental finding, not a
Type-4 novelty argument.

---

## 2. Permanent controller identities

| `MC_RATE_CTRL_T` | Meaning | Identity rule |
|---:|---|---|
| 0 | PID | Existing behavior unchanged. |
| 1 | Existing Type 1 | Existing behavior unchanged. |
| 2 | Model-based SMC | Existing behavior unchanged. |
| 3 | Legacy ASTSMC | Permanent meaning for all existing cards, ULogs, manifests, and replay fixtures. |
| 4 | PC-PISTA research controller | Separate parameters, acknowledgment, diagnostics, tests, and evidence. |

Never reinterpret historical Type-3 artifacts as Type 4. Never silently migrate
a Type-3 card. Type 4 receives a complete, fail-closed, versioned card and a
separate runtime identity.

---

## 3. Track A — publishable work without algorithmic novelty

Track A is an independent destination and can proceed even if every Type-4 gate
fails.

### 3.1 Defensible outputs

1. A reproducible experimental comparison of PID, model-based SMC, and
   proper-implicit STA on identical hardware, once each included controller
   passes its own trust gate.
2. An implementation note explaining the one-command-delayed allocator-state
   mismatch and the bounded delayed conditioning used by legacy Type 3.
3. Experimental characterization of the Mode-2/a509 integral-free surface's
   inability to reject the measured constant trim disturbance, including the
   restored-integral comparison.
4. Open firmware, parameter-card, ULog, and analysis artifacts with immutable
   provenance.

### 3.2 Claims Track A must avoid

- Type 3 is a new algorithm.
- Equation (18), implicit saturation conditioning, or allocator anti-windup was
  invented here.
- A locked bench is flight validation.
- A single successful run establishes a feasible region or superiority.
- Installed `J/EFF` equality means the physical model is fully identified.

---

## 4. Phase 0 — freeze current evidence

Before Type-4 development, create a baseline manifest containing:

- Git SHA and clean/dirty state;
- target board, firmware-reported revision, and binary SHA-256;
- complete parameters, card hash, generator revision, and exact invocation;
- ULog filenames, dates, immutable hashes, controller identity, and storage;
- analyzer revision, invocation, schema, outputs, and output hashes;
- inclusion/exclusion ledger and known failed gates;
- evidence class: SITL, HIL, locked bench, props-off, restrained flight, or
  free flight.

Use only these reproducibility terms:

- **binary-identical** — equal artifact bytes;
- **deterministic replay-identical** — identical recorded inputs, timestamps,
  initial state, numerical environment, and output bytes;
- **trajectory-equivalent** — output within predeclared absolute/relative
  tolerance over a named profile.

Default-off parameters do not prove binary or replay identity.

Create an annotated baseline tag only after the manifest and corresponding
evidence are committed and reconciled with the binary and logs. Do not backdate
or reuse a tag for later flights.

**Exit criterion:** immutable baseline manifest exists and the current Type-3
status/adverse-event ledger is complete.

---

## 5. Phase 1 — cheapest Type-4 kill gates first

All gates below close before production integration.

### 5.1 Exact literature-gap review

Perform a full-text, proposition-level search for same-sample coupling of
implicit SMC/STA state and constrained allocation. Mandatory anchors include:

- Seeber & Andritsch, saturated scalar Equation (18);
- Mojallizadeh et al., ISA Transactions 2024, DOI
  `10.1016/j.isatra.2024.01.031`;
- implicit-SMC hardware experiments and the implicit-method tutorial;
- variational/vector super-twisting work;
- Härkegård 2004, DOI `10.2514/1.11607`;
- STA plus allocation/fault-tolerant allocation;
- forward/backward citations, theses, patents, and laboratory pages.

For every search, record database, access date, exact query, filters, result
count, full-text status, inclusion/exclusion decision, and proposition
supported.

If no local independent controls reviewer is available, use two distinct
external-challenge artifacts:

1. **Literature challenge:** publish the exact coupling question and draft
   generalized equation as a dated public technical note, then send a concise
   proposition-level prior-art question to authors with directly relevant
   frameworks, prioritizing Seeber, Brogliato, Acary, Plestan, and Härkegård or
   current collaborators.
2. **Theorem challenge:** after Phase 3, publish the theorem statements,
   assumptions, operator construction, proof, and counterexample search as a
   preprint or technical appendix, and request explicit attempts to refute the
   result from the same authors/community.
3. Archive responses and lack of response, but never treat silence as support.
4. Request an open technical seminar/review for both artifacts and preserve
   objections, counterexamples, revisions, and responses.

**Stop condition:** equivalent same-sample coupling with materially equivalent
well-posedness and infeasible-allocation results is found. Continue as Track A.

### 5.2 Weighted versus strict priority

Strict lexicographic priority is not the default. Its active-level solution map
may be discontinuous at priority switches. A strongly convex tie-breaker gives
uniqueness within a level but does not establish continuity across levels or
existence of the coupled fixed point.

Analyze two formulations:

1. strict lexicographic hierarchy — strongest semantics, potentially no usable
   fixed-point theorem;
2. one strongly convex weighted soft-priority problem — improved operator
   regularity, approximate priorities.

Use the weighted formulation unless a rigorous lexicographic treatment is
obtained. Required weighted results are:

- finite positive weights produce a unique allocation response under stated
  assumptions;
- finite-weight priority errors have explicit constants;
- convergence to a lexicographic solution as weight ratios grow is claimed only
  if proved under stated compactness/regularity assumptions.

“Large weights” alone do not prove lexicographic behavior.

### 5.3 Precommit the nontrivial theorem target

Write a one-page theorem-risk memo before derivation. The primary bet is:

> A practical sampled-data sliding/tracking bound under persistent allocation
> infeasibility, explicitly parameterized by finite priority weights,
> allocation residual, `B/J` uncertainty, actuator lag, timing error, and
> numerical residual.

Secondary candidate contributions are:

- a nonstandard well-posedness argument if coupling through the full wrench map
  `A_k` and its torque block `B_tau,k` prevents a direct monotone-operator
  corollary;
- explicit finite-weight priority constants and, if valid, asymptotic
  lexicographic recovery.

Predeclare the downgrade rule:

- if existence/uniqueness is a routine short corollary and there is no new
  infeasible-case or finite-weight bound, Track B becomes an engineering
  composition/theory note—not a novel-controller paper;
- if strict lexicographic coupling admits a useful impossibility result, that
  may be publishable theory but does not authorize Type 4 for flight.

### 5.4 On-target timing microbenchmark

Before a full proof or production integration, implement a disposable
quadrotor-sized benchmark with:

- four actuator variables and the intended effectiveness dimensions;
- actuator box and one-step reachable/slew constraints;
- weighted priority residuals;
- warm-started fixed-size deterministic solver;
- feasible, saturated, near-degenerate, and rapidly switching cases.

Run it on the actual FMU-v6C/H7 at the intended loop rate. Measure median, p95,
p99, and worst-case time; iterations; stack; RAM; and flash delta. Define the
runtime gate from measured work-queue and interrupt margin. A provisional
starting target is worst-case solver time below 20–25% of the minimum accepted
gyro period, not a final safety threshold.

**Stop condition:** no bounded execution margin, excessive memory/flash cost,
or nondeterministic/unbounded iteration behavior. Continue as Track A.

---

## 6. Phase 2 — mathematical Type-4 specification

Write the controller independently of PX4 APIs.

### 6.1 Plant, state, and uncertainty

Define:

- body-rate sliding variable and sign convention;
- inertia, gyroscopic term, full wrench-effectiveness matrix `A_k`, torque block
  `B_tau,k`, force block `B_F,k`, and matched/unmatched uncertainty;
- fixed-step theorem model and separately bounded flight jitter;
- actuator box, slew, and one-step reachable constraints;
- actuator lag and command-to-physical-response uncertainty;
- rank and conditioning assumptions on the relevant blocks of `A_k`;
- which quantities are measured, inferred, estimated, or provisional.

Do not transfer scalar or ISA-2024 results automatically to the coupled PX4
system.

### 6.2 Weighted allocation operator

Use one strongly convex objective, provisionally

```text
min_f  sum_i rho_i ||W_i (A_k f - w_des)_i||^2
       + mu ||f - f_prev||^2,

w_des = [tau_des; F_des].
```

subject to current actuator and one-step reachable bounds.

Requirements:

- `mu > 0`, or another precise condition, gives a unique actuator selection;
- roll/pitch, force/thrust, and yaw residual groups and finite weight ratios are
  explicit;
- units and scaling of every weight are defined;
- residuals remain observable outputs rather than hidden desaturation;
- finite-ratio priority error is bounded.

Sequential desaturation remains a baseline, not the theorem operator.

### 6.3 Same-sample generalized equation

Define one problem whose unknowns include at least

```text
(s_{k+1}, q_{k+1}, f_k)
```

and whose equations include:

- multivariable proper-implicit STA inclusion at the implicit state;
- modeled realizable wrench `w_model,k = A_k f_k`, whose torque block is
  `tau_model,k = B_tau,k f_k`;
- the current force/thrust setpoint `F_des,k`, synchronized to the same sample
  and included in `w_des,k`;
- weighted allocation KKT inclusion;
- normal cones for actuator/reachable constraints;
- explicit nominal and reference terms.

The Type-4 `q` update uses `B_tau,k f_k` returned by this same numerical solve.
The allocator objective uses the full synchronized wrench. Delayed
`control_allocator_status` remains monitoring/model-error evidence, not a term
in the new mathematical law.

### 6.4 Required reductions

Prove and independently test:

1. scalar direct actuation with symmetric saturation reduces to Seeber &
   Andritsch Equation (18);
2. unconstrained feasible allocation reduces to the multivariable implicit law
   plus the unique minimum-change actuator selection;
3. finite weighted infeasibility satisfies the stated priority/error bound;
4. weighted solutions approach strict lexicographic allocation only if the
   theorem assumptions establish it;
5. a decoupled form exists for the primary D-versus-E ablation.

---

## 7. Phase 3 — theorem and inexact-solve contract

Establish, under explicit assumptions:

1. existence for every admissible sample and nonempty reachable actuator set;
2. uniqueness or deterministic selection of actuator command, modeled torque,
   and next controller state;
3. the exact operator regularity used by the proof—do not assume that coupling
   preserves monotonicity;
4. feasible-case convergence or an explicit sampled-data practical band;
5. the precommitted infeasible-case bound with finite-weight constants;
6. robustness to `B/J` error, actuator lag, bounded jitter, and numerical
   residual;
7. recursive feasibility through startup, switching, and matrix/bound changes.

### 7.1 Inexact solving is not delayed inconsistency

Separate **coupling consistency** from **allocation optimality**:

- if firmware applies exactly the returned `f_hat,k` and updates Type 4 with
  exactly `B_tau,k f_hat,k`, same-sample model consistency survives early
  solver termination;
- inexactness degrades allocation optimality, priority residuals, and the
  closed-loop bound; it does not recreate the one-command-delayed mismatch;
- the theorem must parameterize performance by the reported generalized-
  equation/KKT residual and finite iteration budget;
- if clipping, slew, motor-failure masking, or effectiveness callbacks alter
  `f_hat,k` after solving, the state update must use the exact post-processed
  vector's modeled torque in the same cycle, or consistency is lost.

This is a structural property to prove and test, not an excuse to ignore solver
quality.

**Go/no-go:** without a nonroutine infeasible-case/finite-weight result, do not
claim a novel Type-4 controller paper.

---

## 8. Phase 4 — predicted effect-size kill gate

The coupling removes a one-sample model inconsistency, while the physical
actuator response may span tens of samples. A correct theorem can therefore
produce an effect too small to measure on this vehicle.

Before HIL or production integration, compare:

- **D:** decoupled implicit STA followed by the identical weighted constrained
  allocator. Its STA state is updated at sample `k+1` using the fresh,
  epoch-matched modeled allocated torque from command `k`; if that feedback is
  missing, its predeclared hold policy applies. This is the one-command-delayed
  baseline.
- **E:** coupled Type 4. Its state update at sample `k` uses the modeled torque
  block of the actuator vector returned by that same solve.

D and E must otherwise use identical plant model, references, gains, weights,
constraints, actuator post-processing, numerical budget, initialization, and
fault policy. This definition is required so their only intended difference is
the timing/coupling of modeled allocated torque in the implicit state update.

Use a calibrated actuator-lag model over the intended operating range, with
uncertainty covering the observed/estimated response and sensor/model
repeatability. Enter controlled allocation infeasibility through a virtual
actuator envelope.

Predeclare one or two coupling-sensitive primary endpoints, for example:

- post-release rate error and integral-state mismatch;
- higher-priority tracking loss during infeasibility.

Measure the hardware noise/repeatability floor from repeated baseline trials and
define a smallest practically meaningful effect.

**Stop condition:** if the predicted D-versus-E difference across calibrated
lag/model uncertainty is below the hardware repeatability floor or smallest
meaningful effect, stop before HIL. The result may remain theory-only; do not
claim flight validation is warranted.

---

## 9. Phase 5 — production architecture after research gates

### 9.1 Solver ownership

`ControlAllocator` owns the current effectiveness matrix, actuator bounds,
motor-failure state, and dynamic clipping. Execute the coupled solve in or
immediately beside that owner, not upstream using a stale matrix copy.

Create a pure fixed-size solver library under `src/lib/control_allocation/`
with:

- no uORB or parameter access in the mathematical kernel;
- deterministic bounded-memory warm start;
- fixed maximum iterations;
- explicit generalized-equation/KKT residual;
- iterations, active constraints, modeled torque, finite-weight residuals, and
  timing diagnostics.

The exact vector sent downstream must be the vector used to calculate the
Type-4 state-update torque. Put every post-processing constraint inside the
reachable set or perform an immediate same-cycle consistency update using the
final vector.

### 9.2 Request/result interfaces

Add a dedicated Type-4 request carrying:

- exact sample timestamp and controller epoch;
- implicit-controller inputs and reference/model terms;
- the force/thrust setpoint synchronized to the same sample, with an explicit
  stale/missing-setpoint policy;
- gains and priority configuration;
- lifecycle/reset/switch flags;
- configuration version/hash.

The allocator rejects stale or mismatched requests, runs the joint solve, owns
`q`, and publishes independent Type-4 diagnostics.

Keep ordinary `vehicle_torque_setpoint` behavior unchanged for Types 0–3.
There is no hidden in-air fallback. Define a bounded hold/rollback action, latch
a visible runtime fault, and use the independently verified PID recovery path.

### 9.3 Minimal theorem controller

Do not import legacy Type-3 gain scheduling, trim substitution, selective
release, quiet anchoring, variation regularization, residual-authority
scheduling, or yaw governor by default. A mechanism enters Type 4 only if it is:

- included in the formal law and proof; or
- added later as a separately gated, explicitly non-theorem ablation.

---

## 10. Phase 6 — verification ladder

### 10.1 Independent oracle and property tests

Build a high-precision offline oracle and cover:

- scalar Equation-18 reduction;
- feasible and unconstrained cases;
- deliberate weighted-priority infeasibility;
- box and slew constraints;
- active-set/priority-transition neighborhoods;
- rank-deficient and near-singular matrices;
- effectiveness/motor-failure changes;
- branch and constraint boundaries;
- inexact termination at multiple residual levels;
- post-processing consistency;
- deterministic repeated solutions.

Verify bounds, reduction identities, KKT/generalized-equation residuals,
finite-weight inequalities, use of the final actuator vector, and finite
outputs.

### 10.2 C++ and module integration

Test:

- embedded solver versus oracle;
- exact timestamp/epoch contract;
- reset, seed, and mode switching;
- invalid timing and stale requests;
- matrix changes and motor failures;
- bounded timeout and runtime-fault latching;
- unchanged Types 0–3;
- complete diagnostics and time alignment.

### 10.3 Closed-loop simulation matrix

| ID | Controller/allocation configuration |
|---|---|
| A | PID + configured allocator |
| B | Mode 2 + configured allocator, only after its trust gate |
| C | Legacy Type 3 + sequential desaturation |
| D | Decoupled implicit STA + weighted constrained allocator; state update uses command `k` modeled allocation at `k+1` |
| E | Coupled Type 4 PC-PISTA; state update uses same-solve modeled allocation at `k` |

D versus E is the novelty and effect-size ablation. Except for this defined
coupling/timing distinction, use identical model, reference, initialization,
gains, weights, limits, actuator post-processing, numerical budget, and fault
policy. Sweep inertia/effectiveness error,
actuator lag, battery state, jitter, sensor noise, CG/wind disturbance, virtual
derating, and numerical residual.

---

## 11. Phase 7 — experiments that activate the contribution

D and E differ principally during allocation infeasibility. Benign evidence
with little clipping cannot validate the claim.

Create a dedicated research facility that imposes a **virtual actuator/wrench
envelope inside a conservatively identified physical one-step reachable set**.
Static nesting alone does not prove PID recoverability. Before flight, identify
and validate the physical reachable set over the admitted battery, actuator-lag,
slew, thrust-demand, model-error, and mode-transfer state envelope. Then choose
the virtual set so a predeclared minimum physical reserve remains for the PID
recovery maneuver. D and E must use the identical virtual reachable set.

Validation order:

1. offline/SITL synthetic infeasibility;
2. HIL virtual derating and combined-axis demand;
3. locked 3-DOF bench for priority characterization—always labeled bench;
4. on-target shadow computation without applying Type 4;
5. props-off application checks, including worst-case transfer-state replay
   into PID and verification of the predeclared reachable-set reserve;
6. authorized low-energy free flight with conservative virtual limits only
   after the PID recovery-margin gate passes across the admitted battery,
   thrust, lag, and state-transfer envelope;
7. only then natural or higher-demand allocation conflicts.

Do not initially validate by driving physical motors to their actual bounds.
Virtual-envelope activation is a test condition, not by itself a safety proof.

Primary measurements include:

- weighted and axis-specific allocation residuals;
- degradation of higher-priority axes due to lower-priority demand;
- modeled-torque consistency used by the Type-4 state update;
- post-release tracking and state recovery;
- actuator and slew occupancy;
- numerical residual, iterations, timing distribution, and deadline margin;
- motor-command and body-rate/error high-frequency content;
- model-versus-observed response, energy, and safety events.

The key result must compare the precisely defined D and E baselines in the same
controlled infeasible region and test whether E satisfies the theorem's bound
while D exhibits the predicted one-command-delayed state effect. If another
implementation difference remains, the result cannot attribute the effect to
the coupling.

---

## 12. Hardware dependency graph

1. **PID recovery controller.** Independently known-good; mode switch, kill
   switch, and failsafes verified before any Type-4 flight. The recovery gate
   must also demonstrate a quantified minimum reachable-wrench reserve and
   successful transfer from the worst admitted Type-4 actuator/state condition
   across the allowed battery, thrust, actuator-lag, slew, and model-error
   envelope.
2. **Mode 2.** Must independently pass hover, trim, recovery, card/readback, and
   repeatability gates before inclusion in a free-flight comparison. Until then
   it is bench-only and is never the safety dependency.
3. **Legacy Type 3.** Controlled allocator-aware flight remains pending. Its
   adverse-event ledger informs stop rules but does not authorize Type 4.
4. **Type 4.** Literature, nontriviality, timing, predicted-effect-size,
   theorem, oracle, SITL/HIL, shadow, and props-off gates pass in order.

If Mode 2 remains unreliable, the first free-flight comparison is PID versus D
versus E. Defer the PID/Mode-2/Type-4 campaign.

For every required independent review, name a reviewer before the gate starts.
If no reviewer accepts, use the two-artifact literature and theorem challenge
procedure in §5.1. The theorem artifact must expose the assumptions and proof,
not only the architecture or coupling question. Silence is not approval.

---

## 13. Safety and confirmatory campaign

| Gate | Requirement |
|---|---|
| S0 | Literature, theorem, code, model uncertainty, adverse-event, and stop-rule review |
| S1 | Repeated SITL/HIL, fault injection, and virtual infeasibility |
| S2 | On-target shadow timing, numerical residual, and effect-size confirmation |
| S3 | Props-off firmware/card readback, actuator signs/order, physical reachable-set identification, quantified PID reserve, worst-transfer-state recovery, virtual envelope, switching, kill/failsafe |
| S4 | Lowest-energy free flight beginning and ending in PID; small isolated commands; review every log |
| S5 | Controlled combined-axis virtual infeasibility, one change at a time |
| S6 | Frozen randomized/counterbalanced confirmatory campaign |

Stop for:

- numerical residual above the certified bound or deadline miss;
- stale/mismatched request or nonfinite state;
- loss of same-cycle model consistency after actuator post-processing;
- unplanned constraint activity or priority-bound violation;
- physical motor-bound activity outside the virtual-envelope contract;
- growing oscillation, attitude/thrust excursion, estimator degradation,
  failsafe, or pilot intervention.

Flights or matched blocks are the independent experimental unit. Five
repetitions are pilot data only. Determine confirmatory sample size from pilot
variance, a predeclared smallest effect of interest, and the selected
superiority/non-inferiority/equivalence hypothesis. Freeze primary endpoints,
margins, exclusions, processing, and multiplicity rules before confirmatory
data collection.

---

## 14. Publication decision tree

- **Literature gate passes + nonroutine theorem passes + D/E effect is measurable
  + safety/flight gates pass:** claim a new coupled algorithm and its bounded
  flight evaluation.
- **New formulation but only routine existence or partial theory:** publish as
  an engineering composition or theory note with explicit limitations.
- **Equivalent prior art is found:** Track A replication/open PX4 benchmark.
- **D versus E is null or worse:** negative result or boundary-of-applicability
  study.
- **Effect is below hardware repeatability:** theory-only result; no flight-
  validation claim.
- **Safety or repeatability fails:** no performance-paper claim.

Open firmware, cards, ULogs, and analysis add reproducibility value; they do not
create algorithmic novelty.

Do not use “first,” “novel,” “well-posed,” “finite-time,” “guaranteed,” or
“flight-validated” in active publication text until the corresponding gate has
closed with auditable evidence.

---

## 15. Superseded Type-3 novelty proposals

**SUPERSEDED — retained as an audit trail. Do not execute as the Type-4 plan.**

Earlier versions proposed making Type 3 novel by:

1. inserting allocator feedback into scalar Equation-18 conditioning;
2. replacing scheduled `K1` with a capped rational barrier gain;
3. claiming first implicit-STA multirotor flight validation.

Corrections:

- allocator feedback is delayed and external; broad allocation-aware anti-
  windup is prior art, while the exact precedent for Type 3's one-command-
  delayed modeled-torque state correction remains a proposition-level search
  item; legacy Type 3 already implements that bounded correction;
- `K_b(sigma) = epsilon sigma/(zeta-sigma)` plus a finite hardware cap does not
  prove forward invariance for the discrete, saturated, delayed, variable-step
  PX4 loop; gain-explicit/control-implicit evaluation is not a joint implicit
  gain solve;
- `lambda = k2-k1^2/4 >= 0` must not be called a theorem requirement without a
  precise source result;
- a “first flight” claim rests on a negative literature search and cannot be
  activated without the §5.1 protocol;
- even if the flight gap survives, that is an application contribution, not a
  new Type-3 algorithm.

These ideas may remain Track-A engineering ablations after safety review, but
they cannot carry the algorithmic novelty thesis.

---

## 16. Final acceptance checklist

- [ ] `MC_RATE_CTRL_T=3` permanently means legacy ASTSMC.
- [ ] `MC_RATE_CTRL_T=4` uniquely identifies PC-PISTA across code, cards, logs,
      manifests, and plots.
- [ ] The literature ledger narrows novelty to same-sample coupling and finds no
      equivalent theorem.
- [ ] The theorem-risk memo precommits to a nonroutine infeasible-case or
      finite-weight result.
- [ ] Routine existence alone automatically downgrades Track B.
- [ ] Weighted priority semantics and finite-weight constants are explicit.
- [ ] Strict lexicographic discontinuity is not hand-waved away by strong
      convexity.
- [ ] `A_k f_k` is consistently called modeled realizable/allocated wrench;
      its `B_tau,k f_k` block is modeled torque, never measured achieved torque.
- [ ] Scalar Equation-18 and decoupled reductions are proved and oracle-tested.
- [ ] On-target time, stack, RAM, and flash pass before production integration.
- [ ] Calibrated-lag D-versus-E effect exceeds the hardware repeatability floor
      before HIL.
- [ ] The exact returned/post-processed actuator vector determines the same-
      cycle state-update torque, including under inexact solving.
- [ ] Deliberate virtual infeasibility activates the claimed regime only after
      a conservative physical reachable set, minimum PID reserve, and worst-
      transfer-state recovery have been quantified and verified.
- [ ] Locked bench evidence is never called flight evidence.
- [ ] PID recovery is verified; Mode 2 enters free-flight comparison only after
      its independent trust gate.
- [ ] A named reviewer or the documented two-artifact public challenge is used
      separately for the literature gap and the complete theorem/proof.
- [ ] Failure of literature, nontriviality, timing, effect-size, theorem,
      ablation, or safety gates reverts to Track A without relabeling.

---

## 17. Citation and evidence ledger seed

| Reference/evidence | Access level as of 2026-07-27 | Proposition currently used |
|---|---|---|
| Seeber & Andritsch, arXiv:2406.16094 / *Automatica* 173 (2025) | Full equation structure checked | Scalar proper-implicit conditioned STA and saturation are prior art. |
| Mojallizadeh et al., DOI `10.1016/j.isatra.2024.01.031` | Bibliographic record and reported scope checked; full text required for theorem mapping | Multivariable Euler-implicit generalized-equation solving and 6×6 hardware are prior art. |
| Härkegård, DOI `10.2514/1.11607` | Bibliographic record and reported scope checked; full text required for exact comparison | Dynamic constrained-QP allocation is prior art. |
| Hanus/conditioning and allocation-aware anti-windup sources | Recalled/search-level only; exact references and propositions must be added in §5.1 | Broad conditioning on realized-versus-demanded control is probably prior art; the exact Type-3 delayed modeled-torque update is not yet claimed as externally published. |
| PX4 PID saturation gating (`rate_control.cpp:1717-1728`) | Repository full text | PX4 PID gates integration from allocator saturation signs; it is not equivalent evidence for Type 3's modeled-torque state update. |
| `astsmc_mode3_final_validation.md` | Repository full text | No validated final Type-3 card; paired gates failed. |
| `uav985_real_flight_readiness.md` | Repository full text | Later adverse-flight chronology and allocator-aware props-off status. |
| `PX4_SMC_Implementation_Tutorial.md` | Repository full text | Current delayed allocator-conditioning behavior and Mode-2/Type-3 evidence chronology. |
| `a509.pdf` | Full paper checked | Mode-2/model/cascaded attitude-SMC context; locked bench, not Type-3 prior art. |

Before submission, expand this table into the reproducible search ledger required
by §5.1 and rerun forward-citation searches immediately before finalizing any
negative-literature statement.
