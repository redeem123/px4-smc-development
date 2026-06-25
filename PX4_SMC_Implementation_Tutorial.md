# PX4 Sliding Mode Rate Controller: End-to-End Implementation Tutorial

This tutorial documents the complete workflow for implementing, tuning, and validating a custom **Sliding Mode Controller (SMC)** in the PX4 multicopter rate loop, running alongside the stock PID controller with a single parameter switch in QGroundControl. The final validated configuration corresponds to flight log **`11_58_58.ulg`**: pure SMC flight with deployment safeguards active and the safety watchdog disabled for uninterrupted research testing.

---

## Overview

| Milestone | Flight log | Controller | Outcome |
|-----------|------------|------------|---------|
| PID baseline | `10_06_52.ulg` | PID (`MC_RATE_CTRL_T = 0`) | Reference tracking performance |
| Untuned SMC | `11_23_11.ulg` | SMC, default gains | Severe chattering detected |
| Tuned SMC | `11_25_48.ulg` | SMC, tuned gains | Chattering suppressed (~95% reduction) |
| Watchdog test | `11_54_51.ulg` | SMC → PID fallback | Safety watchdog verified |
| **Final working state** | **`11_58_58.ulg`** | **Pure SMC + safeguards** | **Smooth actuators, stable tracking** |

Logs are stored under:

```
~/Desktop/px4/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/*.ulg
```

---

## Phase 1 — Bootstrap PX4 on macOS

### 1.1 Clone the repository

From an empty workspace directory (e.g. `~/Desktop/px4`):

```bash
git clone --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git .
```

This fetches the main PX4 flight stack and all required Git submodules (NuttX, simulator bridges, etc.).

### 1.2 Install development dependencies

Run the official macOS setup script:

```bash
bash Tools/setup/macos.sh
```

For jMAVSim simulation support, pass the simulator tools flag:

```bash
bash Tools/setup/macos.sh --sim-tools
```

The script installs Homebrew dependencies, including taps such as `px4/px4` and `osx-cross/arm`.

### 1.3 Toolchain requirements

The following tools must be present after setup:

- `cmake`
- `ninja`
- `arm-none-eabi-gcc` (version 13.2.1 was used during validation)

### 1.4 Python virtual environment

Create and populate a virtual environment for PX4 Python tooling:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r Tools/setup/requirements.txt
```

Key packages include `pyulog` (log analysis) and `pymavlink`.

### 1.5 QGroundControl

Install the QGroundControl desktop application:

```bash
brew install --cask qgroundcontrol
```

The application installs to `/Applications/QGroundControl.app`. QGroundControl discovers custom PX4 parameters automatically via MAVLink; no QGC source modifications are required.

---

## Phase 2 — Implement the SMC Rate Controller

### 2.1 Design objective

The implementation adds a **parallel** SMC path in the rate control library. A single integer parameter selects the active controller at runtime:

| `MC_RATE_CTRL_T` | Active controller |
|------------------|-------------------|
| `0` | Standard PID (unchanged) |
| `1` | Sliding Mode Controller (pure SMC — no internal PID fallback) |

When SMC is selected, the PID branch is bypassed entirely inside `RateControl::update()`.

### 2.2 Control law

For each axis \(i \in \{\text{roll}, \text{pitch}, \text{yaw}\}\):

**Tracking error:**

\[
e_i = \omega_{sp,i} - \omega_i
\]

**Integral sliding surface** (eliminates steady-state offset):

\[
s_i = e_i + c_i \int e_i \, dt
\]

**Torque output:**

\[
\tau_i = K_{ff,i} \cdot \omega_{sp,i} + K_{eq,i} \cdot e_i + \eta_i \cdot \mathrm{sat}\!\left(\frac{s_i}{\Phi_i}\right) + K_{s,i} \cdot s_i
\]

Where:

| Symbol | Parameter prefix | Role |
|--------|------------------|------|
| \(c_i\) | `MC_SMC_C_*` | Sliding surface integral coefficient |
| \(\eta_i\) | `MC_SMC_ETA_*` | Switching gain |
| \(\Phi_i\) | `MC_SMC_BND_*` | Boundary layer thickness (chattering suppression) |
| \(K_{s,i}\) | `MC_SMC_KS_*` | Linear reaching gain |
| \(K_{eq,i}\) | `MC_SMC_KEQ_*` | Equivalent control gain |

The saturation function clamps its argument to \([-1, 1]\).

### 2.3 Files to modify

| File | Purpose |
|------|---------|
| `src/modules/mc_rate_control/mc_rate_control_params.yaml` | Declare all SMC parameters for QGroundControl |
| `src/modules/mc_rate_control/MulticopterRateControl.hpp` | Bind parameters via `DEFINE_PARAMETERS` |
| `src/modules/mc_rate_control/MulticopterRateControl.cpp` | Read parameters in `parameters_updated()` and pass to `_rate_control` |
| `src/lib/rate_control/rate_control.hpp` | SMC state variables, setters, `updateSMC()` declaration |
| `src/lib/rate_control/rate_control.cpp` | Branch in `update()`, SMC math, integral anti-windup |

#### Parameter definitions (`mc_rate_control_params.yaml`)

Append the following parameter group (defaults shown):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `MC_RATE_CTRL_T` | int32 | `0` | Controller selector: 0 = PID, 1 = SMC |
| `MC_SMC_C_R` / `P` / `Y` | float | `1.0` | Sliding surface integral gain |
| `MC_SMC_ETA_R` / `P` / `Y` | float | `0.1` | Switching gain |
| `MC_SMC_BND_R` / `P` / `Y` | float | `0.1` | Boundary layer thickness |
| `MC_SMC_KS_R` / `P` / `Y` | float | `0.2` | Reaching gain |
| `MC_SMC_KEQ_R` / `P` / `Y` | float | `0.15` | Equivalent control gain |

#### Rate control library (`rate_control.hpp` / `rate_control.cpp`)

Key implementation points:

- Add `_controller_type`, SMC gain vectors, and `_smc_rate_int` state.
- Expose `setControllerType(int)` and `setSmcGains(...)`.
- In `update()`: if `_controller_type == 1`, call `updateSMC()`; otherwise run the existing PID path unchanged.
- In `resetIntegral()`: zero both PID and SMC integrators to prevent transients when switching controllers.
- In `getRateControlStatus()`: log unified status for either controller type.

#### Module wiring (`MulticopterRateControl.cpp`)

Inside `parameters_updated()`:

```cpp
_rate_control.setControllerType(_param_mc_rate_ctrl_t.get());
_rate_control.setSmcGains(
    Vector3f(_param_mc_smc_c_roll.get(), _param_mc_smc_c_pitch.get(), _param_mc_smc_c_yaw.get()),
    Vector3f(_param_mc_smc_eta_roll.get(), _param_mc_smc_eta_pitch.get(), _param_mc_smc_eta_yaw.get()),
    Vector3f(_param_mc_smc_bnd_roll.get(), _param_mc_smc_bnd_pitch.get(), _param_mc_smc_bnd_yaw.get()),
    Vector3f(_param_mc_smc_ks_roll.get(), _param_mc_smc_ks_pitch.get(), _param_mc_smc_ks_yaw.get()),
    Vector3f(_param_mc_smc_keq_roll.get(), _param_mc_smc_keq_pitch.get(), _param_mc_smc_keq_yaw.get())
);
```

### 2.4 Build pitfalls

Two build errors were encountered and resolved during initial compilation:

1. **Integer parameter type:** PX4 requires `type: int32`, not `type: int`, for integer parameters.
2. **Parameter name length:** PX4 limits parameter names to 16 characters. The original name `MC_RATE_CTRL_TYPE` (17 chars) must be shortened to **`MC_RATE_CTRL_T`** (14 chars) across YAML and all C++ bindings.

### 2.5 First build and headless verification

```bash
cd ~/Desktop/px4
source .venv/bin/activate
make px4_sitl none
```

At the `pxh>` shell prompt, verify parameter registration and dynamic switching:

```bash
param show MC_RATE_CTRL_T
param set MC_RATE_CTRL_T 1
param save
```

The rate control task must continue running without crash after switching to SMC.

---

## Phase 3 — Simulation Setup and Baseline Flights

### 3.1 Launch SITL with jMAVSim

```bash
source .venv/bin/activate
make px4_sitl jmavsim
```

Open QGroundControl. It connects automatically to the running simulation via MAVLink.

### 3.2 Switching controllers in QGroundControl

1. Navigate to **Vehicle Setup → Parameters**.
2. Search for `MC_RATE_CTRL_T`.
3. Set to `0` for PID or `1` for SMC.
4. Click **Save** (Write). The switch takes effect immediately at runtime.

All `MC_SMC_*` gain parameters appear in the same parameter list without any QGC modification.

### 3.3 Preflight configuration for SITL

During SITL testing, the following parameter ensures heading estimation and preflight checks pass:

```bash
param set SYS_HAS_MAG 1
param save
```

Without a valid magnetometer reference, arming may fail with *"Preflight Fail: no heading reference"*.

### 3.4 Recommended flight test sequence

Execute flights in this order, saving a log after each:

| Order | Setting | Expected log | Purpose |
|-------|---------|--------------|---------|
| 1 | `MC_RATE_CTRL_T = 0` | `10_06_52.ulg` | PID baseline |
| 2 | `MC_RATE_CTRL_T = 1`, default SMC gains | `11_23_11.ulg` | Identify chattering |
| 3 | `MC_RATE_CTRL_T = 1`, tuned gains (Phase 4) | `11_25_48.ulg` | Validate tuning |
| 4 | Safeguards active, watchdog test (Phase 5) | `11_54_51.ulg` | Verify fallback |
| 5 | Safeguards active, watchdog disabled (Phase 6) | `11_58_58.ulg` | Final working state |

Takeoff can be initiated from QGroundControl (Hold mode + takeoff slider) or from the PX4 shell:

```bash
commander takeoff
commander land
```

---

## Phase 4 — Log Analysis and Gain Tuning

### 4.1 Log analysis script

A Python script parses ULog files and reports controller parameters, RMS tracking error, and a motor chattering metric (variance of motor output derivative):

```bash
.venv/bin/python analyze_log.py build/px4_sitl_default/rootfs/log/2026-06-25/<log_name>.ulg
```

Example output fields:

- `Controller Type (MC_RATE_CTRL_T)`
- Per-axis RMS / max / mean tracking error
- `Motor 0 Derivative Variance` (chattering indicator; values above ~1e4 indicate severe chattering)

### 4.2 Untuned SMC results (`11_23_11.ulg`)

Default SMC gains produced unacceptable performance compared to the PID baseline:

| Metric | PID (`10_06_52.ulg`) | Untuned SMC (`11_23_11.ulg`) |
|--------|----------------------|------------------------------|
| Roll RMS | 1.26 deg/s | 10.93 deg/s |
| Pitch RMS | 0.60 deg/s | 11.80 deg/s |
| Yaw RMS | 0.31 deg/s | 2.20 deg/s |
| Motor chattering var | 1.49e-04 | **2.34** (severe wobble) |

Root cause: the default switching gain (`MC_SMC_ETA = 0.1`) and thin boundary layer (`MC_SMC_BND = 0.1`) produced high-frequency switching in the saturation term.

### 4.3 Tuned gain set (applied before `11_25_48.ulg`)

The following parameter values were written via the PX4 shell and saved:

| Parameter | Roll | Pitch | Yaw |
|-----------|------|-------|-----|
| `MC_SMC_C_*` | 2.0 | 2.0 | 1.0 |
| `MC_SMC_KS_*` | 0.10 | 0.10 | 0.10 |
| `MC_SMC_KEQ_*` | 0.05 | 0.05 | 0.10 |
| `MC_SMC_ETA_*` | 0.02 | 0.02 | 0.02 |
| `MC_SMC_BND_*` | 0.20 | 0.20 | 0.20 |

PX4 shell commands (repeat per axis suffix `_R`, `_P`, `_Y` as needed):

```bash
param set MC_SMC_C_R 2.0
param set MC_SMC_C_P 2.0
param set MC_SMC_C_Y 1.0
param set MC_SMC_KS_R 0.10
param set MC_SMC_KS_P 0.10
param set MC_SMC_KS_Y 0.10
param set MC_SMC_KEQ_R 0.05
param set MC_SMC_KEQ_P 0.05
param set MC_SMC_KEQ_Y 0.10
param set MC_SMC_ETA_R 0.02
param set MC_SMC_ETA_P 0.02
param set MC_SMC_ETA_Y 0.02
param set MC_SMC_BND_R 0.20
param set MC_SMC_BND_P 0.20
param set MC_SMC_BND_Y 0.20
param set MC_RATE_CTRL_T 1
param save
```

### 4.4 Tuned SMC results (`11_25_48.ulg`)

| Metric | Untuned SMC | Tuned SMC | Change |
|--------|-------------|-----------|--------|
| Roll RMS | 10.93 deg/s | 8.43 deg/s | −23% |
| Pitch RMS | 11.80 deg/s | 3.31 deg/s | −72% |
| Yaw RMS | 2.20 deg/s | 1.65 deg/s | −25% |
| Motor chattering var | 2.34 | **0.12** | −95% |

Steady-state bias remained near zero on all axes, confirming the integral sliding surface eliminates static offset.

---

## Phase 5 — Deployment Safeguards

Before hardware deployment, three safeguards were added on top of the tuned SMC gains.

### 5.1 Sliding surface low-pass filter (LPF)

High-frequency IMU noise on the sliding surface can re-trigger switching chattering. A first-order LPF is applied to \(s_i\) before the saturation and reaching terms:

\[
\alpha = \frac{dt}{\tau_{lpf} + dt}, \quad \tau_{lpf} = \frac{1}{2\pi f_c}
\]

\[
s_{filtered,i} = \alpha \cdot s_i + (1 - \alpha) \cdot s_{filtered,prev,i}
\]

**Parameter:** `MC_SMC_LPF` — cutoff frequency in Hz (default **20.0**; set to **0** to disable).

### 5.2 Torque slew rate limiter

Limits the maximum rate of change of torque output per axis:

\[
d\tau_{max} = L_{slew} \times dt
\]

\[
\tau_{limited,i} = \mathrm{constrain}(\tau_{new,i},\; \tau_{last,i} - d\tau_{max},\; \tau_{last,i} + d\tau_{max})
\]

**Parameter:** `MC_SMC_SLEW` — max torque rate (default **10.0**; set to **0** to disable).

### 5.3 Safety watchdog (PID fallback)

While armed and in flight under SMC, if the rate tracking error on any axis exceeds a threshold for a sustained period, the watchdog:

1. Sets `MC_RATE_CTRL_T = 0` via `param_set()`.
2. Emits a critical MAVLink message: `"SMC Watchdog: Large tracking error! Falling back to PID."`

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MC_SMC_WD_ERR` | 1.0 rad/s | Error threshold; **0 disables watchdog** |
| `MC_SMC_WD_TOUT` | 1.0 s | Sustained error duration before fallback |

### 5.4 Code changes for safeguards

| File | Addition |
|------|----------|
| `rate_control.hpp` | `_smc_s_filtered`, `_smc_last_torque`, `_smc_lpf_cutoff`, `_smc_slew_max`; `setSMCSafeguards(cutoff, slew)` |
| `rate_control.cpp` | LPF and slew logic inside `updateSMC()`; reset filter/torque state on land |
| `mc_rate_control_params.yaml` | `MC_SMC_LPF`, `MC_SMC_SLEW`, `MC_SMC_WD_ERR`, `MC_SMC_WD_TOUT` |
| `MulticopterRateControl.hpp` | Watchdog timer `_smc_watchdog_time`; new parameter bindings |
| `MulticopterRateControl.cpp` | Watchdog logic in `Run()`; pass safeguard params in `parameters_updated()` |

### 5.5 Rebuild and verify safeguard parameters

```bash
make px4_sitl jmavsim
```

At the `pxh>` prompt:

```bash
param show MC_SMC_LPF
param show MC_SMC_SLEW
param show MC_SMC_WD_ERR
param show MC_SMC_WD_TOUT
param show MC_RATE_CTRL_T
```

### 5.6 Watchdog verification flight (`11_54_51.ulg`)

To force a watchdog trigger in simulation, set an artificially low threshold:

```bash
param set MC_SMC_WD_ERR 0.06
param set MC_SMC_WD_TOUT 0.5
param set MC_RATE_CTRL_T 1
param save
```

After takeoff, the watchdog triggers within approximately 0.5 seconds. QGroundControl displays the fallback warning and `MC_RATE_CTRL_T` reverts to `0` (PID). Log `11_54_51.ulg` captures this event.

---

## Phase 6 — Final Working Configuration

The validated research configuration disables the watchdog so SMC remains active for the entire flight, while retaining the LPF and slew rate limiter.

### 6.1 Disable watchdog and confirm SMC mode

```bash
param set MC_SMC_WD_ERR 0.0
param set MC_RATE_CTRL_T 1
param save
```

Setting `MC_SMC_WD_ERR = 0.0` completely disables the watchdog logic.

### 6.2 Execute validation flight

With jMAVSim running and QGroundControl connected, perform a takeoff, hover, maneuver, and landing. The resulting log is **`11_58_58.ulg`**.

### 6.3 Confirmed parameter snapshot (`11_58_58.ulg`)

```
MC_RATE_CTRL_T  = 1
MC_SMC_C_R      = 2.0
MC_SMC_C_P      = 2.0
MC_SMC_C_Y      = 1.0
MC_SMC_ETA_R    = 0.02
MC_SMC_ETA_P    = 0.02
MC_SMC_ETA_Y    = 0.02
MC_SMC_BND_R    = 0.20
MC_SMC_BND_P    = 0.20
MC_SMC_BND_Y    = 0.20
MC_SMC_KEQ_R    = 0.05
MC_SMC_KEQ_P    = 0.05
MC_SMC_KEQ_Y    = 0.10
MC_SMC_KS_R     = 0.10
MC_SMC_KS_P     = 0.10
MC_SMC_KS_Y     = 0.10
MC_SMC_LPF      = 20.0
MC_SMC_SLEW     = 10.0
MC_SMC_WD_ERR   = 0.0
MC_SMC_WD_TOUT  = 0.5
```

### 6.4 Final performance summary

| Metric | PID baseline | Tuned SMC (no filters) | **Final (`11_58_58.ulg`)** |
|--------|--------------|------------------------|----------------------------|
| Roll RMS | 1.26 deg/s | 8.43 deg/s | **6.52 deg/s** |
| Pitch RMS | 0.60 deg/s | 3.31 deg/s | **3.34 deg/s** |
| Yaw RMS | 0.31 deg/s | 1.65 deg/s | **2.57 deg/s** |
| Motor chattering var | 1.49e-04 | 0.12 | **0.10** |
| Watchdog fallback | — | — | **Disabled (pure SMC)** |
| Steady-state bias | — | — | **< 0.1 deg/s all axes** |

The LPF and slew limiter further reduced motor derivative variance from 0.12 to 0.10 and improved roll tracking by approximately 22% relative to tuned SMC without filters.

---

## Complete Workflow Checklist

1. Clone PX4-Autopilot with submodules into `~/Desktop/px4`.
2. Run `Tools/setup/macos.sh` (with `--sim-tools` for jMAVSim).
3. Create Python venv and install requirements.
4. Install QGroundControl via Homebrew cask.
5. Implement SMC in five source files (params YAML, module hpp/cpp, rate_control hpp/cpp).
6. Fix build issues: `int32` type and `MC_RATE_CTRL_T` name length.
7. Compile: `make px4_sitl none` — verify parameter switch at `pxh>`.
8. Launch GUI sim: `make px4_sitl jmavsim`; connect QGroundControl.
9. Set `SYS_HAS_MAG 1`; save parameters.
10. Fly PID baseline (`MC_RATE_CTRL_T = 0`); analyze log.
11. Fly untuned SMC (`MC_RATE_CTRL_T = 1`, defaults); analyze chattering.
12. Apply tuned gains (`BND = 0.20`, `ETA = 0.02`, etc.); re-fly and analyze.
13. Implement LPF, slew limiter, and watchdog; rebuild firmware.
14. Verify watchdog with `MC_SMC_WD_ERR = 0.06`, `MC_SMC_WD_TOUT = 0.5`.
15. Disable watchdog: `MC_SMC_WD_ERR = 0.0`; set `MC_RATE_CTRL_T = 1`; save.
16. Execute final validation flight; analyze log **`11_58_58.ulg`**.

---

## Notes for Hardware Deployment

- SMC tracking error on roll remains higher than PID (6.52 vs 1.26 deg/s RMS). Further gain optimization or outer-loop tuning may be required before production use.
- For physical aircraft, re-enable the watchdog with conservative defaults (`MC_SMC_WD_ERR = 1.0`, `MC_SMC_WD_TOUT = 1.0`) unless the operational risk of pure SMC flight is explicitly accepted.
- All custom parameters are visible and tunable in QGroundControl immediately after firmware flash; no QGC rebuild is necessary.
- Firmware for Pixhawk hardware is built with the standard PX4 toolchain after SITL validation: `make px4_fmu-v5_default` (or the target matching the flight controller board).

---

## Reference: SMC Control Path in Code

When `MC_RATE_CTRL_T = 1`:

1. QGroundControl writes the parameter via MAVLink.
2. `MulticopterRateControl::parameters_updated()` calls `_rate_control.setControllerType(1)`.
3. `RateControl::update()` detects `_controller_type == 1` and invokes `updateSMC()` instead of the PID branch.
4. Torque outputs publish on the standard uORB topics consumed by the control allocator — no changes to downstream modules are required.

This architecture allows A/B comparison between PID and SMC on identical hardware, simulation, and logging infrastructure with a single parameter change.
