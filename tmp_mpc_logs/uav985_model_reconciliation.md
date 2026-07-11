# Measured-Envelope Physical Model Reconciliation

This is the model contract shared by the legacy-named `uav985` Gazebo target,
rate MPC, and model-based SMC. The target now represents the user's current
airframe. Only mass and arm length are measured; inertia and propulsion values
remain provisional and must not be presented as identified hardware values.

## Known and estimated quantities

| Quantity | Value | Status |
|---|---:|---|
| All-up mass | `1.8 kg` | user measurement, approximate |
| Center-to-motor arm length | `0.300 m` | user measurement |
| Rotor x/y offset | `0.212132 m` | derived as `0.300/sqrt(2)` |
| Total `Jxx` | `0.041972 kg m^2` | similarity estimate |
| Total `Jyy` | `0.036686 kg m^2` | similarity estimate |
| Total `Jzz` | `0.052853 kg m^2` | similarity estimate |
| Rotor assembly mass | `0.022 kg` each | retained legacy assumption |
| Rotor local inertia | `6.998e-6, 2.2647e-5, 2.1617e-5 kg m^2` | retained legacy assumption |

The provisional inertia scales the previous tensor by
`(1.8/0.985) * (0.300/0.230)^2 = 3.109`. This assumes geometrically similar mass
distribution. Mass and arm length alone do not determine inertia, so bifilar or
CAD measurement is still required. The controller uses `x=roll`, `y=pitch`,
`z=yaw`; swapping physical axes is a model error, not a tuning adjustment.

## Gazebo link split

The four assumed rotor links total `0.088 kg`, so the base link is `1.712 kg`.
With the rotors at `z=0.060 m`, the assembled center of mass is `z=0.0029333 m` relative
to the base-link origin. Applying the parallel-axis theorem gives the required
base-link local inertia:

| Base-link quantity | Value |
|---|---:|
| `Ixx` | `0.03768242697 kg m^2` |
| `Iyy` | `0.03233450189 kg m^2` |
| `Izz` | `0.04484682286 kg m^2` |

Those base values assemble to the estimated totals above. Putting total inertia
directly on the base link would double-count the rotor contributions. The
current contract assumes that the quoted `1.8 kg` all-up total already contains
the optical-flow and range-sensor hardware. Under that assumption,
`uav985_flow` uses `1e-6 kg` fixed sensor links for simulation attachment only;
merging the shared `0.050 kg` optical-flow and `0.020 kg` LW20 models would
raise the vehicle to `1.056 kg`. Confirm this assumption by weighing the exact
flight assembly. If the sensors were not in the CAD total, add their measured
mass and location and recompute the full inertia instead of tuning around the
mass error.

## Hover-linear actuator model

The current propeller constants are `KF=1.1e-5 N s^2` and
`KM=2.75e-7 Nm s^2`, equivalently `momentConstant=KM/KF=0.025 m`. The simulated
ESC command maps linearly from `150` to `1000 rad/s`.

At `1.8 kg`, the nominal hover speed is `633.4 rad/s`, normalized hover command
is `0.5687`, and the local thrust slope is `11.84 N` per normalized motor
command. With the PX4 normalized quad-X allocator, this gives the hover
linearized torque effectiveness:

| Axis | Effectiveness |
|---|---:|
| Roll | `7.107 Nm/unit` |
| Pitch | `7.107 Nm/unit` |
| Yaw | `1.184 Nm/unit` |

The motor plugin has `12.5 ms` spin-up and `25 ms` spin-down constants. The
controller uses a conservative symmetric `25 ms` lag matching the slower
spin-down response. This is a simulated-model value, not a known real-aircraft
constant.

This effectiveness is local to hover. The same model gives roll/pitch/yaw
effectiveness of `4.544/4.544/0.757 Nm/unit` at collective command `0.30` and
`6.451/6.451/1.075 Nm/unit` at `0.50`, or `0.639x` and `0.908x` the hover value.
The current SMC uses one constant `MC_MSMC_EFF_*` vector, so identification
must cover the collective-thrust window intended for the experiment. Do not
hide a large operating-point mismatch by changing inertia.

## Identification gate

Estimate effectiveness and lag from a log containing deliberate, bounded
rate-axis excitation:

```sh
.venv/bin/python Tools/uav985_rate_model_identification.py flight.ulg \
  --axis yaw --inertia 0.04197,0.03669,0.05285
```

The tool fits Euler rigid-body torque, actuator lag, linear rate damping, and a
constant bias. It excludes landed data, allocator misses, and motor saturation;
reports the thrust/motor operating window; and accepts or rejects each requested
axis independently. It prints matching `MC_MPC_EFF_*` and `MC_MSMC_EFF_*`
values only for accepted axes. It never infers inertia from a fitted controller
gain and never sets `MC_MSMC_CFG`.

An axis is rejected for insufficient command excitation, `R2 < 0.5`, negative
fitted passive damping, out-of-range effectiveness, or a lag optimum on the
search boundary. Closed-loop hover without deliberate axis excitation is only
a sanity check because command and disturbance are correlated.

After independently verifying mass/inertia, propulsion data, every accepted
axis, gains, and torque limits, set `MC_MSMC_CFG=1` as the final card step.
Commander blocks mode-2 arming while the acknowledgment or runtime model status
is invalid.

## Measurements still required for the real vehicle

1. Measure all-up inertia and center of mass with the exact battery, landing
   gear, flight computer, and payload used in flight.
2. Measure command-to-RPM and thrust-to-RPM on a thrust stand across battery
   voltage for the installed motor, ESC, and propeller.
3. Measure reaction torque or `KM/KF`; the current yaw constant is not verified.
4. Record motor step responses to identify separate spin-up and spin-down lag.
5. Fly a tethered, low-altitude, bounded roll/pitch/yaw excitation and retain
   `vehicle_angular_velocity`, `vehicle_torque_setpoint`, `actuator_motors`, and
   battery topics at the highest practical log rate.
