# Stage 01 - Datasheet motor-inertia disk derivation

## Sources and conventions
QUBE-Servo 3 system parameters from uploaded manufacturer Table 2.2. The inertia disk is modelled as a **uniform solid circular disk**, rigidly mounted to the motor/hub. The datasheet does not specify a viscous-friction coefficient, so `b=0` is an **explicit simplifying assumption** for the initial analytical model, not an identified or manufacturer-supplied value. Load torque is set to zero for voltage-to-speed transfer functions. All model outputs are in rad/s, inputs in volts.

## Inertia
- Rotor inertia: `Jm=1.4e-6 kg*m^2`.
- Hub inertia: `Jh=0.6e-6 kg*m^2` (manufacturer's stated attachment hub inertia, not recomputed from mass/radius).
- Disk mass `md=0.053 kg`, radius `rd=0.0248 m`.
- Uniform-solid-disk inertia: `Jd = 0.5*md*rd^2 = 1.629856e-5 kg*m^2`.
- Equivalent inertia: `J = Jm + Jh + Jd = 1.829856e-5 kg*m^2`.

## Governing equations

Electrical loop (armature winding):

`v(t) = Lm * di(t)/dt + Rm * i(t) + Km * omega(t)`

Mechanical loop:

`J * domega(t)/dt = Kt * i(t) - b * omega(t) - tau_load(t)`

`Rm=7.5 ohm`, `Lm=1.15e-3 H`, `Kt=0.0422 N*m/A`, `Km=0.0422 V/(rad/s)`.

For the state `x=[i;omega]` with inputs `[v;tau_load]` and output `y=omega`:

```
A = [ -Rm/Lm,   -Km/Lm;
       Kt/J,      -b/J];
B = [1/Lm, 0;
        0, -1/J];
C = [0, 1];
D = [0, 0];
```

For the voltage-only system, use `B(:,1)` and scalar `D=0`.

## Transfer functions

Full second-order speed response:

`G_full(s) = Omega(s)/V(s) = Kt / [(Lm*s+Rm)*(J*s+b) + Kt*Km]`

or

`G_full(s) = Kt / [Lm*J*s^2 + (Lm*b+Rm*J)*s + (Rm*b+Kt*Km)]`.

Reduced first-order speed response (armature electrical transient neglected):

`G_red(s) = Kt / [Rm*J*s + (Rm*b+Kt*Km)]`.

Reduced speed DC gain and time constant:

`K = Kt/(Rm*b+Kt*Km)` in rad/s per V, `tau = Rm*J/(Rm*b+Kt*Km)` in seconds.

For the initial `b=0` assumption:
- `K = 23.6966824645 rad/s/V`.
- `tau = 0.07706430673 s`.
- `Lm/Rm = 0.00015333333 s`.
- Full poles approximately `-6508.73703` and `-13.0020976 rad/s`.
- Reduced pole approximately `-12.976... rad/s` (compute directly from the script; avoid rounding this source manually for comparison).
- The steady speed for a 2 V constant ideal command is `47.393364929 rad/s`.

This is an **ideal manufacturer-parameter model**, not hardware system-identification data. The absence of viscous/Coulomb friction, amplifier deadband, current limiting, voltage mapping and unmodelled dynamics can produce large discrepancies. The manufacturer table lists 0.65 V default amplifier deadband and recommends +/-10 V output range, but the uploaded controller code reads its actual limits from an unavailable `parameters.m` and we do not substitute the datasheet recommended range for those unknown run settings.

## Numerical tests to archive
1. Both open-loop step responses on one axis (full vs reduced, identical voltage input).
2. Pole locations and stable/fast-vs-slow mode comparison.
3. Bode magnitude and phase comparison.
4. Rise time, 2% settling, overshoot, steady value; model and source/assumption table.
5. Simulink-built model screenshot/export and matched `.mat` logs after MATLAB execution.

## Stage 01 source files
- `01_plant/ps5_parameters_datasheet.m` - parameter definitions.
- `01_plant/ps5_step01_derive_and_plot.m` - analytical TF/SS calculations, plots, metrics and log export.
- `01_plant/ps5_build_and_run_openloop_simulink.m` - generates `ps5_stage01_openloop_2024a.slx` and runs actual Simulink normal simulation.

No MATLAB R2024a environment is available in this assistant's runtime. Only the preliminary independent mathematical cross-check is populated; run the two scripts on your R2024a installation to produce authoritative MATLAB/Simulink `.fig`, `.slx`, `.mat`, and CSV results.
