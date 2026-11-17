# EE302 PS5 — Motor–Inertia Disk Speed Modelling and Control

**System:** Quanser QUBE-Servo 3 with inertia-disk attachment  
**Software:** MATLAB/Simulink R2024a (QUARC is required only for running the models on the physical QUBE).

## Where to find things

| Folder | Contents |
|---|---|
| `01_plant/` | Datasheet parameters, motor model derivation and open-loop Simulink model. |
| `02_identification/` | Simulated voltage-step identification, ramp/multi-step validation and fitted parameters. |
| `03_measurement/` | Discrete-time model, encoder quantization, speed estimation and filtering. |
| `04_p_control/` | P-controller Simulink model and simulation experiments. |
| `05_pi_control/` | PI-controller Simulink model and simulation experiments. |
| `results/` | Saved plots, numerical results and logs, organised by stage (`01_plant` through `05_pi_controller`). |
| `reference_originals/` and `docs/` | PS5 handout, datasheet, original reference scripts and supporting notes (where included). |
| Hardware data folders/archives | Open-loop measurements, hardware P/PI trials and their recorded `.mat` files. |

## MATLAB scripts to run

Run these from the project folder, in the order shown. The model-builder scripts create the corresponding Simulink models automatically.

| Purpose | MATLAB file |
|---|---|
| Derive and analyse the full/reduced motor model | `01_plant/ps5_step01_derive_and_plot.m` |
| Build and run the open-loop plant model | `01_plant/ps5_build_and_run_openloop_simulink.m` |
| Identify and validate the simulated first-order model | `02_identification/ps5_stage02_identify_validate.m` |
| Analyse encoder sampling and filtering | `03_measurement/ps5_stage03_measurement_analysis.m` |
| Build the encoder measurement Simulink model | `03_measurement/ps5_stage03_build_simulink.m` |
| Build the P-controller model | `04_p_control/ps5_stage04_build_p_simulink.m` |
| Run P-controller simulation experiments | `04_p_control/ps5_stage04_run_p_experiments.m` |
| Build the PI-controller model | `05_pi_control/ps5_stage05_build_pi_simulink.m` |
| Run PI-controller simulations and P–PI comparison | `05_pi_control/ps5_stage05_run_pi_experiments.m` |

**Other useful files (if included):** `identify_qube_hw.m` analyses the measured hardware open-loop data; `build_model3_p_controller.m` and `build_model4_pi_controller_hw.m` generate the hardware P/PI models; `ps5_stage06c_compare_in_R2024a.m` reproduces the final simulation-versus-hardware waveform comparison.

## File types

- `.m` — MATLAB scripts and parameter files.
- `.slx` — Simulink models.
- `.mat` / `.csv` — recorded signals, simulation results and numerical tables.
- `.png` / `.fig` — plots and model figures.

The results are already saved, so **running the scripts again is not required to view the submitted data**. Hardware execution additionally needs the laboratory QUARC/HIL installation; offline placeholder blocks are not connected to real hardware.
