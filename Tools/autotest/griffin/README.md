# Griffin Pro landing-controller SITL tests

Automated SITL tests for the Griffin Pro tilt-quadplane and its Lua landing controller (time-varying LQR from
K/U/Y arrays plus the attitude/cross-track loops). The suite starts `arduplane` SITL itself with the
`quadplane-tilt` frame and the `GriffinPro.json` model, flies each scenario over MAVLink, analyses the
DataFlash log and reports PASS/FAIL with metrics and a plot for every run.

This branch (`auto_test`) also carries the local SITL motor model (`libraries/SITL/SIM_Motor.*`,
`SIM_Frame.*`: momentum-theory rotor drag and motor rpm state) that the tests were developed on.

## Setup

```bash
./waf configure --board sitl && ./waf plane           # build/sitl/bin/arduplane
python3 -m venv ~/venv-ardupilot && ~/venv-ardupilot/bin/pip install pymavlink numpy matplotlib
cd Tools/autotest/griffin
~/venv-ardupilot/bin/python griffin_tests.py all      # ~45 min for all 66 runs (6 parallel, speedup 10)
```

Results go to `results/<suite>_<date>/`: `logs/<run>.BIN`, `plots/<run>.png`, `summary.csv`,
`results.json`, `args.json`. The exit code is 1 if any run fails.

## Suites

All suites use the VTOL_LAND waypoint from flight log 255 (40.0594829, -88.5513282). The approach axis is due
north. `--motor fastmot,lagmot` runs every case twice (see the note on motor lag below).

| suite | what it does | default matrix |
|---|---|---|
| `wind` | Take off 399 m S of the WP, force VTOL, loiter. The script triggers from hover (hover-hold arrays). Wind is 0 until t0+15 s (t0 = script start), then steps to the given speed. | head (from 0 deg), x45 (45 deg), cross (90 deg) x 2.5/5/7.5/10 m/s |
| `baseline` | Same as `wind` with `SCR_ENABLE 0`: ArduPilot's VTOL loiter holds the point. t0 = LOITER item reached, the run ends at t0+36 s. | same |
| `moving` | No wind. QLOITER run-up with the harness closing loops on the sticks (heading 0, hold the line, full stick, `Q_LOIT_SPEED_MS` = v), then a switch to AUTO as the trigger point (399.5 m S of the WP) is crossed. The script triggers on its next loop with the aircraft at speed. | fwd (north) / side (east, nose north) x 2.5/5/7.5/10 m/s |
| `approach` | Mission from log 255: VTOL takeoff, fixed-wing to 3 WPs at 50 m, VTOL_LAND. The script triggers 400 m from the WP and flies the 31 s approach arrays. | no wind (`--approach-wind "0,0;5,90"` for more) |

Notes:
- `moving` sets `Q_ENABLE 2`. Otherwise, switching QLOITER -> AUTO resumes the loiter as fixed-wing: AUTO starts a
  transition, and the transition timeout later triggers QRTL.
- `moving`, side: above ~6 m/s the airframe cannot hold its heading in still-air sideways flight (it weathervanes,
  even with ArduPilot flying). So by default the side run-up uses a "carrying" wind from the west equal to the ground
  speed (the aircraft sees still air), set to 0 when the script triggers. `--no-carry` gives a real still-air
  run-up (fine at 2.5 and 5 m/s, and it gives the same response as carrying there).
- The script is copied into each run with `sim_time_total` / `sim_time_steps` patched from the array set
  (`meta.json` t_total, and the number of Y columns). The hover set is 30 s / 121 steps. The approach set is
  31 s / 187 steps (6 per s), with K at 1 s intervals (32 sets).

## Motor lag (important)

`assets/griffin_sitl.parm` (the full parameter dump of the SITL log 255 setup) sets `SIM_VIB_MOT_MAX 168`. In
`SIM_Frame::calculate_forces` the vibration code then overwrites the motor model's rpm state every step, so the motors
spin up almost instantly (`fastmot`). `lagmot` sets it to 0, so the model's 0.15 s spin-up lag acts (closer to the
real aircraft). With lag, the script's roll loop limit-cycles at ~3-4 Hz even in calm air.

## PASS criteria

- Scripted runs: the script took over (LXTE logged), the hand-off message ("Landing Approach Completed") arrived,
  and there was no ground impact ("Hit ground") between script start and hand-off + 1 s.
- Baseline runs: no ground impact up to t0+36 s.
- Any timeout (default 600 s wall per run), SITL or MAVLink error is a FAIL.

PASS only means the aircraft survived. Judge quality from the metrics in `summary.csv` and the plots: x/h/pitch
error against the script's reference, cross-track, roll/pitch/yaw-error peaks, and the % of time the roll
differential thrust is saturated. Pre/post windows are split at the wind step.

## Options

```
--script PATH        controller to test (default assets/controller_spd.v2.lua)
--arrays DIR         K/U/Y (+ meta.json) instead of the suite default
--binary PATH        arduplane SITL binary (default <repo>/build/sitl/bin/arduplane)
--motor LIST         fastmot,lagmot
--dirs / --move-dirs / --speeds / --approach-wind   matrices
--parallel N         concurrent SITL instances (default 6; 2-4 min wall per run at speedup 10)
--instance-base N    first -I instance; use distinct ranges for concurrent invocations
--keep-runs          keep the per-run SITL directories (eeprom, sitl.out, scripts as run)
```

## Files

- `griffin_tests.py`: scenarios, job matrices, parallel runner, verdicts, summary.
- `griffin_sim.py`: SITL launch (own run directory, relative model path, `--defaults` chain), MAVLink helpers
  (mission upload with request de-duplication, force-arm before AUTO, RC override).
- `griffin_analysis.py`: log metrics and plots.
- `assets/`: `GriffinPro.json` (SITL model), `GriffinPro.parm` + `griffin_sitl.parm` (params),
  `controller_spd.v2.lua`, `arrays/hover_hold`, `arrays/approach_31s`.

Known gotcha: SITL resolves an absolute model-JSON path relative to its cwd (the leading `/` is stripped), so the
harness copies the JSON into the run directory and passes a relative path.
