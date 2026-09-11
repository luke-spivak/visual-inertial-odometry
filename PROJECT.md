# PROJECT.md — GPS-Denied VIO Quadcopter

## Objective

Build a monocular visual-inertial odometry system on a 5-inch quadcopter that can hold position outdoors without GPS, and characterize its drift against measured ground truth. The deliverable is not "it flies" — it is a number: drift as a percentage of distance traveled, across varied altitude, speed, and terrain, with the evaluation harness that produced it.

Secondary objective: the sim harness and evaluation framework are reusable for any estimator, and are arguably more valuable than the specific VIO implementation.

**Status of the number, in simulation (2026-09-03).** Estimator, open loop: **2.29 % median drift over three flights, 1.91–2.89 % across eight runs, ATE 0.31–0.42 m over 155 m at 96 % coverage.** Aircraft, GPS-denied closed loop: **0.38–1.73 % net drift, 3/3 flights**, with peak mid-flight excursions of 10–13 m. Both meet the < 5 % gate; the peak excursion does not, and it is the number that would matter near obstacles.

Phase 3 milestones 1–6 are complete. **This is not the deliverable.** The deliverable is this number on hardware against measured ground truth, and the sim flatters the front end in every way PROJECT.md already lists — perfect global shutter, no motion blur, no vibration, exact timestamps — plus two more found here: the mission is a closed circuit that re-observes its own features, and the 3D obstacle field turns out to contribute nothing that a textured plane does not.

---

## Current status

| Phase | State |
|---|---|
| Airframe triage | **Complete (2026-08-27)** — flies cleanly on Betaflight 2026.6.1. Gate met: stable hover, even motor temps, failsafe verified, arm/disarm on ELRS. Residuals: one motor ticks when hand-spun (random, no play — debris; no gyro or thermal signature under load), and level trim left rough deliberately since ArduPilot redoes it |
| Pi + IMU bench bringup | **In progress (2026-09-09)** — Pi 5 up (`viopi`, Pi OS 13 trixie, kernel 6.18.39+rpt-rpi-2712), SD verified genuine via f3, SPI enabled, Active Cooler fitted. **ISM330DHCX wired to SPI0 CE0 and verified end to end on raw spidev**: WHO_AM_I 0x6B, gravity 9.63 m/s², gyro 0.79 dps at rest, INT1 asserting and clearing on GPIO25, tagged FIFO draining both sensors (`harness/imu_probe.py`). **The bus works at 10 MHz and nowhere else** — see *IMU bringup*, 2026-09-09. **Overlay installed and loading**, but Pi OS builds no `st_lsm6dsx` (`# CONFIG_IIO_ST_LSM6DSX is not set`) so nothing binds — out-of-tree module build still to do. **Allan run complete (2026-09-09)**: 3 h stationary, 4.75 M samples/sensor, 0 overruns, no gaps, via `harness/imu_log_spidev.py` off the hardware FIFO, no root. Part delivers **440 Hz for a requested 416**. **Noise densities measured: accel 5.37e-04 m/s²/√Hz, gyro 1.04e-04 rad/s/√Hz** (both at or better than datasheet), bias instability 3.97e-04 / 1.77e-05. **Phase 2 steps 1-3, 5 done; step 4 partially.** Driver built out of tree and under DKMS, INT1 interrupting, monotonic clock pinned by udev. **Step 4 PASSED (2026-09-09)**: 439.57 Hz, clock skew 0 ppm (timestamp jitter was reported as 0.062 µs; that does not hold at the default watermark: 22.8 µs, see 2026-09-10 correction under the patch) against CLOCK_MONOTONIC, monotonic and gap-free — after patching `st_lsm6dsx` to re-anchor `ts_ref`, which stock drifts 1.2 s per 10 min. **PHASE 2 COMPLETE (2026-09-09).** Step 6 done: `harness/buildenv/` is a debian:trixie arm64 container matching viopi's ABI exactly (glibc 2.41, gcc 14.2.0, `__GLIBCXX__` 20250315), gated by compiling a binary in the container and executing it on the Pi — see *Build environment* |
| Sim harness | **In progress** — Ubuntu 24.04 arm64 in UTM. Milestones 1–4 done. OpenVINS **validated on EuRoC V1_01_easy: ATE RMSE 0.115 m, RPE 0.72 %/10 m, scale 1.000**. On our own sim: **15.4–16.0 % drift, ATE 2.5 m over 156 m**, two flights × three replays, down from 97.8 % — see 2026-09-02. **Milestone 3's < 5 % gate is MET: drift 2.29 % median over three flights (1.91–2.89 % across eight runs), ATE 0.31–0.42 m over 155 m, 96 % coverage** — against a EuRoC reference of 0.72–0.80 % and 0.067–0.115 m. Two fixes got there: the chi-squared gate (97.8 % → 15 %) and holding heading through the corners (15 % → 2 %). Milestone 6 done (`harness/sweep.sh`). **Milestone 5 done: 3/3 GPS-denied flights complete the mission, net drift 0.38–1.73 %** (peak excursion 1.0–7.9 %, which is the real operational limit). **Phase 3 milestones 1–6 all complete** |
| Camera bringup | **In progress (2026-09-07)** — OV9281 enumerates on Cam0, all six modes reported, `ov9281_mono.json` tuning file ships with Pi OS and loads. Raw capture confirmed good: 640×400 R8, well-exposed, full dynamic range. **The ISP's processed RGB output is silently all-zero and must not be used** — see *Camera bringup*, 2026-09-07. 640×400 confirmed **binned, not cropped**, so full lens FOV is preserved and the bracket's §7 geometry holds. **Timestamp gate PASSED**: `SensorTimestamp` jitter 0.60 µs stdev, 82× tighter than userspace arrival, zero drops, monotonic timebase confirmed (`harness/cam_timing.py`). **Focus set and threadlocked (2026-09-10).** Kalibr image built on the sim VM and verified on arm64 after five silent failures — see *Kalibr on an arm64 VM*, 2026-09-10. Target generated (Aprilgrid 6×8, 45 mm tags, A2). **EuRoC validation PASSED (2026-09-10)**: every intrinsic within 0.1 %, stereo baseline within 0.08 %. **Intrinsics (2026-09-10)**: fx ≈ 1116 px, field of view H 69° / V 42° / D 83° — the spec's 118° is wrong for this lens. Right quarter of the image not yet covered by the target. **Camera-IMU (2026-09-10)**: rotation identity + 2.2°, IMU ~30 mm behind the optical centre, time offset 2.2 ms, stable across two IMU models. **Step 7 prepared (2026-09-10)**: OpenVINS runs natively on the Pi (`vio_live`, no ROS), config loads the Kalibr calibration, every run records for offline replay — see *Step 7 prepared*. Bench run 1: images noise-dominated at 1 ms (cap now 4 ms); IMU stream died after 15 s: the sensor went bad on the handheld rig (one-shot reads ~20 g, doubled bytes); suspect dupont clips. Next: power-cycle, secure wiring, re-verify, then bench |
| ArduPilot transition | **Flashed and verified on the board (2026-09-09)** — `vio_full` at 899,524 B used / 99,888 B free, the 09-03 numbers reproduced exactly. DFU'd from Betaflight with `arducopter_with_bl.hex`; the `.apj` could not have done it. Board enumerates as `ArduPilot`/`speedybeef4v4`, QGC reads V4.8.0-dev, **`VISO_TYPE` present** — the gate that proves it is the custom build. Unconfigured as yet: frame class undefined, accel uncalibrated, no compass attached, radio uncalibrated |
| Payload integration | Printer in hand; mounts not yet designed. Blocked on Phases 1/2/4 |
| Vision in the loop | **Complete in sim.** Vision reaches EKF3 with correct frames (milestone 4) and flies the full mission GPS-denied, 3/3, at 0.38–1.73 % net drift over 118–194 m on vision alone. Peak mid-flight excursion 10–13 m is the operational limit. Hardware is untouched |
| Evaluation | Blocked |

---

## Hardware inventory

### Owned (existing quad, condition unverified)
- SpeedyBee F405 V4 stack — STM32F405, 1 MB flash, 55 A BLHeli_S 4-in-1 ESC, 30.5×30.5
- TBS Source One V5 5-inch frame (open-source; CAD at `github.com/tbs-trappy/source_one`, community STEP on Printables 261673 / 593881)
- EMAX ECO II 2306 1700 KV × 4 — these are **4S** motors
- Happymodel ExpressLRS Nano EP2 2.4 GHz receiver
- HQProp DP 5×4.8×3 PC V1S, 3-blade
- Transmitter, LiPo charger, 2 × 4S LiPo packs (**4S confirmed**; capacity, connector, and health still unverified)

### Purchased
- Benewake TF-Luna rangefinder
- Adafruit ISM330DHCX IMU (product 4502)
- Smoke stopper
- InnoMaker CAM-MIPI9281RAW-V2 camera — Amazon B09WTP5GZH. 32 × 32 mm, 28 × 28 M2 holes, f = 2.8 mm, 148°D/118°H **confirmed** from manual V1.4
- SanDisk Extreme 128 GB microSD ($40) — boot + flight recording card. `f3write`/`f3read` on arrival (Extreme is the most-counterfeited card on the market)
- Matek 12S Pro BEC — **meter the output rail at 5 V before it touches the Pi**
- Bambu Lab A1 mini 3D printer (base, no AMS) — textured PEI plate, no glue stick needed for PETG
- Filament: **ELEGOO PETG 1.75 mm 1 kg black, B0D41Y3WWZ** (standard, not Rapid — slower printing gives better layer adhesion, and speed is not a constraint on 20–60 min parts). Set "Generic PETG" in Bambu Studio; let the A1 mini run flow calibration. No PLA — 60 °C Tg creeps under load in sun. Store sealed with desiccant between sessions. Inspect black parts for stress cracks with a raking flashlight, not head-on
- SEQURE M10-25Q GPS/compass — u-blox M10 (UBX), QMC5883L, 12.2 g. **Set `GPS_TYPE=2`, not Auto** — Auto can fall back to NMEA and silently drop 3D Doppler velocity, which is the VIO scale-error check

### Ordered (PiShop, $140.85)
- Raspberry Pi 5 4GB + Active Cooler + official 27 W USB-C PSU — $132.90
- Camera Cable for Pi 5, 300 mm, ×3 — $7.95 (**verify Standard-to-Mini variant: 22-pin Pi end, 15-pin camera end**)

### Ordered (Amazon)
- M2.5 nylon standoff/screw assortment

### Still to buy
| Item | Part | Price | Source |
|---|---|---|---|
| FC blackbox card | Any **≤32 GB SDHC**, FAT32 (overwrite format). Commodity part — cap spend ~$10; 32 GB is a dead capacity and often overpriced. Betaflight addresses only ~4 GB; ArduPilot uses all | ≤$10 | anywhere |
| Incidentals | 20 AWG silicone wire, blue threadlocker, momentary pushbutton (GPIO shutdown), silicone damping balls. XT60 pigtail optional — bench-testing the BEC only, not needed in the flight build | ~$20 | — |
| Later | ArduSimple simpleRTK2B + multiband antenna | ~$271 | ardusimple.com |

### Explicitly dropped
- **Matek 3901-L0X optical flow** — needs four separate features re-enabled in the 1 MB firmware build (`AP_OPTICALFLOW_ENABLED`, `HAL_MSP_OPTICALFLOW_ENABLED`, `EK3_FEATURE_OPTFLOW_FUSION`, `MODE_FLOWHOLD_ENABLED`), all competing for flash needed by external nav. Also low-altitude-only, so contributes nothing at cruise.

---

## First-principles derivations

This section exists because most of the parts selection here contradicts what a search would surface. The requirements were derived from what the estimator mathematically needs; the parts were then chosen to satisfy them.

### Camera

VIO fuses two things: an IMU giving high-rate motion that drifts, and a camera giving drift-free geometric constraints by observing the same 3D points across frames. Every camera spec derives from one of four requirements.

**1. The projection model must be valid.**
VIO assumes every pixel in a frame was captured at one instant from one camera pose. Rolling shutter breaks this structurally — rows expose sequentially, so on a quad yawing at 200°/s with 20 ms readout, top and bottom of the image come from poses 4° apart while the math says they don't. This is a *systematic bias correlated with rotation rate*, not noise that averages out.
→ **Global shutter is a hard requirement, not a preference.**

**2. Frame capture time must be known relative to the IMU.**
The estimator integrates IMU samples between frame timestamps. A frame stamped at *t* but exposed at *t+δ* attributes motion to the wrong interval — at 5 m/s, a 20 ms error injects 10 cm of position error per frame.
- A **constant** δ is fine. OpenVINS (`calib_camimu_dt`) and VINS-Fusion (`estimate_td`) solve it online.
- A **variable** δ is unmodelable and corrupts everything.

CSI gives `SensorTimestamp` on `CLOCK_MONOTONIC`, marking first-pixel/start-of-readout — a hardware event, fixed offset to exposure, same clock the SPI IMU reads on. UVC stamps on frame *arrival*, which varies with transfer time and, under MJPEG, with scene content (compressed frame size depends on what you're looking at).
→ **CSI over USB. This is the deciding factor between the two interfaces.**

**3. Features must persist across frames.**
You need the same points visible long enough, with enough baseline, to triangulate. Narrow FOV loses features fast during rotation; motion blur makes corners unlocalizable.
→ **Wide FOV (100–160° target) and short exposure.**

**4. Features must be detectable.**
Corner detectors key on local intensity gradients.
- Noise looks like texture → need SNR → need photons per pixel → **large pixels, large optical format**
- Compression artifacts create fake corners → **RAW output, not MJPEG**
- A Bayer filter discards ~2/3 of incident light, and demosaic *interpolates* — it invents gradients that weren't in the scene → **mono, not color**

**5. Optics must be stable.**
Intrinsic calibration (focal length, principal point, distortion) must hold constant or every observation is projected through a wrong model. Autofocus changes focal length. Auto-exposure hunting causes tracker dropout flying shadow→sun.
→ **Fixed focus, manual exposure. Both are features here, not limitations.**

**Resulting choice: InnoMaker CAM-MIPI9281RAW-V2.** OV9281 mono global shutter, CSI, RAW8/RAW10, 148° D / 118° H, manual exposure, mainline `ov9281` driver (`dtoverlay=ov9281`), opto-isolated hardware trigger and strobe via TLP281. Satisfies all five. Costs $35–57. **32 × 32 mm board, 4 corner holes on a 28 × 28 mm pattern, M2.**

**Corrected from the InnoMaker manual (V1.4, §3.2):** the board is **32 × 32 mm**, not 38 × 38; stated weight 4 g; focal length is **2.8 mm**, not the ~0.7–1 mm estimated here; 1/4", 3 µm pixels. **FOV is confirmed 148° D / 118° H** — the spec-table contradiction is resolved in favour of the bullets.

That focal length moves the hyperfocal distance from the ~5–10 cm claimed here to **~0.9 m** at f/2.8 (H ≈ f²/Nc, c ≈ 3 µm). Practically the conclusion is unchanged — set to infinity and everything past ~0.5 m is sharp, which at flight altitude is everything — but **focus is not the non-issue this section implied**. Set it badly and distant texture goes soft, corners stop localising, and the front end degrades. The Phase 4 Laplacian-variance procedure on a 100 m+ target is required, not belt-and-braces. Lock with blue threadlocker, then calibrate. **Order matters — refocusing after calibration invalidates the intrinsics.**

### IMU

The VIO estimator *is* the fusion algorithm. It estimates orientation, velocity, and both accel and gyro biases as part of its state vector. A sensor-fusion chip delegates that job, producing two estimators fighting.

**Why not a BNO085** (the obvious search result for "IMU breakout"):
- It runs CEVA SH-2 firmware on an onboard Cortex-M0+ and outputs quaternions and gravity-removed acceleration. VIO wants raw.
- It performs continuous dynamic calibration, silently adjusting gyro bias. Your estimator simultaneously estimates that bias and expects it to drift slowly and predictably. When the sensor corrects itself underneath you, the bias state chases a non-physical moving target. This produces drift that looks like an estimator tuning problem and isn't.
- Reports arrive over SHTP with the fusion firmware's scheduling and latency in the path, stamped on its internal clock — reintroducing exactly the variable offset that requirement 2 above exists to eliminate.

**ISM330DHCX** is deliberately dumb: raw accel + gyro, up to 6.66 kHz, hardware FIFO with per-sample timestamping, INT1 data-ready interrupt, mainline `st_lsm6dsx` IIO driver.

The FIFO matters specifically: the sensor stamps samples on its own schedule and you drain them in batches, so *your read timing doesn't contaminate the sample timing*.

Use SPI, not I2C — the STEMMA QT connector is the path of least resistance and the wrong one.

**Corrected reasoning (2026-08-27).** The original justification here was "400 kHz I2C bottlenecks at target ODR and adds jitter." Neither half survives arithmetic at the actual target rate:

- **Bandwidth.** In FIFO mode a sample is a 1-byte tag plus 6 data bytes per sensor: 14 B/sample for accel + gyro. At 416 Hz that is 5.8 kB/s, and I2C spends ~9 bit-times per byte, so ~52 kbit/s against a 400 kbit/s bus — **~13% utilization**. I2C is not a bottleneck here. It saturates somewhere north of 2-3 kHz ODR.
- **Jitter.** Weaker still, and for a reason central to this design: hardware FIFO timestamping exists precisely so read timing cannot contaminate sample timing. A slower bus does not corrupt timestamps the sensor already stamped.

The decision stands, on different grounds:

1. **Headroom.** 416 Hz is a plan, not a commitment. If the sim phase argues for 833 Hz+, SPI costs nothing, while I2C means re-soldering an assembly that Phase 4 explicitly forbids disassembling after Kalibr calibration.
2. **Mechanical — the real reason.** STEMMA QT is a 1 mm-pitch friction-fit JST-SH connector: fine on a desk, a liability on a 5-inch quad. Phase 6 already demands strain-relieved soldered wire on this bracket. An intermittent IMU connection in flight is indistinguishable from estimator divergence.
3. **SPI costs two extra wires.** No saving to trade against the risk.

Pattern worth noting: the original claim was plausible, directionally sensible, and wrong — the same failure mode catalogued in *Where priors failed*.

**Why not ICM-42688-P** (better noise specs, was the original pick): genuine parts are supply-constrained with a live counterfeit market, no first-party breakout from SparkFun or Adafruit, and the available options are Tindie (Australia), TDK Pmod boards (awkward form factor), or generic Amazon listings of uncertain provenance. The ISM330DHCX is noisier but gyro bias instability barely matters when integrating over the ~30 ms between frames. **Calibration quality and time sync dominate the error budget by a wide margin — not sensor noise density.**

### Why a GPS receiver on a GPS-denied navigation project

The compass is the actual requirement. The GPS is a bonus on the same board.

The SpeedyBee F405 V4 has **no onboard magnetometer** (ArduPilot docs: attach external via I2C on SDA/SCL pads). Without a compass, ArduCopter has no yaw source. The alternatives are GSF (needs GPS velocity to converge) or ExternalNav (which is the untrusted thing being validated). With neither compass nor working VIO you are locked out of Loiter, PosHold, and all auto modes — permanently in Stabilize/AltHold, unable to bench the thing being built.

The GNSS earns its place four ways:
1. **Validation reference.** With `EK3_SRC1` = GPS and `EK3_SRC2` = ExternalNav, ArduPilot continuously aligns the vision estimate to GPS while GPS is primary. A swapped axis or ENU/NED error appears the instant you flip the source switch, not when the aircraft flies into a fence. This is free correctness checking unavailable in an actual GPS-denied environment.
2. **Failsafe.** RTL and Loiter on GPS when VIO diverges. It will.
3. **Free velocity ground truth.** GNSS Doppler velocity is accurate to ~5 cm/s — far better than its position — so it catches VIO scale errors before RTK is purchased.
4. **EKF origin.** Set automatically; otherwise manual from the GCS every flight.

**Compass chip constraint — narrower than first assumed.** Pulling `features.txt` directly, the stock build enables AK09916, BMM150, BMM350, HMC5843 (covers HMC5883L), ICM20948, **IST8308**, IST8310, LIS3MDL, MMC3416, QMC5883L, and RM3100. The *only* relevant exclusion is `!AP_COMPASS_QMC5883P_ENABLED`.

So practically every GPS/compass module on the market works. Avoid exactly one thing: a module carrying the **QMC5883P**, the 2026-era successor to the QMC5883L (discontinued June 2025), which is what an unlabeled "5883" module may now silently ship.

**And even that stops binding at Phase 5**, because that's a custom build — if the only module in stock has a QMC5883P, enable its driver on `custom.ardupilot.org`. One compass backend is rounding error against the OSD, VTX, and telemetry stacks already being cut. The original "must be IST8310" reasoning assumed stock firmware and does not survive the custom build.

### Power

Pi 5 needs 5 V at up to 5 A. The F405 V4's onboard 5 V rail is **3 A total** across all nine 5 V outputs (stack manual, spec table), already shared with the FC itself, the ELRS receiver, and the LED pads. A separate BEC is mandatory.

FPV BEC marketing quotes **peak**, not continuous, because the parts are designed for servos and VTXs that draw in bursts. A Pi pulling steadily for ten minutes is a different load. Concrete examples of parts that fail this:
- Matek Micro BEC: 1.5 A continuous, 2.5 A peak for 5 s per minute
- MEIVIFPV adjustable: 5 V / 3 A

A 3 A BEC survives the bench and fails in flight, producing intermittent companion-computer reboots that look like software problems. **Look for the word "continuous."**

**Topology — the BEC taps raw battery voltage in parallel with the ESC:**

```
4S LiPo ──XT60──> ESC BAT+ / BAT-  ──┬──> ESC FETs → motors
                  (+ 1000 µF cap)    ├──> 8-pin cable VBAT → FC (voltage sense, FC's own 5 V / 9 V regs)
                                     └──> Matek 12S Pro ──5 V──> Pi GPIO pins 4 & 6
```

Solder the BEC input to the ESC's `BAT+` / `BAT-` pads — the same pads the XT60 pigtail and the low-ESR cap land on. If Phase 1 triage finds those pads marginal, tap the XT60 pigtail leads instead and leave the board alone.

**Input current is small.** Stepping 16 V down to 5 V means ~15 W at the Pi is only **~1.1 A on the input side** (~2 A at the BEC's full 5 A output). 22 AWG is ample there. The 20 AWG spec belongs on the **5 V output run**, where there are only ~0.3 V of brownout headroom — though at a 15 cm run the drop is single-digit millivolts either way, so 20 AWG is really buying thermal margin and vibration durability, not voltage.

Feed via GPIO 5V/GND, which bypasses USB-C PD negotiation, so set `usb_max_current_enable=1`. Don't run USB-C and the BEC simultaneously on the bench.

**Consequence: the Pi boots when the pack goes in and loses power the instant it comes out** — no clean shutdown, which is the classic way to corrupt a rootfs. Three layers, cheapest first:

1. **Read-only rootfs via overlayfs** (`raspi-config` → Performance Options). Root mounts read-only with writes going to a RAM overlay, so an unclean yank *cannot* corrupt it. Enable for the flight config, disable while developing. This is the structural fix.
2. **Momentary shutdown button on the airframe** — `dtoverlay=gpio-shutdown`, button between GPIO3 (pin 5) and GND (pin 6). Note this is **shutdown-only on a Pi 5**: GPIO3 wake worked on Pi 4 but not here, because GPIO comes from the RP1 southbridge and power-on is PMIC-controlled — neither is powered when halted. Irrelevant in practice, since replugging the pack boots it. Use `gpio_pin=` to relocate if the Pi ever needs I2C (GPIO3 is SCL1).
3. **RC-switch shutdown over the existing MAVLink link** — the bridge node already parses FC telemetry; have it watch a spare aux channel in `RC_CHANNELS` and call `shutdown -h now` when flipped **while disarmed**. No extra hardware, works from the transmitter.

Keep the recording partition separate from root regardless, so a bad unplug costs one flight's data instead of the OS.

*Rejected: supercap or UPS hold-up power.* Covering an 8 s shutdown at ~15 W needs ~120 J; a 5 F bank at 5 V yields ~10 J before dropping under the Pi's brownout threshold, so you'd need a boost stage and a far bigger bank. UPS HATs with 18650s run 50–80 g against a 140 g payload budget. Overlayfs is free.

*Rejected: powering the BEC from the FC's 9 V rail.* It's free once VTX is cut, but it cascades two regulators, shares a 3 A budget, and adds a failure point for nothing.

### Smoke stopper

A 4S pack sources well over 100 A into 14 AWG wire with no fuse anywhere in the system. A bridged pad, reversed capacitor, or chafed motor wire against carbon turns into a fire in under a second on direct connection.

A series resistor or incandescent bulb drops a little voltage under normal draw and limits fault current to 1–2 A under a short. Specifically warranted here because the aircraft has unknown history — the case the tool exists for.

### Ground truth measurement

**The reference must be roughly an order of magnitude better than what you're measuring.** VIO drift runs 0.5–2% of distance traveled, so a 200 m path accumulates 1–4 m of error. Standard GPS at 1–3 m is the same magnitude as the measurement — useless as truth. RTK at 1–2 cm is right.

**PPK over RTK.** Real-time corrections require an NTRIP subscription or a base station plus radio link. Evaluation happens offline anyway. So: log raw observables (RXM-RAWX) from the F9P during flight, download RINEX from the nearest NOAA CORS station (free), post-process with RTKLIB. Same centimeter accuracy, no corrections infrastructure, no extra UART on an already-full F405.

**Free tiers, do these first:**
- Closed-loop return error — mark takeoff, fly a loop, land on the mark, read final estimate. One number per flight, errors can cancel, but catches gross failure.
- GPS velocity comparison (above).

**Metrics:**
- **ATE** — align to truth (Umeyama, Sim(3) since monocular scale isn't directly observable), RMSE over the path. Dominated by the single worst early moment; one bad second poisons the remaining three minutes.
- **RPE** — error over fixed sub-trajectory lengths (10 m, 20 m, 50 m, KITTI-style). Gives actual per-distance drift rate. **Report this as the headline.**
- Scale error separately — distinct failure mode, distinct cause.

Tooling: `evo` (`evo_ape`, `evo_rpe`, `evo_traj`) or `rpg_trajectory_evaluation` for multi-run statistics.

**Two traps that produce fictitious error:**
- **Time alignment.** VIO timestamps on the Pi's clock, GNSS on GPS time. A 50 ms offset at 5 m/s is 25 cm of apparent error — potentially most of the measured budget, entirely fake. Discipline the Pi clock to the F9P's PPS, or at minimum estimate the offset during alignment.
- **Lever arm.** The GNSS antenna and camera aren't co-located. 15 cm of separation plus yaw is 15 cm of apparent error unrelated to VIO. Measure and apply it.

**Throughput multiplier:** record raw camera frames and IMU every flight; evaluate offline from the recording. Six flights per afternoon versus fifty parameter configurations replayed overnight against one recording. **Fly to collect data, not to test hypotheses.**

### Compute

Pi 5 4GB at $110 (up 83% from $60 launch — industry-wide LPDDR4 shortage; 2GB $65, 8GB $175, 16GB $305).

**Why not 2GB:** the estimator itself would fit — OpenVINS covariance for a dozen clone poses and fifty features is a few hundred KB, 640×400 mono frames are 256 KB each, a few hundred MB resident. What breaks is everything around it. Compiling Eigen-heavy template code peaks past 1.5 GB per g++ process, so `make -j4` OOMs and you drop to `-j1` with swap. Recording buffers have no headroom. ORB-SLAM3 (DBoW2 vocabulary plus unbounded keyframe map, 1–2 GB) is out entirely.

**Why not 8GB:** $65 more buys capacity used only for large persistent maps, and flights are 8–11 minutes so map growth is bounded. At launch pricing ($60 vs $80) the 8GB was the obvious pick. At $110 vs $175 it isn't.

**Why not Orange Pi 5** (cheaper at ~€65, faster RK3588S, pin-compatible GPIO, M.2 NVMe): the camera driver stack goes through Rockchip's ISP and a vendor BSP kernel, not mainline libcamera. Given that camera timestamping is the highest-risk item in the project, moving to a platform where you don't control the camera driver is a bad trade for $40.

**Why not Jetson Orin Nano:** $249 for the dev kit, bulky carrier, 7–25 W. Would enable CUDA-accelerated VIO (Isaac ROS cuVSLAM: 3.8 ms per stereo-inertial call on AGX Orin vs ~41 ms for OpenVINS on Pi 5), learned front ends, real-time detection. The right board *after* the integration is understood by hand — jumping straight to Isaac ROS means learning to configure NVIDIA's stack rather than learning why the seams exist.

### Why not an integrated-IMU camera (OAK-D, RealSense)

The premise was that hardware-synced camera + IMU on one device removes the sync problem. It doesn't, here.

- **Spectacular AI SDK is free only on x86.** ARM requires a commercial license. On a Pi 5 the turnkey path is closed, so you're integrating OpenVINS by hand anyway — which is exactly what the cheap camera requires.
- **BMI270 timestamp jitter.** Constant offset is fine (estimator solves it). Polling-mode stamp-on-read produces *variable* error — the unmodelable kind. Documented in DepthAI issues; "timestamps no longer perfectly even."
- **75 mm baseline** makes stereo depth meaningless past 10–15 m. At outdoor cruise altitude everything is effectively at infinity and stereo VIO degenerates to monocular with extra compute.
- **TF-Luna already anchors metric scale**, which is the specific problem stereo would have solved.
- 61 g vs 18 g, plus USB3 draw on a Pi already asking for 5 A.

RealSense D435i is the cleanest integrated option (BMI055, hardware-aligned timestamps) but is ~$874 and largely unavailable since RealSense spun out of Intel in July 2025.

---

## Platform constraints

### F405 firmware — the binding constraint

STM32F405, 1 MB flash. ArduPilot's own non-GPS navigation docs state that standard firmware for 1 MB boards omits many AHRS and sensor features used for non-GPS position estimation. Confirmed by pulling `features.txt` from `firmware.ardupilot.org/Copter/stable/speedybeef4v4/`.

**Compiled out, needed:**
```
!HAL_VISUALODOM_ENABLED        ← driver that receives VISION_POSITION_ESTIMATE
!EK3_FEATURE_EXTERNAL_NAV      ← EKF3 fusion path that consumes it
```
Both are required. Enabling either alone gets nothing — a driver that parses a message an estimator ignores, or an estimator that could fuse vision with no way to deliver it.

**Compiled out, dropped:**
```
!AP_OPTICALFLOW_ENABLED  !HAL_MSP_OPTICALFLOW_ENABLED
!EK3_FEATURE_OPTFLOW_FUSION  !MODE_FLOWHOLD_ENABLED
!AP_QUICKTUNE_ENABLED    ← use Autotune instead (always compiled in)
!AP_SCRIPTING_ENABLED  !AP_OAPATHPLANNER_ENABLED
!HAL_PROXIMITY_ENABLED  !AP_DDS_ENABLED  !MODE_GUIDED_NOGPS_ENABLED
```

**Already enabled, useful:**
```
AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED   ← TF-Luna works on stock firmware
HAL_NAVEKF3_AVAILABLE
AP_RCPROTOCOL_CRSF_ENABLED  HAL_CRSF_TELEM_ENABLED
AP_GPS_UBLOX_ENABLED
AP_COMPASS_IST8310_ENABLED  AP_COMPASS_IST8308_ENABLED  AP_COMPASS_HMC5843_ENABLED
AP_COMPASS_QMC5883L_ENABLED  AP_COMPASS_AK09916_ENABLED  AP_COMPASS_LIS3MDL_ENABLED
AP_COMPASS_RM3100_ENABLED  AP_COMPASS_BMM150_ENABLED   # only !QMC5883P is excluded
```

**Enabled and cuttable to make room:** entire OSD stack, VTX control (`AP_VIDEOTX`, `AP_SMARTAUDIO`, `AP_TRAMP`), all four camera symbols, every telemetry protocol except CRSF, every RC protocol except CRSF, non-quad frame types, `AP_AIRSPEED_ENABLED`, `HAL_PARACHUTE_ENABLED`, `AP_LANDINGGEAR_ENABLED`, `MODE_TURTLE`, `MODE_FLIP`, surplus compass backends, NMEA GPS drivers, unused rangefinder variants.

**Build target name:** `speedybeef4v4` — all lowercase, and "F4v4" not "F405v4". Most SpeedyBee boards use CamelCase, so on a case-sensitive ASCII sort this appears at the **bottom** of the board dropdown on `custom.ardupilot.org`, past everything starting with a capital. Do not confuse with `speedybeef4v5` (different RC input pin: V4 on UART2, V5 on UART6) or `SpeedyBeeF405AIO`.

The build server fails if the feature set doesn't fit, so the test is empirical.

**Consequence for architecture:** the F405 is the ceiling, not the Pi. Treat ArduCopter as a stabilization and setpoint-tracking layer, drive it with Guided-mode setpoints from the Pi over MAVLink, and keep all autonomy on Linux where there's no flash constraint. This is also how production systems are built.

### The 1 MB firmware question, answered by building it (2026-09-03)

`custom.ardupilot.org` is a web form; the question here is quantitative, so the
build was done locally against the `~/ardupilot` checkout already on the sim VM
(4.6.0-beta1+8209, the same tree SITL runs). `ardupilot/build_speedybeef4v4.sh`
reproduces it; the feature sets are the `.dat` files beside it.

The board declares `FLASH_SIZE_KB 1024` with `FLASH_RESERVE_START_KB 48`, so the
application has ~976 KB. waf reports free flash against that, and an over-large
build fails outright with `region 'flash' overflowed` rather than truncating.

| variant | flash used | free | vs stock | builds |
|---|---|---|---|---|
| stock `speedybeef4v4` | 888,492 B | 110,920 B | — | yes |
| **+ visual odom + EKF3 external nav** | 899,516 B | **99,896 B** | **+11,024 B** | yes |
| + rangefinder frontend explicit | 899,524 B | 99,888 B | +11,032 B | yes |
| + full optical flow stack | 926,484 B | 72,928 B | +37,992 B | yes |

**Everything this project needs fits, with 98 KB to spare.** Visual odometry and
external nav together cost 11 KB — about 1 % of flash.

**The real constraint was never flash, and the note this file carried was
misleading about which.** Both features are compiled out on this board by
*source default*, not by anything in the board's hwdef:

```
HAL_VISUALODOM_ENABLED     HAL_PROGRAM_SIZE_LIMIT_KB > 1024
EK3_FEATURE_EXTERNAL_NAV   EK3_FEATURE_ALL || HAL_PROGRAM_SIZE_LIMIT_KB > 1024
```

`1024 > 1024` is false, so a 1 MB board gets neither — regardless of how much
room is actually left. Confirmed against the stock build: no `VISO_` parameters.
A custom build is required, but for a threshold reason, not a capacity one.

**The rangefinder is free.** `minimize_fpv_osd.inc`, which this board includes,
sets `AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED 0` and then re-enables the Benewake
TF02 / TF03 / TFMINI / TFMINIPLUS drivers explicitly. The TF-Luna's driver is
already in the stock firmware; making the frontend explicit cost 8 bytes.

**And the optical-flow rejection needs correcting.** The Rejected options table
gave two reasons for dropping the Matek 3901-L0X, and the first was flash:
"four features to re-enable, all competing for flash needed by external nav."
They do not compete — all four build alongside external nav with 71 KB still
free. The second reason (low-altitude-only, contributes nothing at cruise) is
untouched by this and is sufficient on its own, so the decision stands; but it
should stand on the reason that is true.

### Flashed, and two things this tree changes (2026-09-09)

`vio_full` is on the board. 899,524 B used, 99,888 B free — the 09-03 numbers
reproduced exactly, from the same tree
(`ArduPilot-4.6.0-beta1-8209-g14f70f1028`, clean, one clone in the reflog).
It reports itself as **ArduCopter V4.8.0-dev**: this is master, not a stable
release, chosen because it is the tree SITL flew milestones 4-6 on. Matching the
simulator that produced the drift numbers beats matching a release tag.

Verified in the order that each step actually proves something:

- **USB descriptor** reads `ArduPilot` / `speedybeef4v4` (VID 0x1209, PID
  0x5741) — not Betaflight's. This alone proves the firmware swapped, without
  opening a MAVLink connection.
- **QGC**: V4.8.0-dev, ICM42688 on SPI1, SPL06 on I2C0.
- **`VISO_TYPE` present.** The gate. Stock `speedybeef4v4` has no `VISO_`
  parameters at all, because both features are compiled out by source default on
  a 1024 KB board; their presence is proof that the custom build, and not a
  stock one, is what is running.

**The `.apj` could not have done this flash.** A board arriving from Betaflight
has no ArduPilot bootloader to receive one, so the first flash must be
`arducopter_with_bl.hex` over DFU. What worked, from the Mac:

```
dfu-util -a 0 -d 0483:df11 -s 0x08000000:mass-erase:force:leave -D arducopter_with_bl.bin
```

`mass-erase` because Betaflight's stored config sits in the flash sectors
ArduPilot uses for parameter storage. dfu-util writes raw binaries rather than
Intel hex, so the hex was converted with `arm-none-eabi-objcopy -I ihex -O
binary --gap-fill 0xff` and then checked rather than assumed: bootloader
(14,272 B) at 0x08000000 byte-identical to `speedybeef4v4_bl.bin`, application
(899,536 B) at 0x0800C000 byte-identical to `arducopter.bin`, 34,880 B gap all
`0xff`, total 948,688 B, sha256 `9400e328…`.

**waf omits `_with_bl.hex` silently when the python `intelhex` module is
missing.** `HAVE_INTEL_HEX` is evaluated at *configure* time; without the module
the hex task never registers, the build still succeeds, and the one artifact
that can flash a board coming from Betaflight simply is not produced. Nothing
warns. That is why the 09-03 build directory held only `.apj` and `.bin`, and it
is the same failure mode as every other entry in this file: an operation
reported success and did less than it claimed. `build_speedybeef4v4.sh` now
names the missing file and the reason, and no longer exits 1 on a successful
build (its feature-check loop died on the first symbol left at its source
default, because a failing `grep` under `set -eo pipefail` took the script with
it — so the `<source default>` fallback it printed could never actually print).

**`ARMING_CHECK` does not exist on this tree.** It is replaced by
`ARMING_SKIPCHK`, with the sense inverted: a bitmask of checks to *skip*,
default 0, rather than a bitmask of checks to run. **Bit 18 is VisualOdometry** —
the check behind milestone 4's `PreArm: VisOdom: not healthy`. Any instruction
to "set `ARMING_CHECK`" is now a no-op that writes nothing at all. QGC's
parameter metadata still expects it and raises a dialog on connect saying so;
the dialog is benign and names exactly this parameter. Nothing in `harness/`
ever set it, so no sim result is affected.

### Weight and thrust budget

| Component | Mass (g) |
|---|---|
| Frame (Source One V5) | ~120 |
| Motors ×4 | ~128 |
| FC + ESC stack | ~35 |
| Props | ~12 |
| Receiver | ~1 |
| Wiring/hardware | ~30 |
| **Dry subtotal** | **~340** |
| Pi 5 + Active Cooler | ~58 |
| Camera (32×32 board + M12 lens) | ~15 |
| IMU | ~3 |
| GPS/compass | ~15 |
| TF-Luna | ~5 |
| Printed tray + mounts | ~35 |
| **Payload subtotal** | **~140** |
| Battery 4S 1500 / 2200 | 180 / 250 |
| **AUW** | **~660 / ~730** |

Thrust: 2306 1700 KV on 5×4.8×3 at 4S ≈ 1000–1200 g/motor → ~4400 g total.
Hover throttle = √(660/4400) ≈ **39%**; √(730/4400) ≈ **41%**. Both inside the 40–50% target for controllable low-speed perception flying.
Endurance: **8–11 min** including Pi draw.

Compute: Pi 5 runs OpenVINS at ~41 ms/frame ≈ **24 Hz** (SMF-VO benchmarks, arXiv 2511.09072). VINS-Fusion 36.4 ms/frame.

---

## Software architecture

```
Camera (CSI) ──┐
               ├─→ Pi 5: OpenVINS/VINS-Fusion ──→ ENU→NED bridge ──┐
IMU (SPI) ─────┘                                                    │
                                                        MAVLink 921600 UART
                                                                    │
                                                                    ▼
                                          F405: VISION_POSITION_ESTIMATE → EKF3
                                                     ↑
                                     GPS/compass, TF-Luna (I2C)
```

**ArduPilot parameters:**
```
VISO_TYPE = 1                  # MAVLink visual odometry
VISO_POS_X / Y / Z             # camera offset from IMU (lever arm)
VISO_QUAL_MIN                  # reject low-quality vision rather than fusing it
EK3_SRC1_POSXY = 3 (GPS)       # primary source set — GPS for validation phase
EK3_SRC2_POSXY = 6 (ExternalNav)
EK3_SRC2_VELXY = 6
EK3_SRC2_YAW   = 6 or 1 (Compass)
RCx_OPTION = 90                # EKF source set switch
RCx_OPTION = 80                # Viso Align — re-aligns camera yaw to AHRS pre-flight
```

`RCx_OPTION = 80` is not optional: ArduPilot docs are explicit that skipping Viso Align can cause loss of position control ("toilet bowling"). Make it part of the pre-flight routine from the sim phase onward.

**Frame conventions are the #1 integration bug.** ROS/VIO convention is ENU or FLU (REP-103); `VISION_POSITION_ESTIMATE` expects NED. This class of error is why the validation phase runs with GPS enabled.

### UART allocation (F405 V4)

From the board's `hwdef.dat`, not the silkscreen. `SERIALn` follows position in
`SERIAL_ORDER`, which is why the numbering is worth reading off the source once:

| `SERIALn` | Port | Pads | DMA | Board default | Assignment |
|---|---|---|---|---|---|
| 0 | OTG1 | USB | — | MAVLink2 | bench GCS |
| 1 | USART1 | PA9 / PA10 | no | DJI FPV | free |
| 2 | USART2 | PA2 / PA3 | **yes** | RCIN | **ELRS EP2, CRSF 420000** — already the default; no parameter change needed |
| 3 | USART3 | PC10 / PC11 | no | none | GPS, uBlox |
| 4 | UART4 | PA0 / PA1 | no | none | free |
| 5 | UART5 | PD2, **RX only** | no | ESC telemetry 19200 | cannot carry a bidirectional link |
| 6 | USART6 | PC6 / PC7 | **yes** | GPS | **Pi 5, MAVLink2 921600** |

**Only USART2 and USART6 have DMA**, and the two highest-rate consumers are CRSF
at 420000 and the Pi's MAVLink at 921600. CRSF has no say in the matter —
USART2 *is* the RCIN pin pair. So the Pi takes USART6 and the GPS moves to a
NODMA port.

**This reverses the build plan's "Pi ↔ UART3".** USART3 is NODMA, and 921600 of
vision data is the last stream on this aircraft that should be interrupt-driven
a byte at a time. The GPS tolerates NODMA far better; if PPK raw observables
(RXM-RAWX) end up flowing through the FC rather than being logged on the Pi,
re-check that judgement, because it is the one thing that could make the GPS
link the heavy one.

TF-Luna on I2C rather than serial specifically to free a UART.

---

## Sim harness

**Architecture:** ArduPilot SITL + Gazebo Harmonic via `ardupilot_gazebo`, ROS 2 for perception.

**Two design decisions worth recording:**

*Gazebo, not the orthoimagery-warp approach used for the fixed-wing TRN work.* VIO needs genuine 3D structure. A textured ground plane produces coplanar features — a known monocular degeneracy — and you'd accidentally design around a failure mode that doesn't exist in the sim.

*IMU from Gazebo's sensor, not from ArduPilot over MAVLink.* This mirrors the hardware architecture, where the ISM330DHCX lives on the companion computer's clock alongside the camera. Same seam, so the sim tests what will actually be built.

**Milestones:**
1. SITL + Gazebo, quad flies Guided with GPS
2. Camera and IMU topics at sane rates with correct timestamps
3. VIO produces a trajectory, logged only, compared to Gazebo truth via `evo`
4. **VIO into ArduPilot with GPS still on** — the milestone that matters most; free frame-convention checking
5. GPS disabled mid-flight, EKF3 holds position on vision alone — **done 2026-09-03. 3/3 flights complete the mission denied, net drift 0.38–1.73 % of distance flown on vision. Needed fixed yaw AND altitude back on the barometer; vision driving the altitude controller was crashing the aircraft.**
6. Headless batch runs across altitude/speed/texture, ATE and RPE out — **done 2026-09-03, `harness/sweep.sh`; results below**

**ROS 2 launch arguments silently override YAML.** `subscribe.launch.py` always passes `max_cameras`, `use_stereo`, and `save_total_state` as ROS parameters, using its own defaults when not specified — and ROS parameters beat config-file values. A mono config with `max_cameras: 1` still tries to load `cam1` and dies with "unable to parse all parameters". Launch the sim config as:

```
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=$HOME/vio_openvins/gz_sim/estimator_config.yaml \
  max_cameras:=1 use_stereo:=false rviz_enable:=false
```

Whenever a config setting appears to have no effect, check whether a launch file passes that same name.

**Environment trap — ROS shadows Gazebo.** Sourcing ROS 2 Jazzy breaks the standalone `gz sim` CLI. ROS ships *vendored* gz libraries (`gz_transport_vendor`, `gz_msgs_vendor`, `gz_math_vendor`) and points `GZ_CONFIG_PATH` at them; those vendor packages contain no `gz-sim`, so the `sim` subcommand silently disappears and `gz sim` prints its help text instead of an error. Run Gazebo and `ros_gz_bridge` in **separate shells** — Gazebo without ROS sourced, the bridge with it.

**Sensor rates look wrong and aren't.** Gazebo sensors publish in *sim* time, so wall-clock rate is RTF × configured rate. At ~0.5 RTF a 200 Hz IMU measures as ~105 Hz on `ros2 topic hz`. Timestamps are correct, so the estimator is unaffected.

**The trap:** sim VIO is misleadingly easy. Perfect global shutter, no motion blur, no vibration, exact camera-IMU timestamps. A working sim proves the plumbing and proves nothing about the front end. Mitigation: put realistic noise and bias random walk on the Gazebo IMU in the SDF, add camera noise, and **inject a known 30 ms camera-IMU time offset to confirm online `td` estimation recovers it.** If a synthetic offset can't be detected in sim, the real one won't be either.


### Validated baseline — OpenVINS on EuRoC (2026-08-27)

Ran before trusting the estimator on our own data. Known data, known answer.

Two runs, identical code and data:

| Metric | run 1 | run 2 |
|---|---|---|
| ATE RMSE (translation) | 0.115 m | **0.067 m** |
| ATE mean / median | 0.108 / 0.101 m | 0.059 / 0.056 m |
| RPE @ 10 m | 0.072 m (**0.72 %**) | 0.080 m (**0.80 %**) |
| Umeyama scale correction | 1.000 | 1.000 |

Published OpenVINS on the easy EuRoC sequences is roughly 5–9 cm ATE. Run 2 lands inside that; run 1 is slightly outside. Scale of exactly 1.000 confirms metric observability, as expected with an IMU in the loop; a monocular-only system would need `sim3` alignment and report a scale factor.

**The spread is the finding, not the mean.** 0.067 vs 0.115 m — a factor of 1.7 — from identical inputs. Cause is almost certainly VM timing: when the estimator momentarily lags, frames are dropped, and which ones vary per run. Two implications for the evaluation harness, which is the actual deliverable:

- **Report distributions over repeated trials, never a single ATE.** A one-shot number from this setup carries roughly ±40 % of noise and would let us "improve" or "regress" the estimator by chance.
- **Log dropped-frame counts per run** so timing-induced variance can be separated from genuine estimator behaviour. Without that, VM jitter is indistinguishable from an algorithmic change.

**This is the reference point.** Any later drift number from our own platform is interpretable only against it.

**Environment:** OpenVINS on ROS 2 Jazzy needs its deprecated `.h` includes ported to `.hpp` (`image_transport`, `tf2_geometry_msgs`, `cv_bridge`) — Jazzy deleted the aliases. Our clone is patched; a fresh clone will not build.

**Procedure:** `harness/run_euroc_eval.sh` (verified end-to-end 2026-08-27). Three things that cost real time and are not obvious:

- **Take the trajectory from `/ov_msckf/odomimu`, not `save_total_state`.** The state file is `timestamp q p v bg ba …` — quaternion before position, JPL convention, storing `q_GtoI`. Feed it to a TUM-format reader and you get ~135° orientation error and a plausible-looking wrong answer. `nav_msgs/Odometry` has documented, unambiguous conventions.
- **Record with `-s sqlite3`.** An mcap bag whose recorder is killed before finalizing fails to open with "file end magic is invalid." `ros2 bag reindex <dir> -s <storage>` rebuilds a missing `metadata.yaml`, but cannot repair an unfinalized mcap.
- **Neither `pkill` form works by default.** `pkill -f` matches the shell issuing it, when the pattern appears in that shell's own command line — which killed a live SSH session. `pkill -x` avoids that but silently matches nothing for anything with a long name: `/proc/<pid>/comm` truncates to 15 characters and `run_subscribe_msckf` is 19. Use `-f` from **inside a script file** (where the pattern is in the file, not the command line), which is what `run_sim_vio.sh` does.

### Sim scene realism — the black-image investigation (2026-08-31)

Three consecutive harness runs flew a full mission and recorded **zero** odometry messages. The camera images were near-black, which reads unmistakably as "the renderer is broken" — and that read was wrong, twice, in two different ways. Both are recorded because the wrong diagnosis cost more than the fix.

**Separate the renderer from the scene before touching either.** `gazebo/worlds/render_probe.sdf` is a vehicle-free, SITL-free world: a static camera 15 m up looking down at three 4 m patches of known albedo (0.05 / 0.50 / 0.95), plus the same surfaces the flight world uses. It starts in ~25 s instead of several minutes. The patches came out at **11 / 112 / 214** — linear in albedo and exactly as specified. That single measurement exonerates the whole render path (Ogre2, llvmpipe, GLX, Xvfb, lighting, materials) and says the fault is in the scene. Keep the probe: any future "the camera is black" gets one cheap, decisive answer.

**The actual defect was texture spatial scale, not rendering.** The runway model's 2K albedo map is stretched across a 1500 × 100 m plane. Ground sample distance for our camera is

```
GSD = 2 · alt · tan(hfov/2) / width = 2 · 10 · tan(0.698) / 640 = 2.6 cm/px
```

so at altitude the entire visible patch of ground minified to a couple of texels. On the frame from the failed flight, 97.6 % of pixels sat on two adjacent grey values and **91 % of 16×16 blocks had a local standard deviation below 1.0**. A KLT tracker needs a local gradient. Zero gradient is zero features, and it is visually indistinguishable from a lighting failure.

Measured on the probe, which isolates the two surfaces:

| Scene | flat blocks | verdict |
|---|---|---|
| Failed flight frame (runway only) | 91 % | zero odometry messages |
| Obstacle field, untextured ground | 62 % | ground contributes nothing |
| Obstacle field + tiled ground | **12 %** | trackable |

Fix: `gazebo/worlds/make_ground.py` generates a visual-only tiled ground sized to the *camera* rather than the world — 10 m tiles carrying 512 px textures, 1.95 cm/texel, just under the GSD. Four variants, standardised to a common mean and standard deviation rather than normalised individually, because per-image normalisation made the tile boundaries into perfectly straight edges on a regular 10 m grid: strong features that exist nowhere in reality and repeat, which is an invitation for false correspondences. Flat blocks dropped **62 % → 12 %**.

`gazebo/worlds/make_feature_field.py` generates the depth structure, addressing the coplanar-feature degeneracy noted above. Both are generators, not hand-written SDF, so scene structure is an *experiment variable*: rerun with a different `--count` or height range and measure how drift responds to how much 3D the scene actually has. The camera is mono8, so the albedo that varies is **lightness** — two objects with different hues and equal luminance are the same grey and there is no edge between them.

**A local contrast measure is the right gate, not mean brightness.** `min`/`max`/`mean` all looked plausible on frames that were useless (max was 255 — from 954 pixels of blown highlight out of 256 000). The fraction of image blocks with near-zero standard deviation predicts trackability; brightness does not. `harness/check_camera.py` now reports `flat_blocks=NN%` and the harness refuses to fly above 40 %.

**Pace missions on vehicle state, never on a wall clock.** Software rendering runs Gazebo at RTF ≈ 0.2 (measured: a 200 Hz IMU arrives at 39 Hz wall, a 30 Hz camera at 5.8 Hz). Every `time.sleep(12)` in the mission script bought 2.4 s of flight, so the next waypoint was commanded while the vehicle was still most of the way from the last one — it chased a moving target and flew nothing resembling the intended square. `fly_sim_mission.py` now waits on `LOCAL_POSITION_NED` and `EKF_STATUS_REPORT`, with wall-clock values used only as failure backstops. This is correct at any RTF and strictly better than sleeping even at 1.0.

**The camera check is a hard gate.** A featureless image means the mission is wasted wall time, and at RTF 0.2 that is expensive. `run_sim_vio.sh` now aborts before flying unless `ALLOW_BLACK=1`.

### Two silent failures in the sim plumbing (2026-08-31)

Both produce well-formed output that is wrong, which is worse than an error.

**ArduPilot's lockstep freezes Gazebo before the vehicle flies.** `ardupilot_gazebo` runs the sim in lockstep with SITL. From the moment SITL connects until it is actually driving the motors, Gazebo's update loop is blocked — measured directly: `/world/<w>/stats` published **nothing across a 10 s window**, and no camera frames came out either. Free-running without SITL, the same world does RTF ≈ 0.4 and delivers its first frame **1.6 s** after a subscriber appears.

This is what made the render look broken. A camera check placed after SITL start reports a dead camera on a sim that is merely paused, and a full mission then flies with the first frames arriving only once the vehicle is airborne. **Any sensor check has to run before SITL starts**, which is now the order in `run_sim_vio.sh`.

**Ground truth from `dynamic_pose/info` records thousands of useless messages without erroring.** gz populates entity names and a header stamp on that topic — verified with `gz topic -e`. `ros_gz_bridge`'s `Pose_V → tf2_msgs/TFMessage` conversion delivers every transform with **empty parent and child frame ids and stamp 0**. A recorded bag looks healthy: 3006 messages, right type, plausible translations. There is no way to tell which body any of them describes, or when.

Fix: attach `gz-sim-odometry-publisher-system` to the model in the world file and bridge `nav_msgs/Odometry`. Correct sim timestamps, and the same message type as `/ov_msckf/odomimu`, so both sides of the comparison go through identical code — a format bug that hits both equally is far easier to catch than one that quietly biases only the reference.

**But `<robot_base_frame>` on that plugin is a label, not a selector.** It names the frame on the outgoing message; the pose published is always the *model's*, whichever link is named. Measured: the plugin reported z = 0.195 (model origin) where `vio_link` sits at z = 0.215.

That 0.102 m offset is not ignorable. It is comparable to the entire EuRoC-baseline ATE (0.067–0.115 m), and because it is a **body-frame lever arm it survives evo's Umeyama alignment** — alignment removes one global rigid transform, not a rotation-dependent offset — so it would appear as yaw-correlated error and read as estimator drift. `gt_to_tum.py --lever 0.10,0,0.02` composes it. Re-measure after any model change with:

```
gz topic -e -t /world/iris_runway/pose/info -n 1
```

which reports each link's pose in its parent model's frame.

### Real-time factor is a budget, and the scene spends it (2026-08-31)

Measured on the field world, Δsim over Δreal:

| Configuration | RTF |
|---|---|
| Feature field only, no camera subscriber | 0.75 |
| + 81 textured ground tiles | 0.40 |
| + camera subscribed (Ogre2 actually renders) | 0.20 |

Two things worth knowing. **Rendering is only half the cost** — the ground tiles halved RTF before any pixel was drawn, purely as scene-graph load, despite being visual-only with no collision. And **Gazebo's camera sensors are lazy**: with nothing subscribed to the image topic, nothing renders. Any RTF measured without a subscriber is optimistic by 2×, which is easy to fool yourself with.

This matters beyond wall-clock patience, because `ardupilot_gazebo` runs in lockstep (`<lock_step>1</lock_step>`) and the coupling is not robust to a slow sim. When it breaks, the plugin logs `Drained n packets: 142` / `Missed 143 input frames` — ArduPilot running *ahead* and flooding, not starving — Gazebo's clock stops, and it does not recover even after SITL is killed. The vehicle never arms and the mission times out with no obvious cause.

RTF alone is not the whole story: a lighter world completed a mission at RTF ≈ 0.19 while the field world froze at ≈ 0.20. Whatever the precise trigger, headroom is the defence, so scene cost is worth spending deliberately rather than accidentally.

### The recurring bug: assuming an operation finished (2026-08-31)

Three separate failures in this harness, each initially blamed on something else, were the same mistake — issuing an operation and proceeding as if it had completed.

| Where | Assumed | Actually |
|---|---|---|
| `sleep 35` after starting SITL | It is up | Recompiles for 3–4 min on a new frame |
| `time.sleep(12)` between waypoints | Vehicle arrived | 2.4 s of flight at RTF 0.2; it never arrived |
| `pkill` then `sleep 1` before restarting Xvfb | Old server is gone | Still dying; it deleted `/tmp/.X11-unix/X77` *after* the replacement claimed it |

The last one is the most instructive, because the symptom pointed somewhere else entirely: Gazebo died mid-run with `XIO: fatal IO error on X server ":77"`, the camera produced nothing, and the obvious reading was a broken display stack. It cost two runs and was only found by noticing that a leftover process from unrelated manual testing was present both times.

`pkill` delivers a signal; it does not wait. Every teardown in `run_sim_vio.sh` now kills, polls until the processes are actually gone, escalates to `-9`, and removes the stale X lock and socket. The corresponding rule for startup is already in place: poll for the condition (`wait_for_port`, `xdpyinfo`, `LOCAL_POSITION_NED`) rather than sleeping a guessed interval.

**A sleep is a guess about someone else's completion time.** Where the thing being waited on can report its own state, wait on that instead.

### Lockstep was never on, and that silently swapped the EKF (2026-08-31)

The most consequential finding so far, and it was not the one being chased.

`iris_with_vio` is copied from `iris_with_ardupilot`, which upstream ships with **both** `<lock_step>1</lock_step>` and `<no_time_sync>1</no_time_sync>` (ardupilot_gazebo 65937b7). They contradict each other and `no_time_sync` wins. The plugin sends it in its JSON, ArduPilot obeys — `SIM_JSON.cpp` prints `Forcing use_time_sync=0` — and then gates lockstep on the same flag:

```cpp
if (use_time_sync && !state.no_lockstep) { adjust_frame_time(...); }
```

So `lock_step` did nothing in any run. ArduPilot free-ran, flooded the plugin (`Drained n packets: 142`), and Gazebo's clock stopped.

**The part that matters is three lines further down:**

```cpp
if (!use_time_sync) {
    // if not using time sync then default EKF type to 10, as
    // otherwise EKF is likely to diverge
    AP_Param::set_default_by_name("AHRS_EKF_TYPE", 10);
}
```

EKF type 10 is the SITL *fake* EKF: it reads state straight from the simulator. **Milestones 4 and 5 exist to test what EKF3 does when fed vision, and against type 10 they pass while proving nothing.** A GPS-denied hold would have looked perfect because the "estimator" was reading ground truth.

Fix: `<no_time_sync>0</no_time_sync>`. Real lockstep means ArduPilot waits for each Gazebo frame, so a slow sim costs wall-clock time and nothing else — which is fine, because the mission is paced in sim time.

**The near-miss worth remembering.** The route here was an apparent fix: `SIM_SPEEDUP 0.2` made the EKF converge in 11 s of sim time where it had been hanging indefinitely, and the plugin's backlog shrank instead of growing. It looked like the answer. It was not — setting `SIM_SPEEDUP != 1` is another way to disable time sync, so the "success" was ArduPilot no longer waiting for the sim *and* quietly switching to the fake EKF. The vehicle then armed and never climbed, because with time sync off it also gets no FDM at all (`No JSON sensor message received, resending servos`). Had it climbed, this would have produced a plausible drift number measured against ground truth wearing an EKF costume.

`fly_sim_mission.py` now reads back `AHRS_EKF_TYPE` after the EKF is ready and aborts if it is 10. A failure this quiet needs a specific assertion; nothing else in the pipeline would have caught it.

### Answered: gz `<stddev>` is per-sample, not a noise density (2026-08-31)

Measured from a recorded flight, over 7216 samples while the vehicle sat disarmed on the runway (`harness/count_features.py`'s sibling check, `imu_check.py` pattern):

```
accel mean (m/s^2):  x=+0.0081  y=+0.0067  z=+9.7914   |a| = 9.7914
gyro  stddev:        1.88e-04 rad/s   (SDF asked 1.6968e-04)
```

Gravity confirms the frame: a level FLU body at rest reads specific force `(0, 0, +9.81)`, and it does. No IMU frame bug.

The gyro answer is the ratio: measured 1.88e-04 against 1.6968e-04 asked, so **gz applies `<stddev>` per sample**. Our SDF put EuRoC *densities* there, so the simulated IMU is quieter than intended by √200 ≈ 14×:

| | Declared to OpenVINS | Actual sim IMU |
|---|---|---|
| gyro noise density | 1.6968e-04 | 1.33e-05 (12.8× lower) |
| accel noise density | 2.0e-03 | 4.70e-04 (4.3× lower) |

Two consequences, and the second is the one that bites:

- The sim IMU is **better than the hardware will ever be**, which is precisely the "sim VIO is misleadingly easy" trap noted above, arriving through a channel nobody was watching.
- The estimator is told a noise level that does not match its input. A filter given the wrong `R` is not merely conservative; it weights visual against inertial evidence incorrectly.

Fix, as predicted in the SDF's own comment: **`stddev = density × √update_rate`** — gyro `1.6968e-04 × 14.14 = 2.399e-03`, accel `2.0e-03 × 14.14 = 2.828e-02`. Both files must move together, and `kalibr_imu_chain.yaml` says so at the top.

The `dynamic_bias_stddev` terms are untested and presumably carry the same ambiguity; treat them as unverified until measured the same way.

### The estimator could not initialise, and the reason was 20 cm off the ground (2026-09-01)

OpenVINS ran a whole flight against good imagery and produced **zero** output, reporting `[init]: not enough feats to compute disp: 0,0 < 15` throughout and printing `TIME: 9223372036854.775 seconds` at shutdown — `INT64_MAX/1e6`, the state clock never set.

The imagery was not the problem, but proving that took building the right measurement. **The earlier gate measured the wrong quantity.** Block standard deviation is *gradient*; FAST needs a *corner* — a centre pixel differing from a contiguous arc of the 16 around it. Smooth multi-octave noise shades continuously in every direction, so every block looks textured while yielding almost nothing to track. `harness/count_features.py` now asks the question the front end asks, over the same 5×5 grid and `num_pts` budget:

| View | FAST corners @ thresh 20 | cells with features |
|---|---|---|
| On the runway, 0.215 m | 18 | 4 / 25 |
| Airborne | 184 | 23 / 25 |

At 0.215 m the GSD is 0.056 cm/px, so each 1.95 cm ground texel spans ~35 px — the ground texture is sized for the 2.6 cm GSD at 10 m and is a smooth blur up close. OpenVINS' **static** initialiser needs a stationary window *with* trackable features before motion begins. It never got one, and by the time features existed the vehicle was no longer stationary.

Fix: a 2 m × 2 m takeoff pad at 2048 px (0.098 cm/texel), generated alongside the tiles. Verified **before** spending a flight on it — 200 corners in 25/25 cells at ground height, altitude unchanged.

**Dynamic initialisation is not the fix, and its failure mode is worth recognising.** Setting `init_dyn_use: true` does produce a trajectory — 12182 poses where there had been none — and it is wrong: 1398 m off in y, path length 1448 m against 155.9 m of truth, ATE RMSE 400 m, "drift" 101 %. The diagnostic is the *shape* of the error. Altitude stayed bounded (2.7–18.9 m against a true 0.2–10.3 m) while one horizontal axis ran off in a near-straight line at ~23 m/s. Accumulated drift wanders; a constant velocity offset does not. That is a bad initial state, not a bad estimator.

**A negative result worth keeping.** The IMU noise mismatch found the same day looked like a strong candidate for the divergence. Re-running the identical bag with the declared densities corrected to the measured ones moved ATE from 399.5 m to 404.6 m and drift from 100.78 % to 101.09 % — nothing. It was a real bug and not this bug, and only the offline replay made that separable at all.

**Record raw sensors.** `RECORD_SENSORS=1` bags the camera and IMU (~5 GB per 10 minutes). It turns a 25-minute flight into a minutes-long replay, and because the input is byte-identical every time, a change in the output is a change you made rather than the 1.7× timing variance measured on EuRoC. Every estimator conclusion above came from replaying one flight.

### Estimator on sim data: what has been ruled in and out (2026-09-01)

The harness now flies, records, and evaluates end to end, and OpenVINS produces a trajectory. **The trajectory is not yet usable** — 9.4 m ATE, 45 % drift against a EuRoC baseline of 0.72–0.80 %. What follows is the state of the search, because the negative results are worth as much as the fixes.

**The first leg is already EuRoC-class.** Climb plus first 20 m straight: **0.3 m error over 19.5 m, 1.5 %**, altitude tracked to 1 m. So the whole chain — extrinsic, intrinsics, initialisation, scene, IMU — can produce good numbers. Something specific breaks at the **first turn**, after which the estimate keeps travelling in its original direction while truth turns (its y grows 29 → 57 m while truth holds at 19.9 m). That is a failure to register a change of travel direction, not accumulating noise.

**Fixed, each verified by measurement:**

| Change | Effect on ATE |
|---|---|
| Static init impossible → high-resolution takeoff pad | never initialised → initialises |
| 45 s of stationary running before takeoff → 8 s lead-in | 623 m → 10.1 m |
| `gravity_mag` 9.81 → 9.8, matching the world | 10.1 m → 9.4 m |

Gravity deserves a note: gz's default world gravity is **9.8**, not 9.81, and copying EuRoC's value was a 0.01 m/s² model error. That integrates twice — 3.1 m over 25 s on its own.

**Ruled out, on identical replayed input:**

| Hypothesis | Result |
|---|---|
| IMU noise densities wrong | corrected them: 399.5 m → 404.6 m, i.e. nothing |
| Camera–IMU extrinsic wrong | **verified correct by optical flow** (below) |
| Feature parameterisation | inverse depth was worse: 9.4 → 9.9 m |
| Online calibration destabilising | disabling it was far worse: 9.4 → 148.9 m |
| IMU bias random walk | measured wander 0.009–0.014 m/s² over 40 s; declared values only 2–5× high |

**The extrinsic is now verified, not asserted.** `harness/flow_check.py` measures optical flow against ground-truth velocity. Prediction from the derived transform: flying forward pushes ground features *down* the image, by `2·alt·tan(hfov/2)/width` per frame = 7.4 px at 10 m and 5.8 m/s. Measured: **+7.4 px, direction and magnitude**. With `det = +1` ruling out a mirrored axis, the transform is right. Worth keeping — PROJECT.md calls frame conventions the top risk precisely because a rotated camera yields a plausible *wrong* trajectory rather than an obvious failure.

**That online calibration HELPS by 15× is itself unexplained.** If the supplied extrinsic and intrinsics were exact, freezing them could not hurt. It pointed at the camera model, which measurement then exonerated; it also led to the gravity error, which online calibration had been partially absorbing. Something is still being compensated for.

**Next test, and it is the one this sim was built to answer.** At 10 m altitude the 0.4–6 m obstacle field subtends very little of the view, so the scene is effectively the plane PROJECT.md warned about from the start. Flying the same mission at ~4 m puts genuine 3D structure in frame and halves the GSD. That is a scene change rather than more tuning, and it tests the project's central assumption instead of another parameter.

> **Answered on 2026-09-02, and the answer is no on both counts.** The scene was
> never a plane — measured, 5.03 px of parallax at p95 — and flying at 4 m would
> have made it *flatter*, not richer, because the obstacles carry collision
> geometry and have to be capped for clearance. The real fault was a chi-squared
> gate that had the filter rejecting every feature it had. See the two sections
> below.

### The filter was rejecting every feature it had (2026-09-02)

**Result: drift 97.8 % → 15.4–16.0 %, ATE 237 m → 2.5 m, from two numbers in the
estimator config.** Reproduced on two independent flights, three replays each,
every repeat covering 96–98 % of a ~156 m flight. **The < 5 % gate for Phase 3
step 5 is not met**; what was fixed, what was ruled out, and what is left are all
below.

**The planar degeneracy — this project's founding hypothesis — is ruled out by
measurement.** `harness/parallax_check.py` tracks features between two views,
fits the best homography, and reports the residual. Between two views of a plane
a homography explains *every* pixel exactly, whatever the camera did, so the
residual is by construction the part of the image motion no plane can account
for. That is parallax, and parallax is what makes depth observable. Measured on
the structured 10 m flight, over 42 frame pairs during cruise:

```
resid_p50    0.21 px      <- the noise floor
resid_p95    5.03 px      <- the parallax actually available
frac_gt_1px  0.16         <- 16 % of features move >1 px from where a plane says
H inliers    0.91
```

Five pixels of parallax at p95 is genuine 3D structure, not a plane. Drift was
97.8 % anyway. The degeneracy PROJECT.md has warned about since the first
paragraph of the sim-harness section is not what has been wrong.

**What was actually wrong.** OpenVINS logs the number of features it uses per
update at DEBUG verbosity. Counting them:

| phase | MSCKF feats/update | SLAM feats |
|---|---|---|
| climb + first leg | 4.5–11 median | 34–50 |
| first corner | 4.5 | 49 → **3** |
| **remaining 47 s** | **0 — 695 of 709 updates** | **0** |

The visual update pipeline died 17 s into the flight and never recovered.
Everything after was pure IMU dead-reckoning. Meanwhile an independent KLT
tracker on the *same images* found 387 trackable features throughout that
period, with 91 % homography inliers. The imagery was fine. **The filter was
refusing it.**

The mechanism is a chi-squared outlier gate that is too tight, and its defining
property is that the failure is **self-locking**. Once a state becomes slightly
inconsistent, every residual looks like an outlier; every feature is rejected;
no update can correct the state; so the inconsistency is permanent. That is why
this presented as a sudden divergence at the first turn rather than the gradual
drift it was repeatedly assumed to be.

**The state trace shows it precisely.** From `save_total_state`, the
accelerometer bias:

```
t= 7.9-12.6   first leg flown perfectly, 20.2 m      ba_y = +0.013
t=13.0-14.6   1.6 s after the first stop             ba_y  0.013 -> -0.601
t=15.4-64.6   the remaining 50 s                     ba_y = -0.468, FROZEN
```

−0.47 m/s² is 30–50× the sim IMU's measured bias wander (0.009–0.014 m/s² over
40 s). A bias that large integrates to ~280 m over the flight, which is the
entire ATE. A bias frozen to three decimals for 50 seconds is a filter receiving
no information.

**The fix, and the honest half of it.** `up_*_chi2_multipler` 1 → 5 is the
dominant term; `up_*_sigma_px` 1 → 4 refines it. Measured on identical replayed
input:

| sigma_px | chi2 mult | drift |
|---|---|---|
| 1 | 1 | 97.8 / 98.0 / 99.4 % |
| 2 | 1 | 95.6 % |
| 1 | 5 | 25.3 % |
| 2 | 5 | 21.0 % |
| 3 | 5 | 18.3 % |
| **4** | **5** | **16.0 %** |
| 5 | 5 | 16.4 % |
| 8 | 5 | 16.6 % |

Noise alone fixes nothing; the gate is what matters. But raising `sigma_px` is
the more principled half, because unlike the multiplier it also correctly
reduces how hard each admitted measurement pulls the state. And 1 px was
genuinely wrong here: the vehicle crosses ~15 px of image between tracked frames
(30.3 Hz camera, `track_frequency` 21 keeping every second frame = **15.2 Hz
measured**, at 5.8 m/s over a 2.6 cm/px GSD), and KLT error grows with
displacement. 1 px was not conservative, it was a misdescription of the sensor.

Zero-feature updates over the last 47 s fell from 695/709 to 237/709.

**Ruled out along the way, all on the same replayed flight:**

| Hypothesis | Result |
|---|---|
| Monocular planar degeneracy | **measured 5.03 px of parallax at p95** — the scene is not a plane |
| Online calibration | freezing all three: 97.44 % vs 97.83–99.39 % baseline — no effect |
| ZUPT misfiring at waypoint stops | `has_moved_since_zupt` is set unconditionally after the first propagation; the log contains **zero** ZUPT events |
| Dropped IMU or camera samples | streams are exactly uniform: 200.01 Hz and 30.32 Hz, **zero gaps** |
| Zero-baseline clone window at the stops | the arithmetic is seductive — 11 clones at 15.2 Hz spans 0.72 s and the corner dwell is 0.5–0.8 s, so the window *does* collapse onto stationary poses — but widening it to 1.3 s and 2.0 s gave 97.6 % and 95.6 % |
| More features | 400 points / 75 SLAM slots with the gate open: **102.2 %**, far worse. The extra features are the marginal ones |
| Higher tracking rate | `track_frequency` 31 keeps every frame and halves inter-frame motion to 7.4 px: 16.3 % vs 16.0 %, no help |

**A correction to an earlier entry.** The 2026-09-01 note recorded that
disabling online calibration took ATE from 10.1 m to 148.9 m, and read that as
evidence of a wrong camera-IMU transform being absorbed by the filter. Freezing
all three now gives 97.44 % against a 97.83–99.39 % baseline — no effect. The
earlier measurement was real but of a different configuration, before the
gravity fix and on a different scene, and it pointed at a problem that was never
there. The "online calibration helps by 15×, and that is unexplained" open
question is closed: it does not.

**Where it stands, over two independent flights of the same mission:**

| flight | peak speed | path | drift (3 replays) | ATE RMSE |
|---|---|---|---|---|
| default speed | 8.17 m/s | 155.8 m | 15.89 / 15.97 / **15.99 %** | 2.43–2.49 m |
| `WP_SPD` 2.5 | 2.75 m/s | 156.8 m | 15.19 / 15.35 / **15.69 %** | 2.33–2.63 m |

**Speed is not the limiter, and that was the leading candidate.** Cutting peak
speed 3× cuts inter-frame image motion 3× — ~21 px to ~7 px between tracked
frames — which is exactly the quantity `up_msckf_sigma_px` = 4 was introduced to
absorb. It changed nothing: 15.4 % against 16.0 %, inside the ~1.3 %
flight-to-flight spread. Whatever remains is not tracking noise from image
motion.

**Nor is it the accelerometer bias being under-constrained.** The bias is what
absorbed the original failure, so tightening how fast it is allowed to move
looked promising. Measured on the slow flight, against 15.4 % at the declared
3.0e-3:

| accel random walk | drift |
|---|---|
| 1.9e-3 (the measured value) | 18.9 % |
| 5.0e-4 | 22.3 % |
| 1.0e-4 | 29.0 % |

Monotonically worse. The filter needs that freedom; constraining it is not the
fix.

**What the residual actually looks like.** ATE is 2.5 m over 156 m — **1.6 % of
path length** — while RPE over 10 m segments is 15 %. Globally right, locally
wrong. Per-leg, every direction is now correct to within a consistent ~176°
offset (the unobservable global yaw, which evo's alignment removes), but the
*scales* vary leg to leg: 0.92, 0.99, 1.09, 1.03, 1.36, 1.12, 0.81. That is not
noise and not a constant scale error — it is the metric scale being re-derived
differently on each leg. In a monocular filter scale comes from the IMU, so this
is a visual-inertial consistency problem that survives every parameter tried.

**Answered: no, a rangefinder cannot rescue this, and the reason is not the one you would guess.**
The obvious move when a monocular system drifts is to add a metric reference.
Measured on both flights, it does not apply here:

* **There is no scale error to correct.** Letting evo fit a scale factor
  (Sim(3), `-as`) instead of a rigid transform (`-a`) changes ATE by under 1 % —
  2.473 → 2.451 m and 2.626 → 2.605 m. The IMU already delivers correct metric
  scale, exactly as the EuRoC baseline showed with a Umeyama scale of 1.000.
* **The error is in the axis a downward rangefinder does not measure.**
  Splitting ATE after rigid alignment: horizontal 2.35 m / vertical 0.78 m on the
  fast flight (90 % / 10 % of squared error), and 2.58 / 0.50 m on the slow one
  (96 % / 4 %). Perfect altitude knowledge removes at most ~5 % of the ATE.
* **The TF-Luna is out of range anyway.** Its ceiling is 8 m and the vehicle is
  above 8 m for 64–75 % of the mission.

The rangefinder is still worth its place, for different jobs: as a **health check
on the estimator** (in the broken run the estimate climbed to 61 m while the
vehicle held 10 m — a rangefinder catches that immediately), for
**ArduPilot-level fusion** via `EK3_SRC*_POSZ`, which is already supported and
costs nothing, and for low-altitude work where it is actually in range. It does
not belong inside OpenVINS, which has no range input.

**A correction, because it affects a hardware decision.** The note above about
stereo originally justified it as removing "monocular scale ambiguity". That is
wrong, and this measurement is what disproves it: global scale is already right.
The accurate case for stereo is the leg-to-leg *magnitude* inconsistency
(0.81–1.36, which largely cancels globally) — stereo supplies metric depth per
frame rather than requiring the IMU to condition it across a sliding window.

**Honest position: the < 5 % gate is not met, and the next step is not another
parameter.** Six of them have now been swept with no effect or a negative one.
The things not yet tried are structural: a mission that does not stop and turn
at every waypoint (every failure in this investigation began at a corner), and
stereo — the Pi 5 has a second CSI port and $36 of hardware supplies metric
depth per frame instead of asking the IMU to condition it across a sliding
window, which is where the leg-to-leg magnitude inconsistency comes from.

### Two parameters that were never set, and one that was set to nothing (2026-09-02)

**`WPNAV_SPEED` does not exist in ArduCopter 4.8-dev.** The waypoint-nav
parameters were renamed and converted to SI: it is **`WP_SPD`, in metres per
second**, alongside `WP_ACC`, `WP_RADIUS_M`, `LOIT_SPEED_MS`, `RTL_SPEED_MS`.
Confirmed against a full 1380-entry dataflash parameter dump, which contains no
`WPNAV_*` at all.

Two consequences, and the second is worse than the first.

**The mission has been flying at up to 8.15 m/s, not 5.8 m/s.** `WP_SPD`
defaults to 10 m/s. Every inter-frame-motion figure derived from 5.8 m/s in this
file is therefore ~1.4× optimistic: the tracker sees ~21 px between tracked
frames at 10 m, not 15. That is the number `up_msckf_sigma_px` had to absorb.

**A whole flight was recorded, analysed and nearly reported as a slow flight
that was not slow.** `wait_param` returned success as soon as a `PARAM_VALUE`
with the right *name* came back — it never looked at the value, and for a
parameter that does not exist it reported success anyway. The run produced a
clean-looking distribution (12.12 / 12.31 / 12.33 % against 15.89 / 15.97 /
15.99 %) and the obvious reading was "flying slower helps by 25 %". It is not:
the ground-truth speed profiles of the two flights are identical —

```
fast (default)     p50 1.21  p90 6.30  p99 8.07  max 8.17 m/s
slow (WP_SPD 2.5)  p50 1.24  p90 6.30  p99 8.05  max 8.15 m/s
```

— so that 25 % is **flight-to-flight variation between two recordings of the
same mission**, not an effect of anything. `wait_param` now verifies the
read-back value and raises rather than returning False, because a parameter that
silently does not apply attributes every later conclusion to a change that never
happened.

**The variance result matters on its own.** Replay-to-replay spread on one
recording is 1.01–1.04×, measured three times. Flight-to-flight spread on the
same mission is ~1.3×. So the distributions this harness reports are only valid
*within* a recording; two flights are not comparable at the 25 % level, and any
scene or mission change must clear that bar before it means anything. This is
the ATE-span trap in a new costume, and the fix is the same: state what varied.

### Method notes that changed how this was found

**Count what the filter uses, not what it outputs.** Every previous round of this
investigation compared trajectories. The answer was in a number OpenVINS prints
at DEBUG verbosity and nobody had read: features used per update. Two lines of
grep separated "the estimator is drifting" from "the estimator has been blind
for 47 seconds", and those need completely different fixes.

**`subscribe.launch.py` owns four settings, not two.** PROJECT.md already
recorded `max_cameras` and `use_stereo`. Add `save_total_state` and `verbosity`:
both are declared as launch arguments with their own defaults, both silently
beat the YAML, and the launch file *hardcodes* the output paths
(`/tmp/ov_estimate.txt`), ignoring `filepath_est` entirely. `save_total_state:
true` in the config had been inert; the state trace above was unavailable until
it was passed on the command line.

**Measure the scene, don't argue about it.** The planar-degeneracy hypothesis was
three months old, load-bearing for the scene design, and had never been tested
against a single recorded image. It took one 60-line script and no flight.

### Altitude was the wrong knob, and the geometry says so (2026-09-02)

The plan on the table was to fly the same mission at ~4 m: "real 3D structure in
frame, and the ground sample distance halves." Both halves of that turn out to be
wrong here, and the arithmetic is worth keeping because it inverts the intuition.

**The sim camera is not the flight camera.** `/cam0/camera_info` reports
f = 381.347 at 640 x 400, so the simulated FOV is **80 deg horizontal**, not the
OV9281's 118 deg. Every altitude/footprint estimate made with the hardware number
is wrong by a factor of 1.7 in swath width. At 10 m the sim camera sees
16.8 x 10.5 m, not the ~30 m the hardware figure implies.

**The obstacles have collision geometry, and the mission altitude is coupled to
the scene through it.** Flying the 20 m square at 4 m over the existing field
means flying into objects up to 5.27 m tall, and there is one within 1 m of
*every* leg. So height has to be capped along the flight path, and how much can
be left standing under the vehicle is set by the altitude itself:

```
depth spread in frame  ~  alt / (alt - tallest object the vehicle can fly over)
```

Flying lower does not put more structure under the camera. It forces *shorter*
structure there. Measured from the generated SDF against the sim frustum:

| Configuration | depth in frame | spread | feature dwell @ 5.8 m/s |
|---|---|---|---|
| 10 m, old field (6 m objects, no corridor) | 4.0–14.1 m | 3.5:1 | 38 tracked frames |
| 4 m, corridor capped at 2 m | 2.0–5.6 m | **2.8:1** | **15 tracked frames** |
| 10 m, corridor capped at 7 m, field to 9 m | 3.0–14.1 m | **4.7:1** | 38 tracked frames |

So the 4 m test as specified would have made the scene *flatter* while making the
front end 2.5x harder — inter-frame flow rises from 10.5 px to 26.3 px per tracked
frame, and feature dwell falls by the same factor, because the footprint shrinks
with altitude and the speed does not. It would have confounded the hypothesis
with a harder tracking problem and answered neither.

The controlled version of the same hypothesis is the third row: **hold altitude,
speed, GSD and dwell fixed, and raise the scene's depth spread instead.**
`gazebo/worlds/make_scene.sh ALT` generates it, and `harness/field_clearance.py`
reports clearance and depth spread from the SDF before anything is flown. The
check is wired into generation rather than left as a step to remember.

### Three harness faults, each of which silently costs a flight (2026-09-02)

**SITL parameters persist across runs, and one of them blocks arming.** A flight
died with nothing but `[mission] FAILED to arm`. The reason was only in the
dataflash log: `PreArm: VisOdom: not healthy`. The previous Milestone-4 flight had
set `VISO_TYPE=2`, SITL keeps that in its `eeprom.bin`, and every later run
without the bridge inherits it — correctly refusing to arm, since nothing is
sending `VISION_POSITION_ESTIMATE`. The mission script set `VISO_TYPE` only in the
`--extnav` branch, so the off case was "whatever the last run left behind".

Fixes, and the second matters more than the first:
- every parameter the mission depends on is now set explicitly in **both**
  directions, never left inherited
- `arm()` prints the autopilot's own `STATUSTEXT`. ArduPilot explains every
  refusal; not surfacing it turned a one-line answer into a dataflash dig.

**`arm()` budgeted in wall clock.** 60 s of wall is ~12 s of vehicle time at
RTF 0.2 — the same mistake PROJECT.md already records for waypoint pacing,
surviving in the one function whose failure looks like a vehicle problem. Now
budgeted in sim time with a wall-clock backstop.

**`/tmp` does not survive a VM reboot.** Every recorded flight lives in
`/tmp/simvio_*`, and a restart takes all of them. Offline replay is the whole
basis of the estimator investigation — every ruled-out hypothesis was tested by
replaying one recorded flight — and that capability silently reset to zero. Any
bag worth re-replaying belongs outside `/tmp`.

### Evaluation traps, now enforced rather than remembered

Two of the documented traps were things to remember at analysis time, which is
where they had already caused wrong conclusions. Both are now checks:

- **`eval_sim_run.sh` reports trajectory span and coverage automatically**, and
  prints a loud warning below 90 %. A run that diverges and stops early scores
  *better* on ATE, so comparing two numbers over different spans is meaningless.
- **`replay_repeats.sh` refuses to produce a single number.** It replays one
  recorded flight N times and reports min/median/max plus the spread, because
  OpenVINS initialises on a background thread and identical input does not give
  identical output (1.7x measured on EuRoC).

`harness/analyze_run.sh` chains rebase -> measured start offset -> N replays ->
per-leg breakdown, so the whole path from a landed flight to a distribution is
one command with no remembered steps.

### The sim eats the host that runs it (2026-09-02)

The VM died mid-session with no guest-side trace, taking a completed flight
recording with it. It was not a guest fault and not a UTM bug: macOS Jetsam
terminated QEMU under memory pressure.

Measured on the host:

| | |
|---|---|
| data volume | 402 GB used of 460 GB — **96 % full, 19 GB free** |
| RAM / swap | 16 GB, swap 3.0 GB of 4.0 GB used |
| QEMU | the largest process on the machine (VM allocated ~7 GB) |
| prior evidence | a `JetsamEvent` report from the previous day |

Jetsam kills the largest process when it cannot satisfy demand, and it cannot
grow swap on a nearly-full disk. QEMU is by a wide margin the largest process,
so it goes first.

**The loop is self-reinforcing, and that is the part worth recording.** UTM
holds 101 GB of VM images (68 + 24 + 9). Every gigabyte a flight records inside
the guest inflates the host's disk image, consuming exactly the free space that
keeps the VM alive. A session of `RECORD_SENSORS=1` flights is therefore a slow
denial-of-service against its own host. Recording is not free the way "there is
19 GB free in the guest" makes it look — guest free space and host free space
are the same resource, seen twice.

Practical consequences: keep host free space above ~40 GB before a flight
session, prune `~/vio_runs` deliberately, and treat a VM that vanishes without a
guest-side log as a host-side kill rather than a sim fault.

### The 16 % is a bounded error, not a drift rate (2026-09-03)

Before reading anything into the headline number, it is worth asking whether it
is a *rate* at all. RPE over 10 m segments implicitly assumes error accumulates
with distance; if it does, the percentage is scale-free and comparing a 58 m
EuRoC sequence with a 156 m flight is fair. Measured across segment lengths on
the same trajectory:

| RPE delta | mean error | as % of segment |
|---|---|---|
| 1 m | 0.248 m | 24.8 % |
| 2 m | 0.456 m | 22.8 % |
| 5 m | 0.966 m | 19.3 % |
| 10 m | 1.597 m | **16.0 %** |
| 25 m | 3.456 m | 13.8 % |
| 50 m | 3.100 m | 6.2 % |

The percentage falls monotonically, and the absolute error stops growing past
25 m — 3.46 m at 25 m, 3.10 m at 50 m. Genuine drift holds a roughly constant
percentage, because the error grows with the distance. **In open loop this error
is bounded at ~3 m**, which is consistent with ATE 2.5 m over a 156 m path.
Quoting it as "16 % per 10 m" takes a bounded offset and divides it by an
arbitrarily short baseline.

**Why it is bounded, and the caveat.** The mission is a closed circuit — it
returns to (0,0) three times and to (20,20) twice — so long-lived SLAM features
are *re-observed*, and OpenVINS keeps up to 50 of them in the state.
Re-observation anchors the estimate, which is loop closure by another name. This
mission's numbers therefore overstate what a one-way traverse would do, and any
claim about GPS-denied range needs an outbound mission to support it.

**It does not meet the gate.** The project's metric is drift as a percentage of
distance travelled, benchmarked against OpenVINS on EuRoC at 0.72 %/10 m through
this same evaluation path. On that metric this system reads 15.4–16.0 % and the
< 5 % gate is not met. The measurements above explain the *character* of the
error; they do not turn a 16 % into a 5 %.

### Milestone 5: GPS denied mid-flight — done, after two fixes (2026-09-03)

**Result: 3 of 3 flights complete the GPS-denied mission, with net drift
0.38–1.73 % of the distance flown on vision alone. Milestone 5 passes.** Getting
there took two fixes and, more importantly, one corrected misdiagnosis: the
first six attempts were scored as estimator divergences and half of them were
not.

| configuration | held | net drift | peak excursion |
|---|---|---|---|
| default yaw, altitude from vision | **1 / 3** | — | — |
| fixed yaw, altitude from vision | **2 / 3** | — | — |
| fixed yaw, **altitude from baro** | **3 / 3** | 0.38 / 1.35 / 1.73 % | 1.04 / 5.33 / 7.94 % |

Final flights, denied while translating and flying the remaining square and both
diagonals on vision:

| flight | distance denied | mean err | max err | final err |
|---|---|---|---|---|
| m5g_a | 193.8 m | 3.36 m | 10.34 m | 3.35 m |
| m5g_b | 117.8 m | 0.57 m | 1.23 m | 0.44 m |
| m5g_c | 167.0 m | 5.28 m | 13.27 m | 2.25 m |

Net drift — where the aircraft actually ends up relative to where it thinks it
is — is 0.38–1.73 %, comfortably inside the < 5 % gate. **Peak excursion is not:**
10–13 m mid-flight on two of three flights, 5.3 % and 7.9 % of distance. That is
an operational limit worth stating plainly — this system should not be flown
GPS-denied within ~15 m of anything solid, even though it comes home accurately.

Note the denied distance varies 118–194 m against a nominal 118 m: a vehicle
correcting a drifting estimate flies further than the mission asks, which
inflates the denominator. m5g_b, the best of the three, is also the one that flew
the nominal distance.

#### The fix that mattered most: vision was flying the altitude controller

`EK3_SRC2_POSZ` was 6 (ExternalNav), so on switching source sets the altitude
controller was handed a monocular filter's z estimate. **It crashed the
aircraft.** Ground truth from the flight that found it — the vehicle reaches its
waypoint at 10.35 m, nearly stationary, the source set switches, and 0.4 s later:

```
t=79.3  z=10.07   t=80.5  z=6.92   t=81.7  z=4.15   t=83.3  roll +16.8  pitch -24.3
t=79.7  z= 9.25   t=80.9  z=5.91   t=82.1  z=3.34   t=83.8  roll +61.5  pitch -78.2
t=80.1  z= 8.09   t=81.3  z=5.00   t=82.5  z=2.52   t=84.2  z=0.21, motionless, pitch -68.5
```

Five seconds from the switch to lying in the dirt at a 68° angle. The estimator
then "diverged" to 11 km — which is what an MSCKF does when its camera is face
down on the ground.

**Two flights had already been recorded in this file as estimator failures on
exactly that evidence.** The divergence was a *consequence* of the crash, and
nobody had read the ground-truth attitude. `z` is the weakest axis a monocular
filter has, the measured vertical error was 1.77 m mean / 3.27 m max even on a
flight that survived, and handing that to an altitude controller at the instant
of a source switch is a step input. Baro is ArduPilot's own default for POSZ,
costs nothing, and is not what this milestone is testing. With it: vertical error
drops to 0.73 m mean and nothing crashes.

**The methodological lesson, which is the expensive one.** "The estimator
diverged" was true and useless. The estimator diverges whenever the vehicle is
broken, so it is the *last* thing to conclude from, not the first — and checking
took one query against data already on disk. Worse: the superseded runs were
deleted to free space before the crash mechanism was understood, so the
retrospective check on those two flights is no longer possible. **Do not delete a
run whose failure has been explained by inference rather than by measurement.**

The test is deliberately harder than the milestone's wording. "Holds position on
vision alone" invites a hover, and a hover is nearly free — drift is a fraction
of distance travelled and a hovering vehicle travels none. This flies one leg on
GPS, denies GPS **while translating**, and then flies the remaining square and
both diagonals on vision.

Denial is two steps and the order matters. Aux function 90 (`EKF_POS_SOURCE`)
switches EKF3 to source set 2, which milestone 4 configured as ExternalNav and
verified against GPS; only then does `SIM_GPS1_ENABLE=0` remove the receiver.
Doing it the other way leaves EKF3 with no horizontal position for a moment and
trips a failsafe, which tests nothing. Removing the GPS at all is what makes
this a real test: with the receiver still running, a source-set switch alone
leaves ArduPilot free to fall back, and a silent fallback would look exactly
like success — the same shape as the EKF type 10 incident.

`harness/gps_denied_eval.py` scores the **autopilot's** position, not OpenVINS'.
It recovers the transform between ArduPilot's NED frame and the simulator's from
the GPS-ON segment alone and measures everything after denial through it, so no
frame convention is assumed and every reported metre is post-denial.

| flight | distance flown denied | horizontal error mean / max / final | verdict |
|---|---|---|---|
| m5 | 155.3 m | 2.10 / 6.10 / **1.39 m** | **held** — 3.92 % max, 0.89 % final |
| m5b | 224.4 m | 19558 / 74343 / 74304 m | **flyaway** |
| m5c | — (never completed the box) | EKF reached n = −121.9 km, e = +140.0 km | **flyaway** |

**The first flight alone would have been reported as a pass.** 155 m flown on
vision with a final error of 1.39 m — 0.89 % of distance travelled — comfortably
inside the < 5 % gate and by far the best-looking number this project has
produced. It was not representative. This is the third time in this
investigation that a single run has told a confident lie, and the only reason it
did not become a milestone-complete entry is the standing rule about
distributions.

**What actually fails.** It is OpenVINS, not the bridge or EKF3. In m5b the
estimator's own `/ov_msckf/odomimu` reached x = 25.6 km, y = 78.1 km,
z = −16.1 km while ground truth had the vehicle at x = 217 m; EKF3 followed its
only position source, and ArduPilot flew the aircraft 217 m out of a 20 m box
chasing waypoints it could not reach. ArduPilot logged repeated
`EKF3 lane switch` / `primary changed` — both lanes unhappy, no healthy
alternative to switch to.

**A finding that survives the correction: open-loop stability does not imply
closed-loop stability.** It is weaker than it looked — most of the closed-loop
failures were crashes — but it is not empty: one diverged flight was replayed
offline from its own recording and diverged there too, ruling out live CPU
contention as the explanation. The remainder of this note is kept because the
distinction still holds, and because the CPU hypothesis is now measured rather
than assumed.

**The original, partly superseded reasoning:** The identical estimator configuration, replayed offline against
recorded flights, is stable every time — nine replays across three recordings,
spread 1.01–1.04×. Put it in the loop and it diverges two flights in three. Two
mechanisms are available and are not yet separated:

* **CPU contention.** Live, OpenVINS competes with Gazebo's software renderer,
  SITL, the bridge and the recorder; offline it has the machine. Dropped frames
  make KLT see large jumps and tracks die. `replay_openvins.sh` has warned about
  this in its docstring since it was written.
* **Positive feedback.** A bad estimate steers the vehicle, which worsens the
  geometry, which worsens the estimate. This mechanism *cannot* exist in replay,
  and it is the one milestone 5 is uniquely able to expose. In m5b the vehicle
  left the textured ground entirely (the tiles span ~90 m; it flew to 217 m), at
  which point recovery was impossible — but that is the end of the story, not
  the start.

Separating them needs the raw sensors from a diverged flight, which were not
recorded: `RECORD_SENSORS` defaults to 0 when the bridge is on, because
milestone 4's evidence was the dataflash log. That default is now wrong and has
been changed — **a GPS-denied flight is the one most worth being able to replay.**

**The vertical channel is a free improvement.** `EK3_SRC2_POSZ` is ExternalNav,
so altitude comes from vision. Baro is better and costs nothing; it was left on
vision only to match the configuration milestone 4 verified.

### Milestone 6, and the gate is met: stop yawing at the corners (2026-09-03)

**Drift 14.64 % → 1.91 %, ATE 1.97 m → 0.31 m, by changing one autopilot
parameter that has nothing to do with the estimator.** `harness/sweep.sh` flies a
set of configurations headlessly and tabulates ATE and RPE; the first batch, all
over one scene generated for the lowest altitude in the sweep so that altitude
varies alone:

| config | path | drift median (min–max) | ATE | coverage |
|---|---|---|---|---|
| `base_a10` — 10 m, default yaw | 156.2 m | 14.64 % (14.01–14.68) | 1.97 m | 96 % |
| `alt6` — 6 m, default yaw | 148.0 m | 32.17 % (32.10–40.56) | 7.48 m | 96 % |
| `fixedyaw` — 10 m, `WP_YAW_BEHAVIOR=0` | 155.3 m | **1.91 % (1.91–2.19)** | **0.31 m** | 96 % |
| `fixedyaw_b` — repeat flight | 155.4 m | **2.29 % (2.08–2.30)** | 0.42 m | 96 % |
| `fixedyaw_c` — repeat flight | 155.4 m | **2.72 % (2.55–2.89), n=2** | 0.39 m | 96 % |
| `flat_ctrl` — fixed yaw, **no obstacles at all** | 155.0 m | **1.85 % (1.85–1.88)** | 0.38 m | 96 % |

Each row is three replays of one recorded flight — **except `fixedyaw_c`, which
is two**: one replay hit `[init]: failed static init: platform moving too much`
and produced no trajectory at all. That is the initialiser's known
nondeterminism (it runs on a background thread) on data its two siblings handled
fine, not a property of the flight. The `n` column exists because the first
version of this table did not have one, and a two-replay row was written up as
three. The three `fixedyaw` rows are three separate FLIGHTS, so the headline
carries both kinds of variance — replay-to-replay and flight-to-flight. **Eight
estimator runs across three flights span 1.91–2.89 %.** Every row covers 96 % of its flight, and the coverage column is
there because a truncated estimate scores better: the comparison is only
meaningful because the spans match.

**The planar control settles the founding hypothesis for good.**
`flat_ctrl` is the same mission, the same camera, the same generated ground
tiles and the obstacle field **entirely removed** — a textured plane, the exact
configuration this project was designed to avoid. It scores 1.85 %, as good as
or better than the structured scene. Combined with the earlier direct
measurement (5.03 px of parallax at p95 in the structured world), the monocular
planar degeneracy is refuted twice over and by two independent methods. **The
3D obstacle field contributes nothing to accuracy here.** It cost RTF, scene
complexity and a collision constraint that shaped the whole altitude analysis,
and the flat world would have done as well.

**Against the reference points this project set itself:** OpenVINS on EuRoC
V1_01_easy gives 0.72–0.80 %/10 m and 0.067–0.115 m ATE over 58 m. This is
**2.29 % median over three flights (1.91–2.89 % across eight runs)** and
0.31–0.42 m ATE over 155 m. Same order of magnitude, on a longer flight, from a
simulated quadrotor. **The < 5 % gate for Phase 3 step 5 is met.**

**What was actually wrong.** ArduPilot ships `WP_YAW_BEHAVIOR=2`, "face the next
waypoint", so a position target with the yaw bits masked off still turns the
vehicle at every corner of the square — measured at 16–30° per corner, while the
vehicle is stopped. The camera points DOWN, so vehicle yaw is a rotation of the
whole image about its centre. OpenVINS' front end is pyramidal KLT, which models
translation only. The long-lived SLAM features are exactly the tracks that
cannot survive that: measured across the first corner, the SLAM feature count
collapses 49 → 3. The filter loses its long-term constraints at every corner and
never fully recovers.

**Altitude, now measured rather than argued.** 6 m is more than twice as bad as
10 m — 32.2 % against 14.6 %, ATE 7.48 m against 1.97 m — on the identical scene,
with altitude the only variable. That confirms the geometric prediction that
redirected the original "fly at 4 m" plan: over a field with collision geometry,
flying lower forces shorter obstacles under the vehicle and flattens the scene
rather than enriching it, while cutting feature dwell time.

**A reasoning error worth recording, because it nearly buried this.** The yaw
hypothesis was raised early and dismissed on the strength of `turn_diagnostic.py`,
which showed the estimate's yaw tracking ground truth to within 0.7° at every
corner. That measurement was correct and the inference from it was wrong.
Tracking a rotation accurately in the STATE says nothing about whether that
rotation destroyed the FEATURE TRACKS — the gyro integrates through a turn
perfectly well while KLT loses every long-lived correspondence. Two different
subsystems, one of which was measured and the other assumed. The lesson
generalises: a diagnostic that clears one mechanism does not clear a second
mechanism that happens to share a cause.

**Scope, honestly.** This is an operational fix, not an algorithmic one. Holding
heading is free here because the camera points down and the mission does not
care where the nose is. It is not free in general — survey patterns, gimbal
pointing and forward-facing sensors all want yaw — so the underlying fragility
remains: **this front end degrades badly under in-place rotation.** The
algorithmic fixes are a rotation-aware tracker, or stereo. Also untested is
whether fixed yaw alone would have been enough without the chi-squared fix; the
two were applied in sequence and only the combination has been measured.

**And the flat-runway control failed for an instructive reason.** The sweep's
`flat_runway` config never flew: the camera gate measured **83 % flat blocks** in
`iris_runway_vio.sdf` and refused. That world is flat *and* untextured — its
runway albedo map is stretched over 1500 m and minifies to grey at altitude — so
it confounds "no depth structure" with "no features", which are different
failures with different fixes. `gazebo/worlds/iris_flat_vio.sdf` is the correct
control: identical generated ground tiles, identical camera and mission, and the
obstacle field removed.

---

## Camera bringup

### The camera that reported success and returned zeros (2026-09-07)

First light on the OV9281. `rpicam-still` ran without error, printed `Still
capture image received`, and wrote a PNG in which **every pixel was exactly
zero** — 3,072,000 of them, one distinct value, in both a 640×400 and a
1280×800 capture.

Two diagnoses suggested themselves and both were wrong.

**Wrong read 1: underexposure.** It is the obvious one and it is refutable
without touching the camera. An underexposed global-shutter sensor still
delivers read noise, so a dark frame has a spread of small values. `distinct=1`
across three million pixels is not darkness; it is absence. *Measuring the
frame rather than looking at it settled this in one command* — the same lesson
as the black-image investigation of 2026-08-31, arrived at from the opposite
direction.

**Wrong read 2: the missing tuning file.** PROJECT.md had carried an open
question predicting exactly this failure, and the build plan told the future
reader to expect libcamera to *error* on a missing JSON. It does not error,
because nothing is missing: `ov9281_mono.json` ships with Pi OS 13 and the log
says it loaded. **A standing prediction that matches the symptom is not a
diagnosis**, and this one cost a detour.

**What actually located it was the AGC.** `rpicam-raw` logs its converged
exposure per frame:

```
#6 (30.00 fps) exp 14994.00 ag 1.44 dg 1.03      ... held for 35+ frames
```

15 ms at 1.44× analogue gain, stable. A sensor delivering zeros would drive the
AGC to its rails, hunting for light that is not there. Instead it settled at a
moderate value and stayed. **The ISP's statistics engine was seeing a correctly
exposed image at the same instant the output stream produced nothing** — which
places the fault after the statistics tap and before the output, and rules out
sensor, CSI link, driver and exposure in one observation.

The configuration line names it:

```
configuring streams: (0) 640x400-BGR888/sRGB (1) 640x400-MONO_PISP_COMP1/RAW
```

`ov9281_mono.json` is a *mono* tuning file with no colour pipeline — the
accompanying `Could not set SHARPNESS - no sharpen algorithm` warning is the
same file saying so. Ask that pipeline to emit BGR888/sRGB from a sensor with
no Bayer pattern and the conversion yields zeros. Stream 1, the RAW one, was
correct the whole time.

**The fix is not to fix it.** The processed path was never the right one for
this project and would have been wrong even had it worked: `sRGB` applies a
gamma transfer function, which is a nonlinear intensity transform sitting
directly in front of a corner detector, on a project whose camera section
([PROJECT.md:104](PROJECT.md:104)) already rejects processed output on exactly
those grounds. **All camera capture from here — focus, Kalibr, flight
recording — goes through `rpicam-raw`.** `harness/focus_check.py` does, and
documents why in its docstring.

**Layout detail that will bite anyone who skips it.** The R8 mode delivers a
**Y16 container with the 8-bit data in the high byte** — every value in the
file is an exact multiple of 256. mono8 is `raw >> 8`. Treat it as a 16-bit
image and you get a plausible-looking result that is wrong by a factor of 256;
truncate the wrong byte and you get zeros again, for an entirely different
reason. `focus_check.py` asserts the multiple-of-256 property rather than
assuming it.

**Confirmed in passing, and it matters for the bracket.** The mode table
answers the binning-versus-cropping question that the camera-IMU bracket's §7
geometry depends on:

| Output | Readout region | Reading |
|---|---|---|
| 640×400 | `(0,0)/1280x800` | full array, 2× downsample — **full FOV** |
| 1280×720 | `(0,0)/1280x720` | 80 rows unread, and off-centre — avoid |
| 1280×800 | `(0,0)/1280x800` | full array, 1:1 |

The field varies per mode, so it describes each mode's readout region rather
than the sensor size. **640×400 preserves the full lens field**, so the 30° tilt
and the `CAM_Y = +120` prop-clearance margin hold as designed. Note this
establishes that the *array* is fully read — the specific figure of 118° H is
still a datasheet claim, and one the manual already self-contradicted, until
Kalibr measures it.

**Focus baseline, for step 6.** As-shipped, indoors, at 15 ms / gain 1:
**focus score 5.5** (noise-suppressed), 16×16 block stddev median 1.71, no
clipping. The lens is far out of focus, which is expected and is what
[PROJECT.md:106](PROJECT.md:106) predicted once the real 2.8 mm focal length
moved hyperfocal to ~0.9 m. That number is the floor to beat; expect a large
multiple, not a few percent.

**The 100 m target requirement is over-specified.** Derived for this lens
(f = 2.8 mm, f/2.8, 3 µm pixels), the blur at infinity caused by focusing on a
nearer target:

| focus target | blur at infinity |
|---|---|
| 2 m | 0.47 px |
| 10 m | 0.09 px |
| 20 m | 0.05 px |
| 50 m | 0.02 px |
| 100 m | 0.01 px |

**Anything past ~20 m is optically indistinguishable from infinity here**, so a
50 m backyard is a fine venue and the build plan's 100 m is conservative by a
wide margin. What distance actually buys is *measurement sensitivity* — far
scenes put real texture at pixel scale, which is where defocus bites first —
not optical correctness.

Point it at the fence line or treetops rather than across the lawn, so the
frame is not half near-ground. (`focus_check.py --roi` can restrict scoring to
a window if that is ever awkward, but aiming solves it.)

**And the metric needed fixing before it could be trusted.** The naive
Laplacian variance the build plan calls for **cannot distinguish sharp detail
from sensor noise** — both are abrupt pixel-to-pixel variation. Measured on the
same static bench scene, same focus, only the exposure changed:

| exposure | raw Laplacian | noise-suppressed | raw/smoothed |
|---|---|---|---|
| 15 ms, gain 1 | 35.3 | **5.50** | 6.4 |
| 2 ms, gain 4 | 102.3 | **3.68** | 27.8 |

The raw metric says the short exposure is **3× sharper**. It is not; it is
noisier, and slightly worse. A focus sweep run on the raw number at high gain
would have chased the noise floor and locked the lens at the wrong place —
then been threadlocked there, permanently, before Kalibr.

`harness/focus_check.py` therefore scores a lightly blurred copy (3×3
binomial) and ranks on that: real edges survive the blur, single-pixel noise
does not. It reports the raw/smoothed ratio as a running check — high early in
a sweep is honest (a defocused frame genuinely has no detail) and should fall
as focus improves; still high at the peak means the gain is too high.
**Compare scores only at fixed exposure and gain.**

### Timestamps: the highest-risk item, measured and passed (2026-09-07)

Requirement 2 of the camera derivation ([PROJECT.md:80](PROJECT.md:80)) makes
timestamp *variance* the thing that matters — a constant camera-IMU offset is
estimated online as `calib_camimu_dt`, a varying one is unmodelable. PROJECT.md
called this the highest-risk item in the build and the reason the Pi was chosen
over faster boards. `harness/cam_timing.py` measures it. 300 frames, exposure
and gain pinned, `main` stream shrunk to 64×64 so the ISP is not in the way:

| | frame-to-frame stdev | spread |
|---|---|---|
| **`SensorTimestamp`** | **0.60 µs** | 4 µs |
| Arrival time in userspace | 49 µs | 1012 µs |

**Userspace arrival is 82× noisier than the hardware stamp**, and zero frames
were dropped over 300. That ratio is the CSI-over-UVC decision at
[PROJECT.md:86](PROJECT.md:86) measured rather than argued — a UVC camera
forces the 49 µs column. It also gets *worse* under load: at 60 fps arrival
jitter rose to 439 µs while `SensorTimestamp` jitter improved to 0.39 µs. The
two paths diverge in opposite directions exactly when it matters.

**Clock, confirmed by comparison rather than assumption.** `SensorTimestamp`
sits ~7 ms behind `CLOCK_MONOTONIC` and ~1.8 × 10¹² ms from `CLOCK_REALTIME`,
so it is on the monotonic timebase. **Linux IIO defaults to `CLOCK_REALTIME`,
which is NTP-disciplined and can step backwards mid-flight.** Before the IMU is
trusted, it has to be moved onto the same timebase:

```
cat  /sys/bus/iio/devices/iio:device0/current_timestamp_clock
echo monotonic | sudo tee /sys/bus/iio/devices/iio:device0/current_timestamp_clock
```

This is the single point where the two sensors are made to agree, and getting
it wrong produces drift that looks like an estimator problem.

**Frame rate is delivered 4.2 % long, at every rate.** Requested versus
achieved, measured three times:

| requested | achieved | ratio |
|---|---|---|
| 30.0 fps | 28.8 fps | 0.960 |
| 20.0 fps | 19.2 fps | 0.960 |
| 60.0 fps | 57.6 fps | 0.960 |

A constant multiplicative factor of exactly 0.96, not a quantisation to whole
readout lines (which would be a roughly fixed absolute offset, not a
proportional one). Mechanism unresolved and it does not block anything, because
**the estimator consumes timestamps, not a nominal rate.** It matters wherever
a nominal rate is *assumed*: CPU budgeting per frame, exposure limits derived
from a frame period, and any comparison against the sim's configured camera
rate. Assume 0.96 × requested until the cause is found.

**Method note.** Both wrong reads shared a shape: a plausible cause that
explained the symptom, adopted before any measurement discriminated between
causes. The thing that resolved it was a number nobody had asked for — the
AGC's converged exposure — which happened to be visible in a log already being
printed. Turning the log level up before forming a hypothesis would have been
faster than either guess.

### Kalibr on an arm64 VM: five failures, none of which announced itself (2026-09-10)

Focus is set and threadlocked, so step 5 — Kalibr intrinsics — is next.
`harness/kalibr_setup.sh` builds Kalibr in Docker on the sim VM: ROS 1 noetic
inside a container, because Ubuntu 24.04 has no ROS 1. Getting a working image
on arm64 took five fixes, and every one was a failure that either reported
success or said nothing at all:

| # | what it looked like | what it was |
|---|---|---|
| 1 | script died right after the sudo prompt, no message | `DF=$(ls a b c 2>/dev/null \| head -1)`: `ls` exits non-zero when any named file is missing (the repo has no plain `Dockerfile`), `pipefail` carries it out, `set -e` kills the script on the assignment. Same bug class as `build_speedybeef4v4.sh`'s feature grep. The sudo prompt was a bystander — once the docker group was active, sudo was never needed |
| 2 | a multi-GB pull of an image this VM cannot execute | upstream `FROM osrf/ros:noetic-desktop-full` is **amd64-only**, a single-arch manifest. Docker pulls it anyway with a warning, and even `--platform linux/arm64` does not fail fast. Rebased onto the official multi-arch `ros:noetic-perception`, which carries everything Kalibr imports from ROS (`roslib`, `rosbag`, `cv_bridge`) |
| 3 | verify passed while Kalibr could not import | upstream's **shell-form `ENTRYPOINT`** discards `docker run` arguments: told to `exit 7`, the image returned 0. And `\| head -2` on the real check reported head's exit status over a Python traceback |
| 4 | `import cv_bridge` → `SystemError: initialization of cv_bridge_boost raised unreported exception` | a known aarch64 bug in `ros-noetic-cv-bridge` 1.16.2, present in the **pure base image** too, same package versions — so neither Kalibr nor the rebase caused it. Imports cleanly if `cv2` is imported first; fixed with a one-line `.pth` preload |
| 5 | `rosrun: not found`, exit 127, with catkin's usage text executed as shell code | `source setup.bash` inherits the caller's positional arguments. The entrypoint sourced it while `$@` still held `rosrun kalibr ... --help`, so catkin's `_setup_util.py` got `--help`, printed its usage, and `setup.sh` eval'd that. Reproduced directly: with an argument present, sourcing prints `usage:: command not found`; with `set --` first, `rosrun` resolves |

And one that was the gate's fault, not Kalibr's: `kalibr_calibrate_cameras
--help` exits **2** by design — a bare `except:` around `parse_args()` catches
argparse's clean `SystemExit(0)`. Its exit status gates nothing.

The verify step now runs a **must-fail control first** (told to exit 7, must
return 7) so that a zero from the real check is interpretable at all; then
executes the tool's exact top-level imports, read out of the script inside the
image, as a plain `python3 -c` with an honest exit status; then checks `--help`
only for its usage banner and the absence of a traceback. Layered checks each
caught something the others could not: the control passed straight through
failure 5, because `bash` is on PATH with or without ROS.

**Target.** Aprilgrid 6×8, 45 mm tags, spacing 0.3, page 373.7 × 490.7 mm —
A2 with 23 / 52 mm margins. A first draft at 50 mm left **2 mm** side margins
on A2, which invites "fit to page", which silently rescales the print: a silent error in every metric distance Kalibr reports (not in focal length — target scale cannot reach the intrinsics). `calibration/aprilgrid_6x8_45mm.yaml`
holds `tagSize: MEASURE_ME` so Kalibr refuses to run until the print has been
measured — both axes, because printers scale feed and cross directions
differently and Kalibr assumes square tags.

**Validation data.** EuRoC's calibration sequences have left
`robotics.ethz.ch`, which now resolves but drops every packet, for the ETH
Research Collection (doi `10.3929/ethz-b-000690084`) as a single 4.2 GB ZIP.
Its bot protection returns 429 to scripted clients, so it comes through a
browser. The reference answer is already on the VM, inside `V1_01_easy.zip`
(`mav0/cam0/sensor.yaml`). Independently, this lens's physical specs predict the answer under an
equidistant model at **1280×800, full resolution — the mode that flies**:
**fx, fy ≈ 585–620 px, cx ≈ 640, cy ≈ 400** (620 from the 118° horizontal
field, 584 from the 148° diagonal; real lenses are not perfectly
equidistant, hence a band). A pinhole fit would land near 933, since
f = 2.8 mm over 3 µm pixels — which implies only 69° HFOV against the lens's
118°, and is itself the confirmation that this lens has to be modelled as
fisheye. (At 640×400 binned every pixel figure halves; that was the original
plan, superseded 2026-09-10.)

**The validator had a bug of its own, found by validating it.**
`harness/kalibr_compare.py` is the gate Kalibr has to pass on EuRoC before it
touches the bracket, so it was run first on five inputs with known answers.
Given a *perfect* camera-IMU result — Kalibr's `T_cam_imu` set to exactly
`inv(T_BS)` — it reported **178.3° and 99 mm of error**, and still printed
"proceed", because extrinsics never entered the verdict. It had been diffing
whichever two matrices it found across opposite conventions, covering the gap
with a note that "~180° means convention, not error". That heuristic held by
accident: a rotation compared with its own inverse differs by twice its angle,
so 178° is EuRoC's ~89° camera mount doubled, and a sensor mounted at 45° would
have read 90° and looked like a genuine error.

Both sides are now converted to `T_imu_cam` by key — valid because EuRoC's
body frame is its IMU frame, which was checked (imu0's own `T_BS` is the exact
identity) rather than assumed — the camera-to-camera `T_cn_cnm1` is no longer
matched against camera-IMU transforms, and extrinsics are gated at 1° and
10 mm. Re-run: the perfect result reads **0.000° / 0.00 mm**, agree; an injected
2° / 15 mm error comes back as **exactly 2.000° / 15.00 mm**, DISAGREE, exit 2.

**Kalibr validated on EuRoC `cam_april` (2026-09-10).** Same model as the
reference (pinhole-radtan), both cameras, the 72 s bag sampled at 4 Hz (~290
views per camera). Reprojection ±0.29 / 0.24 px on cam0, ±0.30 / 0.25 px on
cam1. Against EuRoC's published calibration:

| | Kalibr | EuRoC | difference |
|---|---|---|---|
| cam0 fu / fv | 458.958 / 457.615 | 458.654 / 457.296 | +0.07 % / +0.07 % |
| cam0 cu / cv | 366.559 / 248.234 | 367.215 / 248.375 | −0.09 % / −0.02 % of W |
| cam1 fu / fv | 457.590 / 456.167 | 457.587 / 456.134 | +0.00 % / +0.01 % |
| cam1 cu / cv | 379.222 / 255.177 | 379.999 / 255.238 | −0.10 % / −0.01 % of W |
| stereo baseline | 109.99 mm | 110.08 mm | −0.08 % |
| stereo rotation | | | 0.018° |

Every intrinsic sits an order of magnitude inside the 1 % gate, and the
focal-length differences (~0.3 px) are about one of Kalibr's own reported
standard deviations (±0.31 px) — agreement to the limit of what either
calibration can resolve. The comparison ran through the just-fixed
comparator, and the stereo reference was derived independently from the two
`sensor.yaml` files (`inv(T_BS_cam1) · T_BS_cam0`). Kalibr's configuration is
now proven on a known answer; what remains unknown in step 5 is the printed
target and our own lens. Camchain and Kalibr's summary are in `results/`.

**Capturing for Kalibr: the timestamps that looked absolute were relative
(2026-09-10).** Kalibr reads ROS 1 bags and the Pi runs no ROS, so
`harness/kalibr_capture.sh` records through `rpicam-raw` on the Pi and
`harness/kalibr_bag_from_raw.py` builds the bag inside the Kalibr image on the
VM. The mode is **1280×800, full resolution** — the flight mode, decided
2026-09-10; the build plan had said 640×400.

Reading rpicam-apps' source suggested `--save-pts` writes the absolute sensor
timestamp, offset only after a pause. It does not: the first frame after start
is itself a `FLAG_RESTART`, so the pts file begins at `0.000`. The capture
script checked the first timestamp against `CLOCK_MONOTONIC` rather than trust
that reading, and failed on its first hardware run, 141.8 s adrift. Step 5
would never have noticed; step 6's camera-IMU alignment would have been
silently wrong; and `kalibr_bagcreater` would have choked anyway, since it
reads all but the last nine digits of a filename as seconds and a stamp under
1 s has none.

Absolute time now comes from `--metadata` JSON, whose per-frame
`SensorTimestamp` is `CLOCK_MONOTONIC` in ns, and the metadata records are
proven to pair one-to-one with frames: `SensorTimestamp − first` matches every
pts value to **0.000 µs**. Both tools refuse a count mismatch or pairing worse
than 5 µs (the pts file's whole-microsecond rounding reaches 1.000 µs on real
captures; a misaligned record is off by a whole frame, ~208 ms); the converter's synthetic cases include one record shifted by 5 ms
and a capture with no metadata, and both are refused.

**A mean of 16 is black, not dim.** `SensorBlackLevels` is 4096 in the Y16
container — 16 in mono8 — so the first hardware capture's "mean 16.3" carried
essentially no signal; the metadata put the scene at 1–12 lux. Both tools now
report brightness above the black level. At 2 ms and gain 4, calibration needs
a properly lit room, a few hundred lux at least.

End to end on hardware: 24 frames at 4.80 fps, absolute timestamps, pairing
0.000 µs, bag stamps and pixels round-tripping exactly.

**Measuring the print: the divisors were wrong, and a correct print exposed
it (2026-09-10).** The Letter target (5×7, 25 mm tags) was measured at 170 mm
across and 235 mm down. Divided as first instructed — tags plus inner gaps,
÷6.2 and ÷8.8 — that gives 27.4 and 26.7 mm: a 2.7 % anisotropy that would
have meant reprinting, or, fed to Kalibr, every metric distance ~10 % too large (focal length is immune to target scale). But the
excess over the nominal grid was a constant 15 mm on both axes, which no
printer scaling produces; it is exactly two gaps. Rasterizing the PDF showed
why: Kalibr's grid draws a small black square, one gap wide, at every tag
corner **including the outer perimeter**, so the outer black edge sits one gap
beyond the outermost tags. The correct divisor is `N + 0.3·(N+1)`: 170 / 6.8 =
**25.0 mm** and 235 / 9.4 = **25.0 mm** — the print is exact and isotropic, to
the ~0.5 % a rule can read. Both target YAMLs now carry the corrected
divisors; the Letter one holds the measurement.

### Intrinsics: the lens is 69°, not 118° (2026-09-10)

Three 90 s captures of the Letter target at 1280×800, two lens models. The
centre focal length is the same in all of them:

| capture | model | centre focal length | reprojection |
|---|---|---|---|
| 1 | equidistant | 1111 px | ±0.26 / 0.36 px |
| 2 | equidistant | 1118 px | ±0.21 / 0.33 px |
| 3 | equidistant | 1115 / 1116 px | ±0.44 / 0.44 px |
| 3 | double-sphere | fx / (1 + ξ) = 793 / 0.709 ≈ 1118 px | ±0.41 / 0.42 px |

Field of view, unprojecting the image edges through each calibrated model:
equidistant **H 69.0° / V 41.8° / D 83.0°**, double-sphere 68.8° / 41.7° /
82.9°. Principal point ≈ (598, 419), about 42 px left of centre.

**The spec sheet's 148° D / 118° H does not describe this lens.** A
rectilinear 2.8 mm lens on this 3.84 mm-wide sensor gives
2·atan(1.92 / 2.8) = 68.9° — what was measured. The manual's 2.8 mm focal
length fits; its field-of-view figures do not. The "Corrected from the
InnoMaker manual" note in the camera derivation resolved the manual's own
contradiction the wrong way, and the predicted sanity band (fx 585–620)
inherited it. Kalibr was right from the first run.

Carried forward: the camera derivation's wide-FOV requirement (100–160°) is
not met by this lens; the bracket spec's §7 geometry assumed 118°; the sim's
80° camera is nearer the truth than the spec was.

How it was found, and one wrong turn. The first two runs came back at ~1115
px with unconstrained distortion terms, and the diagnosis "the target never
got beyond 35° from centre" was read off Kalibr's own polar-angle plot —
circular, because angles are computed through the solved fx, and a too-large
fx compresses every angle. Replaced by a coverage map in pixel space (OpenCV
AprilTag detection, 2-cell border to match Kalibr's grid, no lens model) and
by the two-model cross-check. Capture 3 was also too dark for Kalibr's
detector — about 9/255 above black, zero corners in every frame, although
OpenCV found tags in 101 of 144 — and `kalibr_bag_from_raw.py --boost`
applies a linear stretch after black-level subtraction, which moves no edge;
Kalibr then detected the target.

**Status.** Centre intrinsics are solid. The pixel coverage map shows the
right quarter of the image (x > 960 px) essentially unobserved and three
corners empty, so distortion there is extrapolation; the two models agree on
it (37.0° vs 36.9° at the right edge), which is weaker evidence than data. One
more capture reaching the right edge and the corners, in better light, would
close it. Calibration files: `results/kalibr_cam0_2026-09-10-{equi,ds}-*`.

### Camera-IMU extrinsics and time offset (2026-09-10)

Phase 4 step 6. `harness/kalibr_capture_imucam.sh` records both sensors on
`CLOCK_MONOTONIC` — the IMU through `imu_log.py` at 440 Hz, ±16 g / ±2000 dps,
the camera at 1 ms after an auto-exposure probe — and `kalibr_imu_csv.py` with
`kalibr_bag_from_raw.py --imu-csv` bags them for Kalibr. 60 s handheld in
front of the wall-mounted Letter target: 288 frames, 28,987 IMU samples, gyro
up to 248 dps, the IMU covering the camera from 2.0 s before to 4.1 s after.

| | default IMU model | + accel scale & misalignment |
|---|---|---|
| rotation, IMU → camera | identity + **2.21°** | identity + 2.37° |
| IMU origin in camera frame | **(0.6, -2.5, -30.5) mm** | (1.1, 0.1, -28.7) mm |
| time offset, t_imu = t_cam + | **2.18 ms** | 2.25 ms |
| reprojection, median | 0.40 px | 0.38 px |
| gyro / accel, normalised median | 0.57 / 1.60 | 0.55 / 1.58 |

The two models agree to 0.27°, 3.2 mm and 0.06 ms. Calibration files:
`results/kalibr_imucam_2026-09-10-*`.

**The rotation contradicts the bracket spec, and the spec is what is wrong.**
§4.4's axis convention predicts 180° about the camera's x axis; Kalibr finds
identity, 177.8° away, in both models. The gyro settles it: median residual
0.006 rad/s with the rig turning at up to 248 dps, which a rotation 180° off
could not produce — the gyro would disagree with the camera's rotation
throughout. The spec assumed the axes printed on the board are the axes the
driver reports, and they are not. The estimator consumes reported axes, so
Kalibr's answer is the one to use. The 2.2° residual, almost all about the
camera's x axis, sits at the edge of the spec's "a degree or two" and is
consistent with lens and board tilt; it is the extrinsic, not an error.

The translation puts the IMU about 30 mm behind the camera's optical centre,
on the back face, a few mm off axis. A ~2.2 ms offset is a plausible
exposure-plus-filter delay, and its smallness is the end-to-end confirmation
that the two sensors share a clock.

**Open, not blocking.** Accelerometer residuals run ~1.6× the noise model
even at ×5 inflation, and modelling scale and misalignment does not change
that, so the known 1.8 % sensitivity error is not the cause. More likely
handheld dynamics and the capture's clipping — a third of a typical frame at
255, the auto-exposure probe having metered darker than the scene was during
motion. A re-capture at lower exposure would tighten it.

### Step 7 prepared: OpenVINS live on the Pi, no ROS (2026-09-10)

`harness/vio_live/vio_live.cpp` is a native front end for OpenVINS built
ROS-free (upstream `6948812`, the parent of the VM's include-only local
commit). It reads the two IIO chardevs directly — accel interpolated onto gyro
stamps, the pairing used for Kalibr — takes the camera from `rpicam-raw`
through FIFOs, and feeds `VioManager` with the threading of OpenVINS's own ROS 2
node. `harness/vio_live.py` (sudo) does the exposure probe, sets up the IMU
through `imu_log.setup()` and starts both. No ROS and no Docker on the Pi.

| | measured |
|---|---|
| camera at 20 fps, 1280×800 | 19.20 fps delivered, 0 drops in 382 frames |
| `rpicam-raw --flush` into a FIFO | metadata arrives 10 ms after `SensorTimestamp`, one record per frame |
| SD sustained write | 84 MB/s, against 41 MB/s of Y16 recording |
| IMU delivery, watermark 8 | ~18 ms (the logger's 64 costs ~145 ms) |

The camera stays on `rpicam-raw`, per the rule above. Config is
`openvins/hw_pi/`: the Kalibr chain as-is, Allan ×5 noise, 440 Hz. The
estimator differs from the sim's in four ways: gravity 9.81, disparity and
feature spacing doubled for resolution, and `sigma_px`/χ² back to the defaults
1/1. The sim's 4/5 were tuned to its renderer. Confirmed loaded on the Pi at
DEBUG: equidistant model, noise, 2.2 ms offset.

Every live run records its sensors in the capture scripts' format.
`vio_bag_from_raw.py` makes that a ROS 2 bag for `replay_openvins.sh` on the
VM, so a live result can be reproduced and diffed offline. `vio_closure.py`
scores the loop.

**Built in `harness/buildenv`** by `vio_live/build.sh`, which bundles the 54
libraries the Pi lacks (360 MB, no apt on the Pi) and gates on `ldd` plus a
run on viopi. Two failures on the way, both silent at first:

- At `-j6` the OOM killer took a `cc1plus` inside Docker's 8 GB, and `tail`
  hid the `Killed` line. It is now `-j4`.
- The default `RUNPATH` covers only the binary's direct dependencies, so the
  bundled `libceres` could not find the bundled `libglog`. Linking with
  `--disable-new-dtags` (inherited `RPATH`) fixes it.

**Dry run** on the camera-IMU calibration capture: the whole chain runs, and the
estimator never initialises. That capture starts in motion, and static init
needs stillness followed by a jolt. The walk starts at rest.

**Bench run 1 (2026-09-10): two failures, both measured.**

- *The camera saw noise, not the room.* The shutter was capped at 1 ms, carried
  over from Kalibr, which forced maximum gain in a dim room. With the rig still,
  frame-to-frame noise was 9.4 DN against 3.0 DN of scene texture. FAST (on the
  equalised image OpenVINS tracks) found ~70k corners, and only 17 % survived
  one frame with nothing moving. With no usable tracks the filter ran on the IMU
  alone and went 18 m in 7 s. That happened on offline replay of the same data
  too, so live timing is not the cause. Online calibration stayed put
  (intrinsics ±0.3 px, time offset 2.2 → 1.35 ms), so it was not absorbing an
  error. The cap is now 4 ms (2–4 px blur at walking rates), with more light
  the better fix.
- *IMU samples stopped reaching `vio_live` 15 s in* (54.1 → 69.0 s) while the
  camera ran the full 60 s. The IMU interrupt kept firing, 27,093 times, about
  440/s for the whole run, so the driver did not die. `vio_live` exited cleanly,
  so no thread deadlocked. Not yet explained. `vio_live` now prints the
  interrupt rate, the kernel buffer fill, the IMU thread heartbeat and the worst
  feed time every 2 s, to show which side goes quiet.

**Bench run 2 and the IRQ check (2026-09-10): the sensor itself had gone bad.**
Bench 2 got one interrupt and no samples. `imu_irq_check.sh` then got 0
samples and 0 interrupts at watermark 64 as well as 8, so the watermark is
ruled out. One-shot sysfs reads showed why: ~20 g at rest, gyro up to
37 rad/s, and every 16-bit word with equal high and low bytes (`0x6F6F`,
`0xAEAE`, `0x6161`…). That is the sensor returning one register byte twice,
a broken SPI link or scrambled register state, which also explains INT1 never
firing. It broke while the rig was being handled in bench 1. The IMU is on
dupont clips over a link that only works at exactly 10 MHz, so flexing wires
are the leading suspect. Driver code is ruled out: the FIFO read drains until
empty, and nothing logged an error. `vio_live.py` and `imu_irq_check.sh` now
check one-shot reads first and refuse on this pattern. A Pi reboot does not
cut 3.3 V, so recovery needs a full power cycle.

---

## IMU bringup

### The bus works at 10 MHz and nowhere else (2026-09-09)

The ISM330DHCX went onto SPI0 CE0 with INT1 on GPIO25. First contact over raw
`spidev` returned `0x00` for WHO_AM_I at 1 MHz, in both SPI modes, on both chip
selects. Read as a wiring fault it points at MISO; it was not a wiring fault.

**What the bus actually does**, WHO_AM_I over `/dev/spidev0.0`, 20 reads each:

| clock | correct |
|---|---|
| 100 kHz – 9 MHz | **0/20 at every speed** |
| 10 MHz | **20/20** |
| 12 MHz | 10/20 |
| 16 MHz, 20 MHz | 0/20 (past the part's rated 10 MHz) |

A single working point, at the part's rated maximum, with total failure one
megahertz below it. Three things rule out the obvious readings:

- **It is not signal integrity.** That degrades with speed; this is the
  opposite, and 10 MHz is reproducible 20/20 across six rounds spread over
  minutes.
- **It is not the wiring, and not the part.** Bit-banging the identical pins
  through `gpiod` at roughly 25 kHz returns `0x6B` every time. The part answers
  fine when something else generates the clock.
- **It is not the read path.** Writing `CTRL1_XL` at 100 kHz and reading it back
  at 10 MHz shows the write never landed. The whole transaction fails, and an
  all-zero response means SDO was never driven at all — the part never leaves
  I2C mode, which is what a slave does when it does not see a clean frame.

CS was confirmed to toggle, `cs-gpios` is present on the `snps,dw-apb-ssi`
controller, and the measured bit rate at 100 kHz is 99.9 kbit/s, so the clock
frequency itself is correct. **Root cause unresolved** — the remaining
candidate is the RP1 controller's framing at CS assertion, whose duration
scales with the clock period and so would only be short enough to ignore at the
top of the range. `SPI_NO_CS` returns `EINVAL` on this controller, which closes
the cheapest way to isolate it.

**It does not block anything**, because the one working point is the one the
driver wants: `spi-max-frequency = <10000000>` in
`harness/ism330dhcx-spi0.dts`. Mode 0, not the datasheet's mode 3 — 8/8 against
7/8 measured at 10 MHz.

**What it costs is margin.** The part runs at its rated maximum with none, on a
vehicle, for the life of the project. If the IMU ever stops enumerating, the
clock is the first suspect and the wiring is the last;
`harness/imu_probe.py` sweeps clock and mode and prints the matrix rather than
assuming a speed. That sweep exists because the first version of the probe
assumed 1 MHz and confidently reported a healthy part as dead.

### The kernel driver enumerates, streams, and drifts 1.2 s per 10 minutes (2026-09-09)

`st_lsm6dsx` built out of tree (`harness/build-st-lsm6dsx.sh`), under DKMS for
all four installed kernels, INT1 raising real interrupts on GPIO25. The build
plan's step 4 gate — log 10k samples, histogram the deltas, ragged means
software timestamping — **passes perfectly and means almost nothing**:

```
gyro: n=130047  mean=2270.40 us  stdev=0.00 us  min=2270.4  max=2270.4
```

Zero jitter to 0.1 µs across 130k samples, including across FIFO batch
boundaries. Real measurements are never that clean. The gate catches ragged
deltas; it cannot catch smooth ones that are fabricated.

**Regressing sample timestamps against `CLOCK_MONOTONIC` is the test that
matters**, and it fails badly:

| | slope | error | per 10 min flight |
|---|---|---|---|
| gyro | 0.9979975 | **−2003 ppm** | **−1202 ms** |
| accel | 0.9979987 | −2001 ppm | −1201 ms |

Two sensors agreeing to 2 ppm, so one shared clock, not noise.

**Mechanism, from the driver source.** `st_lsm6dsx_buffer.c:327` sets
`sensor->ts_ref = iio_get_time_ns()` **once**, when the buffer is enabled. Every
sample thereafter is `ts_ref + sensor_ticks × ts_gain`, where `ts_gain` is
25000 ns nominal trimmed by the chip's `FREQ_FINE` register
(`ts_gain -= (s8)FREQ_FINE × 37500 / 1000`). It is open loop: no re-anchoring,
no feedback. A wrong gain integrates for the whole session. Ours is wrong by
25000 × 0.002003 ≈ **50 ns per 25 µs tick** — residual oscillator error the
factory trim did not remove.

This is precisely the unmodelable error the camera work at
[PROJECT.md:84](PROJECT.md:84) exists to prevent. `calib_camimu_dt` estimates a
*constant* camera-IMU offset; this one ramps at 2 ms per second.

**Two measurements were reinterpreted by this.** "Delivery latency" of 46 ms at
a 23 s capture and 319 ms at 295 s was never FIFO buffering — it scaled with
capture length because it *was* the accumulating skew. And the userspace
spidev logger, which looked like the crude option, is the one whose timestamps
track real time, because it interpolates against the host clock every 250 ms
instead of extrapolating from the sensor for the whole run.

**Fixed** by `harness/st_lsm6dsx-reanchor.patch`: one single-pole correction on
`ts_ref` per FIFO batch, clamped so a transient cannot reorder samples across a
boundary. Deliberately not a one-step correction — the host reading comes from
a threaded irq handler and carries scheduling jitter, and slewing by that
jitter would trade a slow ramp for fast noise, which is the worse of the two
because no estimator state absorbs it.

| | stock | patched |
|---|---|---|
| clock skew | **−2003 ppm** | **−0 ppm** |
| drift per 10 min flight | 1202 ms | 0 ms |
| implied sample rate | 440.451 Hz | **439.5694 Hz** |
| delta stdev | 0.00 µs (synthetic) | ~~0.062 µs~~ **22.8 µs** at watermark 64, idle (2026-09-10, below) |
| non-advancing / backwards | 2 / 0 | 0 / 0 |

The patched rate agrees with the independent three-hour host-clock measurement
(439.59 Hz) to 0.005 %. ~~Jitter of 0.062 µs is 10× tighter than the camera's
0.60 µs `SensorTimestamp`, so the IMU is no longer the weak side of the time
budget.~~ Wrong at the default watermark; see the correction below.

**Correction (2026-09-10): the patch leaves a sawtooth, not 0.062 µs of jitter.**
The patch corrects the offset (`ts_ref`) once per FIFO batch but leaves the tick
length (`ts_gain`) 0.2 % short. Within a batch, timestamps are spaced by the
uncorrected 2270.4 µs; at each batch boundary the correction jumps them forward
to catch up. Measured on BEC power, same logger (`imu_log.py`, watermark 64):

| | idle, 2 min | 4-core load + camera, 10 min |
|---|---|---|
| samples per batch | 21, every batch | 1 |
| spacing within a batch | 2270.4 µs, identical across 50k samples | — |
| jump at batch boundary | ~96 µs | ~4 µs |
| delta stdev | **22.8 µs** | 2.26 µs |
| duplicate timestamps | 1 (first batch boundary) | 0 |
| rate / gaps | 439.58 Hz / 0 | 439.58 Hz / 0 |

Skew stays 0 ppm and the rate is unchanged, so the drift fix stands. Why the
batch size differed between the two runs with identical settings is unexplained.
The ~96 µs worst case is 0.5 mm at 5 m/s and averages to a constant offset that
`calib_camimu_dt` absorbs, so it is left as is. The IMU is **not** tighter than
the camera's 0.60 µs; the time budget is still far inside what matters. To remove
the sawtooth, correct `ts_gain` as well as `ts_ref`. Noise densities are unchanged — accel 4.707/4.632/5.256e-04, gyro
1.029/0.896/0.726e-04, within 2 % of the three-hour run on every axis — which is
what should happen, since Allan variance depends on the mean sample interval
and nothing else about absolute time.

The alternative considered and rejected was an affine correction in userspace:
no kernel divergence, but every consumer has to apply it or silently get wrong
time, and the residual is temperature dependent so a one-off calibration would
degrade as the IMU warms in flight. The patch is now ours to carry across
kernel bumps; DKMS rebuilds it, and upstream is pinned by SHA so the patch
applying cleanly is the thing that could bite.

**A caution on the result.** `slope = 1.0000000` is what a closed loop
*produces*, not independent confirmation that it is right — the loop forces
that number. The evidence the fix is real is elsewhere: the implied rate now
matches an independently measured one, jitter went from impossibly clean to
plausibly small, and monotonicity held.

**Also logged.** The accelerometer emits occasional duplicate timestamps — 2 in
130,047 — giving a zero `dt` that would divide-by-zero in a naive
preintegrator. `allan.py` splits the record there by itself (99.99 % retained).
The gyro shows none.

**The Allan numbers survive all of this**, and are now confirmed twice over.
Noise density through the kernel driver over 5 minutes against the 3-hour
spidev run: accel 4.656/4.574/5.432e-04 vs 4.697/4.591/5.368e-04, gyro
1.0404/0.8706/0.7261e-04 vs 1.0410/0.9050/0.7382e-04 — every axis within 4 %,
most within 1 %, across two completely independent read paths. Allan variance
needs the mean sample interval and nothing else about absolute time, so a
ramping clock does not touch it.

### The INT1 wire that was connected, verified, and still did nothing (2026-09-09)

The first overlay carried a pinctrl fragment muxing GPIO25 as an input. It was
deleted before installation, with the reasoning that requesting an interrupt
configures the pad anyway and fewer fragments meant fewer failure modes. True
on BCM283x. **False on RP1**, where Pi 5 GPIO moved to the southbridge — the
same architectural shift already recorded at [PROJECT.md:185](PROJECT.md:185)
for the shutdown button.

The result had no error anywhere in it. The driver bound, enumerated both IIO
devices, accepted an ODR, accepted a watermark of 64, accepted a buffer enable,
set the timestamp clock, and answered raw register reads with correct gravity.
Every plausible check passed. The only symptom was `/proc/interrupts` showing
`0` and the buffer returning nothing:

```
185:  0  0  0  0  pinctrl-rp1  25  Edge  lsm6dsx      # pad funcsel was "none"
```

One `pinctrl set 25 ip pd` took the count from 0 to 28 in two seconds.

**A zero counter is evidence, not the absence of it.** That line was in the
first diagnostic and it was the whole answer; 60 KB of driver source got read
before the pad itself was looked at.

**Second-order lesson: the monotonic clock did not survive the reboot.** It was
set at runtime by the install script, and IIO defaults every device back to
`CLOCK_REALTIME` on the next boot. Now pinned by a udev rule
(`/etc/udev/rules.d/99-ism330dhcx.rules`). A setting that has to be reapplied
by hand is not a setting, it is a landmine.

### Allan variance: the numbers, and what a single footstep cost (2026-09-09)

Three hours stationary at 416 Hz nominal (439.59 Hz delivered), ±4 g and
±500 dps, **4,747,614 samples per sensor, zero FIFO overruns, no timestamp
gaps**. Captured with `harness/imu_log_spidev.py`, reduced by
`harness/allan.py`. Raw record in `datasets/imu_allan_2026-09-09/`, curve in
`results/imu_allan_2026-09-09.png`, table in the matching `.tsv`.

**Trimmed to the first 170 minutes**, for the reason below.

| | N (noise density) | B (bias instability) | τ_B | K (random walk) |
|---|---|---|---|---|
| accel x | 4.697e-04 | 3.986e-04 | 24.8 s | 2.523e-05 |
| accel y | 4.591e-04 | 3.919e-04 | 220.6 s | 2.606e-05 |
| accel z | 5.368e-04 | 3.966e-04 | 19.9 s | 4.683e-05 |
| gyro x | 1.041e-04 | 1.768e-05 | 197.8 s | 1.229e-06 |
| gyro y | 9.050e-05 | 2.079e-05 | 114.5 s | 1.749e-06 |
| gyro z | 7.382e-05 | 1.660e-05 | 341.7 s | 7.173e-07 |

Accel in m/s², gyro in rad/s. For `kalibr_imu_chain.yaml`, worst axis of three:

```yaml
accelerometer_noise_density: 5.367825e-04
accelerometer_random_walk:   4.682763e-05
gyroscope_noise_density:     1.041003e-04
gyroscope_random_walk:       1.749075e-06
```

**Against the datasheet**, which is the only independent check available: accel
noise density spec is 70 µg/√Hz = 6.87e-04, measured 4.6–5.4e-04, so **better
than spec**; gyro spec is 5 mdps/√Hz = 8.73e-05, measured 7.4–10.4e-05, **on
spec**. Short-τ slopes −0.41 to −0.46 against an ideal −0.5, and resolution
7.3 LSB (accel) and 4.1 LSB (gyro), both clear of the 2 LSB floor below which
the read is not usable — the narrow ranges were the right call.

Inflate 5–10× before flight as usual: a bench run contains no vibration and no
thermal transient.

**One person walking past, once, for thirty seconds, wrecked the accelerometer
numbers.** A disturbance at t = 171.0 min showed up as a single 30 s block at
**11.2× the median stdev** on both sensors, everything else flat across 360
blocks. It is not a timestamp gap, so nothing in the capture path notices it,
and the run reports 100 % contiguous and zero overruns either way:

| accel x | full 180 min | trimmed to 170 |
|---|---|---|
| short-τ slope | **−0.22** | −0.46 |
| τ_B | 3.2 s | 24.8 s |
| K | **2.41e-04** | 2.52e-05 |
| N | 5.09e-04 | 4.70e-04 |

**Random walk off by 10×**, the slope broken badly enough to be obvious, and
the noise density inflated 8 % — from 30 seconds out of three hours, 0.3 % of
the record. The gyro barely moved, which fits: footfall is translation, not
rotation.

Two things follow. Trimming is now a flag (`allan.py --start/--end`) that
prints what it dropped, because a silent trim is a claim about data nobody can
audit. And the accelerometer is far more sensitive to bench disturbance than
intuition suggests — for any repeat, the profile check in
`harness/` that found this (per-30 s block stdev against the median) should run
before the numbers are believed, not after someone remembers walking past.

### Pi OS does not build the driver this part needs (2026-09-09)

The overlay loads correctly — `/dev/spidev0.0` disappears as intended, and
`spi0.0` appears with `compatible = "st,ism330dhcx"` and modalias
`spi:ism330dhcx`. Nothing binds to it, because **the driver does not exist on
this system**:

```
$ grep LSM6DSX /boot/config-6.18.39+rpt-rpi-2712
# CONFIG_IIO_ST_LSM6DSX is not set
$ find /lib/modules/$(uname -r) -name '*lsm6*'      # nothing
```

Pi OS ships 55 IIO modules and exactly two IMU drivers, `inv-mpu6050` and
`bno055`. The claim at [PROJECT.md:119](PROJECT.md:119) that the ISM330DHCX has
a "mainline `st_lsm6dsx` IIO driver" is true of upstream and false of the
kernel actually running, and nothing in the part-selection reasoning checked the
distribution rather than upstream. Kernel headers **are** installed for the
running kernel, so an out-of-tree build is the fix, and it is not done yet.

**It does not block the Allan run, and that is a fact about the part rather
than a workaround.** Allan variance needs a long stationary series that is
uniformly sampled; the ISM330DHCX samples and buffers on its own schedule in
hardware, which is [the property it was chosen for](PROJECT.md:121). Draining
that FIFO from userspace yields the same samples the kernel driver would have
handed over. The driver matters when IMU samples have to share a clock with
camera frames — Phase 6, not this.

`harness/imu_log_spidev.py` does that, and needs no root. Two consequences of
the overlay had to be worked around: `spidev0.0` is gone, so transfers go out on
`/dev/spidev0.1` whose CE1 (GPIO7, pin 26) toggles into thin air with nothing
wired to it; and GPIO8 is still held by the SPI core for the unbound `spi0.0`,
so `gpiod` cannot claim it and `pinctrl` drives the pad directly. Gated the
usual way: **10/10 correct WHO_AM_I with the CS assert, 0/5 without it**, so it
is the assert doing the work.

### The IMU delivers 440 Hz when asked for 416 (2026-09-09)

Measured over two minutes against the host monotonic clock, twice: **439.84 and
440.09 Hz for a requested 416 Hz, +5.8 %**, with zero FIFO overruns. Plain
internal-oscillator tolerance, uncalibrated.

The same shape as the camera's [4.2 % long frame rate](PROJECT.md:1493), and
the same lesson: **the estimator consumes timestamps, not a nominal rate**, and
anything that assumes the nominal rate is wrong by that much. The first version
of the logger stamped samples at `i / 416`, which would have stretched every tau
by 5.8 % and inflated the noise density read off it by about 3 % — a systematic
error, invisible in the output, in the number the whole run exists to produce.
It now interpolates against the host clock between FIFO drain boundaries, so
the true rate falls out of the data instead of being asserted.

Worth carrying into Phase 6: whatever finally publishes IMU samples must not
assume 416 Hz either.

### The `pkill` trap, paid for twice (2026-09-09)

[PROJECT.md:472](PROJECT.md:472) already records that `pkill -f` matches the
shell that issued it when the pattern is in that shell's own command line. Used
it anyway inside an `ssh` command line to clear a stale capture; it killed the
SSH session before the launch line ran, and the three-hour run silently did not
start. The fix was already written down: **use it from inside a script file**,
which is what `harness/start-allan.sh` now does. Reading the repo before acting
would have been faster than rediscovering it.

### Verified good, on raw spidev, before any overlay

Deliberately before installing the device tree overlay: a driver that fails to
probe reports "failed" and little else, and a dead MISO, a swapped clock and
data pair, an unpowered part and a wrong SPI mode all look identical from
there.

| check | result |
|---|---|
| WHO_AM_I | **0x6B**, 8/8 at 10 MHz mode 0 |
| Gravity, stationary | **9.6302 m/s²** against 9.80665 — 1.8 % low, ordinary uncalibrated sensitivity tolerance, and what Kalibr is for |
| Gyro at rest | **0.79 dps** total, within the part's zero-rate offset spec |
| Temperature | 18.3 °C, plausible and confirms the channel |
| INT1 on GPIO25 | asserts on data-ready, clears on read, 5/5 |
| Burst reads | 1024 B in 0.92 ms at 10 MHz, ≈8.9 Mbit/s |
| Tagged FIFO | 264 samples buffered, both sensor tags present |

**A pin-level trick worth keeping.** Before the part would talk, the useful
measurement was a pull-up/pull-down sweep of every header GPIO: a pin with
nothing on it follows the pull, a pin with something attached does not. That
map showed MOSI and SCLK held high by the breakout's I2C pull-ups (which only
exist if the board is powered), MISO held low by its address-select pull-down,
INT1 driven low by the part, and CS held high by the part's internal pull-up —
against every other header pin floating cleanly as a control. It proved all
seven wires and the power rail without a multimeter, and narrowed the fault to
the controller before a single clip was touched.

---

## Build environment

### The Mac as an arm64 build host, gated on running the binary (2026-09-09)

Phase 2 step 6. `harness/buildenv/` holds a `debian:trixie` container matching
viopi's ABI. On Apple Silicon this is **not cross-compilation** — Mac and Pi are
both arm64, so `--platform linux/arm64` runs natively. It is a faster arm64
machine on the desk, not an emulator, and `buildenv.sh` asserts that rather than
assuming it (an x86 host would silently fall back to QEMU and lose ~10×).

**The gate is not "the image built".** A `bookworm` container builds just as
happily and produces binaries that fail only once they reach the Pi — possibly
weeks later, inside something the size of OpenVINS. So `verify` compiles
`abi_probe.cpp` in the container, runs it there, copies **that same binary** to
the Pi, runs it there, and passes only on byte-identical output. It reads the
Pi's glibc live over ssh rather than trusting a constant in the script.

`abi_probe.cpp` is deliberately not a hello world: it exercises `std::string`
and `std::vector` layouts, `shared_ptr`, and exception unwinding, and prints
`glibc`, `__GLIBCXX__` and `_GLIBCXX_USE_CXX11_ABI`. A pure-libc program links
against almost anything and proves nothing.

| | container | viopi |
|---|---|---|
| arch | aarch64 | aarch64 |
| glibc | 2.41 | 2.41 |
| gcc | 14.2.0 | 14.2.0 |
| `__GLIBCXX__` | 20250315 | 20250315 |
| CXX11 ABI | 1 | 1 |

**Second half of the check, and the one that is easy to forget:** an ABI-clean
binary still will not start if the shared objects are absent or skewed. Both
sides run the same Debian release, so `apt` on the Pi yields identical versions
— but that is a claim worth checking. `verify` compares them: OpenCV
`4.10.0+dfsg-5` is already installed on the Pi and identical, and Ceres
`2.2.0+dfsg-4.1+b2`, glog `0.6.0-2.1+b2`, gflags `2.2.2-2+b1` are all available
at matching versions.

**The split, unchanged from the plan:** heavy third-party C++ in the container;
anything touching libcamera, kernel headers, or IIO built on the Pi itself,
because those bind to the running kernel and the Pi's own libraries. Re-run
`verify` after any kernel or base-image change — it catches skew before it
reaches something big enough to be expensive.

---

## Rejected options

| Rejected | Why |
|---|---|
| 10-inch 6S airframe (original BOM) | Wrong platform for GPS-denied dev; ~10 kg thrust is a serious injury risk in the indoor/low-altitude environments this work implies |
| Pixhawk 6C Mini | Only 2 general-purpose UARTs — fully allocated with zero spare |
| PM08-CAN 200 A power module | 14S/200A on a 4S quad; ESC already has current sensing |
| Arducam IMX219 (original BOM) | Rolling shutter — disqualifying |
| Arducam OV9281 USB UB0232 (B08Q2WXQL6) | 850 nm IR-pass filter; only works with IR illumination |
| Arducam OV9281 eval kit (B08HYPX1VV) | Bundles USB3 shield and MIPI adapter you don't need |
| Official RPi Global Shutter Camera | Color IMX296 (Bayer loss), C/CS mount needs a separate ~$25 lens, 50–60 g assembly, ~60° HFOV |
| Arducam IMX296 Mono M12 (B0GS59PT9W) | Viable at $65 but 96° FOV and no trigger pin; InnoMaker is cheaper, wider, and has opto-isolated trigger |
| BNO085 | Sensor fusion chip — see derivation above |
| ~~Matek M10Q-5883~~ | **Un-rejected.** QMC5883L is enabled in stock firmware; only the QMC5883P successor is out, and that is fixable in the custom build |
| CanaKit Pi 4 Starter Kit (B07V5JTMV9) | Wrong generation (~half the CPU throughput), single 15-pin CSI kills the stereo upgrade path, kit contents are wall PSU / case / HDMI you won't use |
| Optical flow (3901-L0X) | ~~Four features to re-enable, competing with external nav for flash~~ — **the flash premise is refuted (2026-09-03): all four fit alongside external nav with 71 KB spare.** The decision stands on the other stated ground only: it is low-altitude-only and contributes nothing at cruise |

---

## Where priors failed

Recorded because the failure mode is systematic, not incidental.

- **`SpeedyBeeF405v4`** — the ArduPilot board is `speedybeef4v4`, lowercase, "F4v4". Assumed CamelCase consistency with the other SpeedyBee targets; it isn't there.
- **Arducam URL pattern-matching** — constructed `mini-ov9281-mono-global-shutter-wide-angle-for-pi.html` from the non-wide-angle page. The URL happened to be right, but `arducam.com/*.html` pages are **catalog pages with no cart**. Arducam retail is UCTRONICS, Amazon, or `arducam.com/product/*`. This wasted two rounds.
- **"Skip the 27 W power supply"** — reasoned from the flight configuration (BEC-powered) and ignored that weeks of bench work come first, where a wall supply is required.
- **Amazon search results** — a "raspberry pi 5" query surfaced a Pi 4 kit as result #2, because accumulated reviews outrank relevance. B07-prefix ASINs are 2019; anything genuinely Pi 5 is B0C or later.
- **BEC "5 A" listings** — three consecutive Amazon results matching "5 V / 5 A BEC" were 1.5–3 A continuous. FPV BEC marketing quotes peak universally.
- **"Start slow on a new SPI bus"** — the standard conservative move, and exactly backwards on RP1: 100 kHz through 9 MHz fail completely while 10 MHz is perfect. The first IMU probe hardcoded 1 MHz and reported a healthy, correctly wired part as dead. Sweep the parameter you are about to assume, especially when the assumption is the cautious one.
- **BNO085 recommendation reversal** — recommended over BMI270 in the OAK-D context (comparing two options you don't control, where DepthAI exposes raw output), then rejected for standalone use. Both are correct in context; the earlier statement was scoped and read as general.

The pattern: pattern-matching from adjacent cases produces plausible answers that fail on specifics. Verify SKUs, URLs, and continuous ratings rather than inferring them.

---

## Open questions

- **Bracket tilt against the measured lens.** The bracket was built for a 118° lens; the lens measures 69° H / 42° V, so the 30° nose-down tilt leaves the horizon out of frame (top edge ~8° below horizontal) and puts nearly all features on the ground plane. A ~15° reprint would restore it. Deferred, not urgent (2026-09-10); a reprint means redoing step 6. See `camera-imu-bracket-spec.md` §7.
- Airframe condition unknown — cracked arms, damaged ESC FETs, and bent motor shafts are all live possibilities. Triage gates everything.
- Battery capacity, connector, and health unverified (cell count confirmed 4S).
- ~~Whether visual odom + external nav actually fit in 1 MB alongside the rangefinder~~ — **answered 2026-09-03 by building it: yes, with 98 KB to spare.** See below. The binding constraint was never flash; it is that both features are compiled out by *source default* on a 1024 KB board.
- 3D printing is now optional, not blocking — only the camera+IMU bracket needs rigidity and it is hand-cuttable from FR4. Printer purchase deferred until mount iteration actually bottlenecks.
- Camera works on **Cam0 port only** on Pi 5 per user reports. ~~A missing libcamera tuning JSON in default Raspbian requires vendor support to resolve~~ — **answered 2026-09-07: it is not missing.** `/usr/share/libcamera/ipa/rpi/pisp/ov9281_mono.json` ships with Pi OS 13 and loads without complaint. The real trap was elsewhere and is silent rather than loud — see *The camera that reported success and returned zeros*.
- **Why frame duration is delivered 4.2 % long** — exactly 0.96 × requested at 20, 30 and 60 fps. Proportional, so not line quantisation. Harmless (timestamps are what the estimator reads) but unexplained.
- Whether the OV9281's 640×400 mode bins or skips. FOV is preserved either way (confirmed binned-not-cropped), but skipping aliases, which would cost feature repeatability frame to frame. Test: compare `count_features.py` on a native 640×400 capture against a software-downscaled 1280×800 one.
- Whether gz's `dynamic_bias_stddev` is also per-sample. The white-noise `<stddev>` question is answered (it is); the bias terms were not measured.
- **What takes the sim from 16 % drift to the < 5 % gate.** ATE is already 1.6 % of path length; the residual is local (RPE), i.e. tracking noise, and the parameter absorbing it (`up_*_sigma_px` = 4) is a measurement of that noise. The untested lever is what causes it: ~15 px of inter-frame image motion at 5.8 m/s and 15.2 Hz effective tracking. Flying slower is the direct test.
- **Whether the chi-squared gate is the same story on hardware.** In sim the fix was worth 6× in drift and 95× in ATE. Real imagery has motion blur, vibration and genuine outliers, so the right `sigma_px` will differ — but the diagnostic transfers exactly: count the features the filter *uses* per update, and if that goes to zero while the images are still trackable, the gate is the problem.
- ~~Why online calibration helps by 15×~~ — **closed.** It does not; re-measured at 97.44 % against a 97.83–99.39 % baseline. The original measurement was of a pre-gravity-fix configuration on a different scene.

---

## Future directions on this platform

Available now:
- Terrain-relative navigation on a rotorcraft (port of existing fixed-wing work; hover and repeatability make it a better characterization platform)
- TRN + VIO fusion — VIO propagates smoothly and drifts, TRN provides absolute fixes. This is the architecture of fielded GPS-denied navigation.
- Public GPS-denied dataset with RTK ground truth (outdoor quad-mounted monocular GS is thin in the literature)
- EKF source health monitoring and automated graceful degradation
- Precision landing on fiducials
- Custom outer loop via Guided setpoints — measure MAVLink round-trip latency first, it caps controller bandwidth

Cheap unlocks:
- **Second CSI camera ($36)** — the Pi 5 has two CSI connectors. Stereo removes monocular scale ambiguity and puts obstacle avoidance and mapping back on the table. Better value than the OAK-D.
- Longer-range rangefinder (TFmini-S 12 m, or Lightware) — TF-Luna's 8 m ceiling limits altitude work

Out of reach on this platform: real-time deep learning perception (no NPU, weak GPU — YOLOv8n at 320px is ~5 fps), serious obstacle avoidance (monocular, no depth, no proximity in firmware), mission-length endurance.

---

## Reference

- ArduPilot custom build: `custom.ardupilot.org` (order matters: vehicle → version → board)
- Firmware features: `firmware.ardupilot.org/Copter/stable/speedybeef4v4/features.txt`
- Frame CAD: `github.com/tbs-trappy/source_one`
- Camera driver: `dtoverlay=ov9281,cam0` in `/boot/firmware/config.txt`, Cam0 only. Set `camera_auto_detect=0` alongside it
- Camera tools: `rpicam-*` on Pi OS Bookworm and later, **not** `libcamera-*` (renamed; `rpicam-apps-lite` is the headless package). Capture via `rpicam-raw` only — the processed RGB path returns zeros, see *Camera bringup*
- Camera tuning file: `/usr/share/libcamera/ipa/rpi/pisp/ov9281_mono.json`, ships with Pi OS 13 (Pi 5 uses the `pisp` IPA path, not `vc4`)
- Benchmarks cited: SMF-VO, arXiv 2511.09072 (Pi 5 VIO timings); Isaac ROS cuVSLAM (Jetson comparison)
