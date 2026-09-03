# PROJECT.md — GPS-Denied VIO Quadcopter

## Objective

Build a monocular visual-inertial odometry system on a 5-inch quadcopter that can hold position outdoors without GPS, and characterize its drift against measured ground truth. The deliverable is not "it flies" — it is a number: drift as a percentage of distance traveled, across varied altitude, speed, and terrain, with the evaluation harness that produced it.

Secondary objective: the sim harness and evaluation framework are reusable for any estimator, and are arguably more valuable than the specific VIO implementation.

---

## Current status

| Phase | State |
|---|---|
| Airframe triage | **Complete (2026-08-27)** — flies cleanly on Betaflight 2026.6.1. Gate met: stable hover, even motor temps, failsafe verified, arm/disarm on ELRS. Residuals: one motor ticks when hand-spun (random, no play — debris; no gyro or thermal signature under load), and level trim left rough deliberately since ArduPilot redoes it |
| Pi + IMU bench bringup | **In progress** — Pi 5 up (`viopi`, Pi OS 13 trixie, kernel 6.18.39+rpt-rpi-2712), SD verified genuine via f3, SPI enabled, Active Cooler fitted. Next: IMU wiring |
| Sim harness | **In progress** — Ubuntu 24.04 arm64 in UTM. Milestones 1–4 done. OpenVINS **validated on EuRoC V1_01_easy: ATE RMSE 0.115 m, RPE 0.72 %/10 m, scale 1.000**. On our own sim: **15.4–16.0 % drift, ATE 2.5 m over 156 m**, two flights × three replays, down from 97.8 % — see 2026-09-02. Milestone 3 produces a trajectory and an evo comparison, but **the < 5 % drift gate on it is not met**; residual is leg-to-leg magnitude inconsistency, and the untried levers are structural (no-stop mission, stereo) rather than parameters. Milestones 5 and 6 not started |
| Camera bringup | Camera purchased — Pi now available, not yet started |
| ArduPilot transition | Firmware constraint confirmed, build not yet generated |
| Payload integration | Printer in hand; mounts not yet designed. Blocked on Phases 1/2/4 |
| Vision in the loop | Blocked on all of the above |
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
| UART | Device | Protocol | Baud |
|---|---|---|---|
| 2 | ELRS EP2 | CRSF | 420000 |
| — | GPS | uBlox | 230400 |
| — | Pi 5 | MAVLink 2 | 921600 |
| I2C | Compass + TF-Luna | — | — |

TF-Luna on I2C rather than serial specifically to free a UART. Verify exact UART numbers against the V4 pinout.

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
5. GPS disabled mid-flight, EKF3 holds position on vision alone
6. Headless batch runs across altitude/speed/texture, ATE and RPE out

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
| Optical flow (3901-L0X) | Four features to re-enable, competing with external nav for flash |

---

## Where priors failed

Recorded because the failure mode is systematic, not incidental.

- **`SpeedyBeeF405v4`** — the ArduPilot board is `speedybeef4v4`, lowercase, "F4v4". Assumed CamelCase consistency with the other SpeedyBee targets; it isn't there.
- **Arducam URL pattern-matching** — constructed `mini-ov9281-mono-global-shutter-wide-angle-for-pi.html` from the non-wide-angle page. The URL happened to be right, but `arducam.com/*.html` pages are **catalog pages with no cart**. Arducam retail is UCTRONICS, Amazon, or `arducam.com/product/*`. This wasted two rounds.
- **"Skip the 27 W power supply"** — reasoned from the flight configuration (BEC-powered) and ignored that weeks of bench work come first, where a wall supply is required.
- **Amazon search results** — a "raspberry pi 5" query surfaced a Pi 4 kit as result #2, because accumulated reviews outrank relevance. B07-prefix ASINs are 2019; anything genuinely Pi 5 is B0C or later.
- **BEC "5 A" listings** — three consecutive Amazon results matching "5 V / 5 A BEC" were 1.5–3 A continuous. FPV BEC marketing quotes peak universally.
- **BNO085 recommendation reversal** — recommended over BMI270 in the OAK-D context (comparing two options you don't control, where DepthAI exposes raw output), then rejected for standalone use. Both are correct in context; the earlier statement was scoped and read as general.

The pattern: pattern-matching from adjacent cases produces plausible answers that fail on specifics. Verify SKUs, URLs, and continuous ratings rather than inferring them.

---

## Open questions

- Airframe condition unknown — cracked arms, damaged ESC FETs, and bent motor shafts are all live possibilities. Triage gates everything.
- Battery capacity, connector, and health unverified (cell count confirmed 4S).
- Whether visual odom + external nav actually fit in 1 MB alongside the rangefinder. Build server will answer definitively.
- 3D printing is now optional, not blocking — only the camera+IMU bracket needs rigidity and it is hand-cuttable from FR4. Printer purchase deferred until mount iteration actually bottlenecks.
- Camera works on **Cam0 port only** on Pi 5 per user reports; a missing libcamera tuning JSON in default Raspbian requires vendor support to resolve.
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
- Camera driver: `dtoverlay=ov9281` in `/boot/firmware/config.txt`, Cam0 only
- Benchmarks cited: SMF-VO, arXiv 2511.09072 (Pi 5 VIO timings); Isaac ROS cuVSLAM (Jetson comparison)
