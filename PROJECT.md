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
| Sim harness | **In progress** — Ubuntu 24.04 arm64 in UTM. Milestone 1 done (SITL + Gazebo Harmonic via `ardupilot_gazebo`, Guided flight). OpenVINS built and **validated on EuRoC V1_01_easy: ATE RMSE 0.115 m, RPE 0.72 %/10 m, scale 1.000** |
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
- **`pkill -f` matches your own shell** when the pattern appears in its command line. Use `pkill -x` on the process name.

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
