# VIO Quad — Build Plan

Phases 1, 2, and 3 are independent. Run them in parallel.

---

## Phase 1 — Airframe triage
*Needs: existing quad, smoke stopper. Stay on Betaflight for this whole phase.*

1. Props off. Inspect frame for cracked arms and delamination; check motor wires for chafe against carbon; check ESC/battery pads for cold joints and lifted traces; check capacitor isn't bulged.
2. Spin each motor by hand — grit means bearings, wobble means bent shaft. Back out one motor screw and confirm it isn't reaching the stator windings.
3. Multimeter across battery pads. Brief beep as the cap charges, then open. A dead short means stop.
4. USB power only. Connect Betaflight Configurator, run `diff all`, save the output.
5. First battery connection through the smoke stopper, props off. Listen for four ESC init tones.
6. Motors tab — each motor individually, correct direction, no grinding.
7. Bind EP2, verify all channels, test failsafe by switching the TX off.
8. Check batteries: resting voltage, no puffing, internal resistance if the charger reports it.
9. Props on. Outdoors, open area. 30-second hover, land, check motor and ESC temperature. Then two minutes of gentle flying.

**Gate:** flies cleanly on Betaflight.

---

## Phase 2 — Pi and IMU bench bringup
*Needs: Pi 5, SD card, PSU, ISM330DHCX.*

1. Flash Raspberry Pi OS with Imager. Set hostname, WiFi, and SSH in advanced options — headless, no monitor. Learn the green ACT LED's end-of-shutdown blink pattern now, so you can read "safe to unplug" at the field.
2. Wire ISM330DHCX to SPI0. Solder to the SPI pads, not the STEMMA QT connector. Run INT1 to a spare GPIO.
3. Device tree overlay for `st_lsm6dsx`. Confirm it enumerates under `/sys/bus/iio/devices/`.
4. Verify FIFO and hardware timestamps. Log 10k samples, histogram the timestamp deltas. Ragged deltas mean software timestamping — fix before proceeding.
5. Allan variance run: several hours stationary. Extract gyro/accel noise density and bias instability. These are config inputs for the estimator.
6. Set up cross-compilation on your desktop. On an Apple Silicon Mac this is native, not cross: `docker run --platform linux/arm64 -it -v "$PWD":/work debian:trixie`. **Match the tag to the Pi's release — confirmed Debian 13 (trixie)**; a bookworm container links against a different glibc and the mismatch surfaces as binaries that look corrupt on the Pi. Use the container for heavy third-party C++ (OpenCV, Ceres, OpenVINS); build anything touching libcamera, kernel headers, or IIO on the Pi itself.

**Gate:** IMU streaming at target ODR with tight timestamps.

---

## Phase 3 — Sim harness
*Needs: desktop only.*

1. ArduPilot SITL + Gazebo Harmonic via `ardupilot_gazebo`. Quad flying in Guided with GPS.
2. Add camera and IMU sensors to the model SDF with realistic noise and bias random walk.
3. Build OpenVINS (or VINS-Fusion). Run it on EuRoC and TUM-VI first — known data, known answer.
4. Set up `evo`. Produce ATE and RPE plots from the dataset runs.
5. Run the estimator against sim camera + sim IMU. Log only, compare to Gazebo ground truth.
6. Write the bridge node: estimator output → ENU-to-NED conversion → `VISION_POSITION_ESTIMATE` over MAVLink.
7. **Milestone 4:** `EK3_SRC1` = GPS, `EK3_SRC2` = ExternalNav, `RCx_OPTION = 90` to switch. Fly with GPS primary and confirm the two agree. Any frame error shows here.
8. **Milestone 5:** disable GPS mid-flight, confirm EKF3 holds position on vision alone.
9. Inject a known 30 ms camera-IMU offset. Confirm online `td` estimation recovers it.

**Gate:** Loiter on vision alone in sim, with time-offset estimation demonstrated.

---

## Phase 4 — Camera bringup
*Needs: InnoMaker OV9281, Pi from Phase 2.*

1. `dtoverlay=ov9281` in `/boot/firmware/config.txt`. **Cam0 port only** on the Pi 5.
2. If libcamera errors on a missing tuning file, that's the known JSON issue — get it from InnoMaker support.
3. Confirm `libcamera-hello` streams. Set resolution to 640×400, manual exposure, fixed gain.
4. Set focus: point at something 100 m+ away, maximize Laplacian variance while turning the lens. Lock with blue threadlocker.
5. Kalibr camera intrinsics. Use a fisheye model (equidistant or double-sphere), not pinhole+radtan. Target must reach the image corners.
6. Kalibr camera-IMU extrinsics and time offset. Build the bracket first: **one plate (printed PETG ~4 mm, 100% infill), sensors bolted back-to-back on opposite faces**, 2–3 mm standoffs, no joint anywhere between them. Damping goes between the *bracket and airframe*, never between the sensors. Strain-relieve the CSI ribbon and SPI wires to the plate. Align IMU axes to camera axes so the solved transform should read near-identity plus a known 90° — if it doesn't, something is wrong. Witness-mark the screws; calibrate the final assembly and do not disassemble it afterward.
7. Run the estimator on live handheld data. Walk a loop, return to start, check closure error.

**Gate:** VIO produces a sane trajectory on real data.

---

## Phase 5 — ArduPilot transition
*Needs: Phase 1 complete.*

1. `custom.ardupilot.org` → Add a build → Copter → latest stable → board `speedybeef4v4` (lowercase, sorts at the **bottom** of the list, past everything starting with a capital).
2. Enable: `HAL_VISUALODOM_ENABLED`, `EK3_FEATURE_EXTERNAL_NAV`.
3. Cut: OSD stack (`OSD_ENABLED`, `OSD_PARAM_ENABLED`, `HAL_OSD_SIDEBAR_ENABLE`, `HAL_WITH_MSP_DISPLAYPORT`, `AP_MSP_INAV_FONTS_ENABLED`), VTX (`AP_VIDEOTX`, `AP_SMARTAUDIO`, `AP_TRAMP`), all camera symbols, all telemetry except CRSF (`AP_FRSKY_*`, `AP_GHST_TELEM`, `HAL_SPEKTRUM_TELEM`), all RC protocols except CRSF, non-quad frames (`HEXA`, `OCTA`, `TRI`), `AP_AIRSPEED_ENABLED`, `HAL_PARACHUTE_ENABLED`, `AP_LANDINGGEAR_ENABLED`, `MODE_TURTLE`, `MODE_FLIP`, surplus compass backends, NMEA GPS drivers, unused rangefinder variants.
4. Generate. If it builds, it fits.
5. Flash: DFU with bootloader button held, load `arducopter_with_bl.hex`.
6. Reconfigure from scratch — motor order, ESC protocol, RC on UART2 via CRSF, GPS, compass.
7. Set small-quad starting gains: `INS_GYRO_FILTER` ~50–60 Hz, scaled-down `ATC_RAT_*` P and D. Defaults will oscillate.
8. Hover, then Autotune (QuickTune is compiled out).

**Gate:** flies on ArduPilot as well as it did on Betaflight.

---

## Phase 6 — Payload integration
*Needs: Phases 1, 2, 4 complete; GPS, BEC, mounts (printed or hand-cut).*

1. Only the **camera+IMU bracket must be rigid** — Kalibr extrinsics are baked in, so any relative motion corrupts every observation. Criterion is **stiffness** (resonance), not creep: print it **~4 mm thick at 100% infill**, which beats 1.6 mm FR4 in bending (thickness cubed outruns the 10× modulus gap) for ~3 g. Through-bolts with nuts, never threads into plastic. Verify by tapping the assembly and looking for a resonance peak in the accel spectrum once the IMU streams. Pi tray (58 × 49 mm) and the rest tolerate nylon standoffs + zip ties. Printed parts are a convenience, not a requirement; PETG if printing (no PLA — 60 °C Tg, creeps under sustained load in sun). Dry the spool before each session; PETG absorbs moisture between bursts. Damping via off-the-shelf silicone balls.
2. **Layout: bottom-mount the battery, Pi at top-plate level.** The battery is ~4× the Pi's mass, so whichever side it sits on sets the CG — battery low puts CG ~6 mm above the bottom plate vs ~32 mm with the battery on top and the Pi above it. It also keeps the SD slot, USB, and GPIO reachable instead of under a battery strap. Cost: landing gear tall enough (~35 mm) that the pack isn't the first thing to touch down. Camera+IMU bracket cantilevers off the nose; GPS mast on top, clear of current-carrying leads; TF-Luna facing down; BEC anywhere convenient.
3. Momentary shutdown button (GPIO3 ↔ GND, `dtoverlay=gpio-shutdown`) somewhere reachable with props on.
4. **Consider dropping the Active Cooler for a low-profile passive heatsink** — it's sized for still desk air and you have prop wash. Saves height and ~10 g. Don't assume it: log CPU temp through the hover test and decide from data. Keep the active cooler for bench work regardless.

   *Bench baselines measured 2026-08-27 (Pi 5 4GB, idle, room ambient):* passive/no heatsink with desktop session running — **51.0 °C**; Active Cooler fitted, headless (`multi-user.target`) — **29.6 °C**. The 21 °C delta is the number a passive heatsink has to close under prop wash. Note the two readings differ in load as well as cooling, so re-measure passive+headless if the decision gets close.
5. Wire: BEC input to ESC `BAT+`/`BAT-` (22 AWG is fine, ~1–2 A). BEC 5 V output → Pi GPIO pins 4 & 6, 20 AWG, short. Optional 3 A inline fuse on the BEC input — nothing in this system is fused. **Meter the BEC output under load before it ever touches the Pi** — the 12S Pro's output rail is selectable and anything above 5 V kills the Pi instantly. Add `usb_max_current_enable=1` to config.
6. Pi UART (GPIO 14/15) ↔ FC UART3, TX/RX crossed, common ground, 921600.
7. GPS to a UART + I2C for compass; power from the FC's 4.5 V pad so it stays live on USB during bench config. `GPS_TYPE=2` (u-blox), **not Auto** — Auto can fall back to NMEA and lose 3D Doppler velocity. TF-Luna on I2C to save a UART.
8. Weigh it. Check CG. Confirm hover throttle lands in the 40–50% band.
9. Hover test with full payload. Watch for brownouts and check vibration in the logs.

**Gate:** flies with payload, no brownouts, hover throttle in range.

---

## Phase 7 — Vision in the loop
*Needs: Phases 3, 5, 6 complete.*

1. `VISO_TYPE = 1`. Set `VISO_POS_X/Y/Z` to the camera's offset from the IMU.
2. `EK3_SRC1` = GPS, `EK3_SRC2` = ExternalNav. `RCx_OPTION = 90` for source switching. `RCx_OPTION = 80` for Viso Align — use it before every flight.
3. Set `VISO_QUAL_MIN` so bad vision output gets rejected rather than fused.
4. Fly with GPS primary. Log GPS and vision together. Confirm agreement before trusting anything.
5. At altitude with room, switch to ExternalNav. Hand on the switch to go back.
6. Once stable, disable GPS mid-flight and confirm Loiter holds.

**Gate:** position hold on vision alone, outdoors.

---

## Phase 8 — Evaluation
1. Record raw camera frames and IMU to the 128 GB card every flight. Fly to collect data, not to test hypotheses.
2. Replay offline against parameter sweeps. One flight, fifty configs.
3. ATE and RPE via `evo`, against GPS velocity initially.
4. Add the F9P + antenna for PPK ground truth once VIO is worth grading. Log RXM-RAWX, pull RINEX from the nearest NOAA CORS station, post-process in RTKLIB.
5. Vary altitude, speed, yaw rate, ground texture, sun angle. Five runs minimum per condition.

---

## Still to buy
- Any ≤32 GB **SDHC** card for FC blackbox, format FAT32 (**not SDXC**) — commodity, ≤$10
- 20 AWG wire, blue threadlocker, momentary pushbutton, silicone damping balls (XT60 pigtail optional, bench only)
- 3D printing access
- Later: ArduSimple simpleRTK2B + multiband antenna (~$271)
