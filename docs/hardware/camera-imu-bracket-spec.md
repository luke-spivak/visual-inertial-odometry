# Camera + IMU Bracket — CAD Specification

VIO sensor bracket for TBS Source One V5. Two printed parts.
Printer: Bambu A1 mini. Material: PETG.

---

## 0. How to build this

- **All dimensions are millimetres.**
- **Model it parametrically.** `TILT`, `CAM_Y`, `CAM_Z`, `FIT_HOLE` must be
  editable named parameters. `TILT` and `CAM_Y` are expected to be revised once
  the camera is streaming and you can see what is actually in frame.
- `FIT_HOLE` is not yet known. Use **+0.20 mm** as a placeholder; it is the same
  number calibrated in the landing-leg spec §9 and can be carried across.
- **Deliverables:** STEP for the assembly, plus separate STLs/3MFs for Part 1
  and Part 2.
- **Part 1 is the calibrated item.** Once Kalibr has solved the camera–IMU
  extrinsics, it is never disassembled. Every geometry decision that might be
  revised must therefore live in Part 2.

---

## 1. Design intent

Two parts:

- **Part 1 — sensor plate.** Flat 4 mm PETG plate, 100% infill. Camera on the
  front face, IMU bolted through to the back face on standoffs. No joint of any
  kind between the two sensors. This is the rigid body Kalibr calibrates.
- **Part 2 — nose arm.** Bolts to four existing M3 standoff positions under the
  middle plate, cantilevers forward, and presents a flat face at `TILT` degrees
  for Part 1. Carries all revisable geometry: tilt, forward reach, and the
  optional damping interface.

**No silicone damping balls in the baseline build.** Rationale in §8, along with
the retrofit path and the measurement that decides it.

---

## 2. Frame coordinates and Source One hardpoints

All frame geometry below is extracted from the official CAD at
`github.com/tbs-trappy/source_one`, tag `v5` — `SO_V5_MiddlePlate_2mm.dxf`,
cross-checked against `SO_V5_BottomPlate_2mm.dxf` and
`SO_V5_5inch_Arm_6mm.dxf`.

### Coordinate system
- **Origin** = FC stack centre = frame centre (the 30.5 × 30.5 pattern centre)
- **+Y** = forward, toward the nose
- **+X** = right
- **+Z** = up
- **Z datum** = bottom-plate underside (same datum as the landing-leg spec)

### Hardpoints — measured from the DXF
| Feature | Position | Notes |
|---|---|---|
| Stack pattern | 30.5 × 30.5 and 20 × 20, centred at origin | Ø4.5 in middle plate |
| **Aft standoff pair** | **(±9.50, +32.88)** | M3, Ø3.0 |
| **Nose standoff pair** | **(±9.50, +76.88)** | M3, Ø3.0 — the primary hardpoint |
| Camera-cage tab slots | 1.50 × 4.50 at (±5.50, +77.54) | Confirms +Y is the nose |
| Motor centres | radius 113.0, at ±45° → (±79.9, ±79.9) | 226 mm diagonal |
| Prop disc | radius 63.5 about each motor centre | 5" prop |

The two standoff pairs are 44.00 mm apart in Y and both are 19.00 mm across in
X. They carry the top plate, so they are structural and already fastened — the
bracket flange is simply captured under the middle plate by lengthening the
existing M3 screws by 4 mm.

**The FPV camera cage is not used.** VTX and camera support are compiled out of
the firmware in Phase 5, so the cage plates come off and the nose is free.

### `Z_MID` — already determined, no measurement needed
`Z_MID` is the height of the **middle-plate underside** above the bottom-plate
underside. It is where Part 2's flange bolts up, so it sets the whole bracket's
height.

The middle plate sits directly on top of the arms, so:
```
Z_MID = H_ARM_UNDER + ARM_T = 3.0 + 6.0 = 9.0 mm
```
Both terms are already known — `H_ARM_UNDER` measured, `ARM_T` from the arm DXF.

The only thing that could break this is a washer or spacer between the arm and
the middle plate. Look at the edge of the sandwich; if there is one, add its
thickness. To check directly: stand the frame on a flat table on its bottom
plate and run a caliper depth rod down to the table from the middle plate's
**top** face at the nose (open area, nothing in the way), then subtract the
plate thickness of 2.0 mm from `SO_V5_MiddlePlate_2mm.dxf`.

### Cable width — and a variant warning
CSI ribbon widths follow directly from pin count and pitch:

| End | Pins | Pitch | Ribbon width |
|---|---|---|---|
| Pi 5 ("mini") | 22 | 0.5 mm | **~11.5 mm** |
| Camera ("standard") | 15 | 1.0 mm | **~16.0 mm** |

**The measured 11 mm is the 22-pin Pi-5 end, not the camera end.** If that is
genuinely the widest end of the cable, then both ends are mini and it will not
mate with the InnoMaker camera's 15-pin connector — precisely the risk flagged
at [PROJECT.md:52](../development-log.md). Count the contacts at each end (15 vs 22) to
settle it.

The strain relief in §4.6 is sized for the 16 mm case, so it works either way
and this does not block the CAD.

---

## 3. Parameters

| Name | Value | Source |
|---|---|---|
| `TILT` | **30°** nose-down | §7 |
| `CAM_Y` | **+120.0 mm** | Camera front-face centre, forward of frame centre. §7 |
| `CAM_Z` | **+12.0 mm** | Camera front-face centre height. See §7 — at +2 the plate's lower edge hung 21 mm below the frame. |
| `PLATE_T` | 4.0 mm | Sensor plate thickness — do not reduce, see §1 |
| `CAM_HOLES` | 28.0 × 28.0, M2 | InnoMaker CAM-MIPI9281RAW-V2, confirmed from manual V1.4 |
| `CAM_BOARD` | 32.0 × 32.0 mm | Same source |
| `IMU_STANDOFF` | 3.0 mm | Metal, not nylon — nylon creeps under preload. Must exceed the M2 camera-nut height on the back face. |
| `IMU_PCB` | 25.40 × 17.78 mm | Adafruit 4502 STEP file |
| `IMU_HOLES` | 2 × Ø2.50, **20.32 mm apart** | Adafruit 4502 STEP file |
| `IMU_ENVELOPE` | 28.40 × 20.26 mm | Includes the STEMMA QT connector overhang |
| `FIT_HOLE` | +0.20 mm placeholder | Carried from landing-leg calibration |

Cantilever from the nose standoff to the camera is `CAM_Y - 76.88` = **43.1 mm**.

---

## 4. Part 1 — Sensor plate

### 4.1 Local frame
The plate lies in the plane tilted `TILT` from vertical. In frame coordinates:
- plate **u** (right) = `(1, 0, 0)`
- plate **v** (up)    = `(0, sin TILT, cos TILT)` = `(0, 0.500, 0.866)`
- plate **n** (outward, toward the scene) = `(0, cos TILT, −sin TILT)` = `(0, 0.866, −0.500)`

Plate origin `(u,v) = (0,0)` sits on the front face at `(0, CAM_Y, CAM_Z)`.

### 4.2 Body
- **38.0 (u) × 54.0 (v) × 4.0 (n)**, all outer edges filleted R2
- Plate spans u = −19.0 … +19.0, v = −27.0 … +27.0
- 100% infill. This is the one part in the build where solid fill is correct:
  the criterion is bending stiffness, and thickness³ is doing the work.

### 4.3 Camera — front face
- 4 × Ø `2.4 + FIT_HOLE` through, on a **28.0 × 28.0** square centred at
  `(u,v) = (0, +8.0)` — hole centres at u = ±14.0, v = −6.0 and +22.0
- Board footprint spans u = ±16.0, v = −8.0 … +24.0. Keep this area clear of any
  other feature on the front face.
- **Edge margins:** every camera hole keeps ≥3.7 mm of material to the nearest
  plate edge. This is why the plate is 38 × 54 and not smaller — do not shrink it.
- **Fastener — length depends on the board's rear face.** Check where the CSI
  connector sits before ordering:

  | Camera board rear | Standoff | Screw | Stack under head |
  |---|---|---|---|
  | Clear — board seats flush | none | **M2 × 8** | 1.0 PCB + 4.0 plate + 1.6 nut = 6.6 |
  | Connector/components on the back | **3.0 mm M2 spacers** | **M2 × 12** | 1.0 + 3.0 + 4.0 + 1.6 nut = 9.6 |

  Flush is stiffer — use standoffs only if the rear face forces it. Standoffs
  here are legitimate: [vio-quad-build-plan.md:63](../archive/vio-quad-build-plan.md)
  specifies 2–3 mm standoffs on both sensors, and a metal spacer inside a
  preloaded bolt path is not a compliance joint.
- **Plain M2 nuts (1.6 mm), NOT nyloc.** The relief pockets in §5.3 are 2.5 mm
  deep; an M2 nyloc stands ~2.5 mm and would bottom out, holding Part 1 off its
  bearing pad. Use **blue threadlocker** instead.
- Never a thread into plastic. Witness-mark all four after Kalibr.
- The two lower nuts (v = −6.0) sit inside Part 2's bearing pad and require the
  relief pockets specified in §5.3; the two upper nuts (v = +22.0) sit inside the
  IMU clearance pocket.

### 4.4 IMU — back face, directly behind the camera

**Board geometry, extracted from Adafruit's own STEP file** —
`github.com/adafruit/Adafruit_CAD_Parts`, path `4502 6DoF IMU/4502 6DoF IMU.step`.
Import that file directly rather than modelling from these numbers.

| | |
|---|---|
| PCB outline | **25.40 × 17.78 mm** (1.0" × 0.7") |
| Mounting holes | **2 × Ø2.50 mm**, spaced **20.32 mm** (0.8") |
| Hole position | both on one long edge, **2.54 mm** in from it |
| Component height | 2.96 mm above PCB top, 0.21 mm below |
| Envelope with STEMMA connectors | 28.40 × 20.26 mm |

**There are only two mounting holes, and they lie on one edge** — bolting them
alone leaves the board free to hinge about that line. The mount must add a third
constraint:

- Board long axis along plate **u**, hole line horizontal, centred at
  `(u,v) = (0, +8.0)` — coaxial with the camera, minimising the lever arm in the
  solved extrinsics. Envelope then spans u = ±14.2, v = −2.13 … +18.13.
- **Components face aft**, away from the plate — they stand 2.96 mm proud and
  would foul a 3.0 mm standoff if turned inward.
- 2 × **M2.5 through-bolts** at the holes, on **3.0 mm metal standoffs**
- **Retention tab:** a printed lip overhanging the opposite long edge by 1.5 mm
  at standoff height, so the board slides under it. Bolts hold the near edge,
  the tab captures the far edge. Fully constrained, no extra fasteners.
- **Keying lip:** 1.0 mm tall, following three sides of the board outline with
  0.15 mm clearance, fourth side open for the slide-in. Two holes alone permit a
  180° error; this makes the wrong orientation physically impossible.
- **Axis convention:** IMU board silkscreen **+X → plate +u**, **+Y → plate +v**.
  The board's +Z then points out of the back face, anti-parallel to the camera's
  optical axis. Record this convention. Kalibr should return a rotation composed
  only of 90° multiples — any residual beyond a degree or two means the mount or
  the convention is wrong, and that is the whole point of aligning the axes.

  **Measured (2026-09-10):** Kalibr returns identity + 2.2°, not the 180°
  about camera x predicted here — the axes the driver reports are aligned with
  the camera's, whatever the silkscreen says. The gyro fit confirms it (see
  PROJECT.md, *Camera-IMU extrinsics*). Use Kalibr's rotation, not this
  convention.

  **Re-measured (2026-09-12)**, after the IMU was re-oriented: 180° about
  camera y + 4.4° (PROJECT.md, *Camera-IMU again*).


### 4.5 Mounting to Part 2
- 2 × Ø `3.2 + FIT_HOLE` through, at `(u,v) = (±12.0, −19.0)`
- **Bearing pad:** the back face from v = −27.0 to −4.0 is flat and bears
  directly on Part 2. This pad, not the bolt spacing, reacts the pitching
  moment — the couple arm is the pad height (~15 mm above the bolt line).
- Fastener: **M3 × 16 socket cap, inserted from Part 1's front face**, into a
  captive hex nut pocketed in Part 2's rear. Bolt path is 4.0 (Part 1) + 6.0
  (Part 2 face) = 10 mm plus nut engagement. Heads sit at v = −19, well below
  the camera board and far outside its field of view.

### 4.6 Strain relief
- 2 zip-tie slots, **3.5 (u) × 1.5 (v)**, at `(u,v) = (±15.0, −23.5)` — 30 mm
  clear between them, sized for a 16 mm standard ribbon; an 11.5 mm mini ribbon
  ties down in the same slots
- Both the CSI ribbon and the IMU's SPI wires tie to **this plate**, so that no
  cable tension ever reaches a connector. Part 2 has a matching relief groove
  so the tie can pass behind.

### 4.7 Witness marks
- A 2.0 × 4.0 × 0.4 mm raised flat adjacent to each of the 4 camera screws, the
  2 IMU screws and the 2 mounting screws, for a paint pen after calibration.

---

## 5. Part 2 — Nose arm

Three regions, one printed part.

### 5.1 Rear flange
- Flat, horizontal, **4.0 mm thick**, top face at `Z = Z_MID` (+9.0), bearing
  directly against the middle-plate underside
- Spans **Y = +25.0 … +85.0**, width **26.0 mm** (X = ±13.0). Note the middle
  plate itself ends at **Y = +81.4**, so the forward 3.6 mm of flange is
  unsupported transition into the spine — do not expect bearing there.
- 4 × Ø `3.2 + FIT_HOLE` at `(±9.50, +32.88)` and `(±9.50, +76.88)`
- Lighten the middle of the flange with a cutout; keep 5 mm of material around
  every bolt and along both edges
- Existing standoff screws are replaced with ones **4 mm longer**

### 5.2 Spine
- From the flange front (Y = +85) forward to the mounting face
- **Ribbed box section, 14.0 mm wide × 12.0 mm deep**, centred on X = 0
- Wall 2.0 mm with a central vertical rib. **The depth is the point** — a flat
  4 mm tongue over this span lands near 150 Hz, inside the motor's operating
  range. A 12 mm ribbed section puts the first mode above 400 Hz. See §8.
- Fillet R4 where the spine meets the flange

### 5.3 Mounting face
- Flat face, normal `(0, cos TILT, −sin TILT)`, positioned so Part 1's front
  face lands at `(0, CAM_Y, CAM_Z)` given `PLATE_T`
- Face extent matches Part 1: **38.0 × 54.0**
- **Solid thickness 6.0 mm** behind the bearing pad — this sets the M3 bolt length
- **IMU clearance pocket:** 34.0 (u) × **28.0** (v) × 10.0 deep, centred at
  `(u,v) = (0, +10.0)` (spans v = −4.0 … **+24.0**). The 28 mm height is set by
  the upper camera nuts at v = +22.0, which must fall inside the pocket rather
  than on its rim. With a wire exit channel running
  down from its lower edge. Depth clears the 7.6 mm IMU stack — standoff 3.0 +
  PCB 1.6 + components 2.96 — with margin.
- **Bearing pad:** the face from v = −27.0 to −4.0 is solid and flat — this is
  what Part 1 clamps against
- **Camera-nut relief:** 2 × Ø7.0 × 2.5 deep pockets at `(u,v) = (±14.0, −6.0)`.
  Without these the M2 camera nuts hold Part 1 off the pad and the joint rocks.
- 2 × Ø `3.2 + FIT_HOLE` at `(±12.0, −19.0)`, with a captive hex nut pocket on
  the rear face: 5.5 mm across flats, 2.6 mm deep
- Relief groove across the pad at v = −23.5, 4.0 wide × 2.0 deep, so the Part 1
  zip ties can pass

### 5.4 Cable routing
- A channel along the underside of the spine, 6.0 wide × 3.0 deep, with two
  zip-tie slot pairs, carrying the CSI ribbon and SPI wires aft to the Pi
- All channel edges R1 minimum

---

## 6. Print settings

### 6.1 Part 1 — sensor plate

**Orientation: flat on the bed, camera face DOWN.** The bed side is the only
perfectly planar face on the part; the camera is the sensor whose alignment
matters most, so it gets that face. The IMU keying lip and retention tab then
print upward as raised features, which is the easy direction. Chamfer the
retention tab's underside at 45° so its 1.5 mm overhang needs no support.

Start from **Generic PETG** and change these:

| Bambu Studio section | Setting | Value | Why |
|---|---|---|---|
| Quality | Layer height | **0.20 mm** | 4.00 / 0.20 = exactly 20 layers. See note below. |
| Quality | Initial layer height | **0.20 mm** | Keeps the layer count integer |
| Quality → Precision | Elephant foot compensation | **0.15 mm** | Stops the bed-side edge bulging and lifting the camera board |
| Quality → Precision | **X-Y hole compensation** | **0** | **Trap — see §6.3** |
| Strength → Shells | Top shell layers | **10** | |
| Strength → Shells | Bottom shell layers | **10** | 10 + 10 = 20 = the entire part is solid shell |
| Strength → Walls | Wall loops | **4** | |
| Strength → Infill | Sparse infill density | **100%** | Belt-and-braces; the shells already fill it |
| Strength → Infill | Sparse infill pattern | **Rectilinear** | |
| Strength → Infill | Infill/wall overlap | **25%** | |
| Cooling | Fan max speed | **30%** | **The single biggest lever — see §6.2** |
| Cooling | Fan min speed | **20%** | |
| Cooling | First layers without fan | **3** | |
| Filament | Nozzle temperature | **250 °C** | Top of ELEGOO's range, for layer bonding |
| Speed | Outer wall | **30 mm/s** | |
| Speed | Internal solid infill | **50 mm/s** | |

No brim needed — PETG grips textured PEI hard. Print it alone, not batched with
other parts, so no travel moves string across the sensor faces.

### 6.2 The two settings that actually decide whether this part works

**Cooling.** PETG layer adhesion is destroyed by aggressive part cooling, and
this is a part whose entire purpose is stiffness. Generic PETG profiles run the
fan far higher than a strength part wants. 30% max is the compromise: enough to
stop droop on a 38 × 54 footprint, low enough that layers actually weld.

**Dry filament.** [PROJECT.md:44](../development-log.md) already says to dry the spool
before each session. On this part it is not cosmetic — wet PETG gives visibly
weaker interlayer bonds, and interlayer bonding is what carries bending stress
through the plate's thickness.

### 6.3 Do not double-compensate the holes

`FIT_HOLE` is baked into the CAD geometry per §3. Bambu Studio's **X-Y hole
compensation** does the same job in the slicer. Applying both stacks two
corrections and the M2 camera holes end up ~0.4 mm oversize. **Set X-Y hole
compensation to 0** and let the model carry it.

### 6.4 Layer height — why 0.20 and not 0.15

The plate is exactly 4.00 mm. At 0.15 mm that is 26.67 layers, so the slicer
delivers either 3.90 or 4.05 mm and the thickness³ stiffness argument silently
moves by ±4%. 0.20 mm divides it exactly into 20 layers. Thicker layers also
bond better in PETG, which is the direction this part wants.

### 6.5 Part 2 — nose arm

| | Value |
|---|---|
| Orientation | **Flange down on the bed** |
| Why | Largest flat area on the bed; spine walls build vertically; the mounting face leans 60° from horizontal, well inside the no-support range; and the 4 flange holes print as true vertical bores. Cantilever bending stress runs along Y, in-plane with the layers. |
| Layer height | 0.20 mm |
| Wall loops | 4 |
| Sparse infill | 30% gyroid |
| Cooling, temperature | As Part 1 |

No supports should be needed — verify in preview before slicing.

Generic PETG profile, A1 mini flow calibration on. Dry the spool.

---

## 7. Why 30° and why Y = +120

### Tilt
With a 118° horizontal FOV, 30° nose-down still leaves the top of the frame
~16° **above** horizontal. You keep distant horizon features for depth diversity
while filling most of the frame with close ground texture.

Pure-down would be a mistake: it puts nearly every feature on one plane, which
is the coplanar monocular degeneracy called out at [PROJECT.md:351](../development-log.md).

### Forward reach
Computed against the real motor positions (r = 113.0, prop r = 63.5). Azimuth of
the nearest prop edge as seen from the camera, versus the 59° half-FOV:

| `CAM_Y` | nearest prop edge | margin vs 59° | cantilever |
|---|---|---|---|
| +77 (at the standoff) | 35.3° | **−23.7° — props in frame** | 0 mm |
| +105 | 58.1° | −0.9° | 28 mm |
| +112 | 64.4° | +5.4° | 35 mm |
| **+120** | **71.4°** | **+12.4°** | **43 mm** |

At +120 the front props sit 40 mm *behind* the camera and fall outside the
horizontal FOV with real margin. Plan-view clearance of the plate corners is
10.2 mm outside the prop disc.

### Height
`CAM_Z` = +12.0 puts the plate's lower edge at Z = −11.4, i.e. **33.6 mm above
ground** on the 45 mm landing gear. At the originally specified +2.0 the edge sat
at Z = −21.4 and would have been the first thing to touch in any nose-over — on
the one part in this build that must never be disturbed after calibration. The
plate's top corner reaches Z = +35.4 at Y = +133.5, which is 17.6 mm clear of the
front prop discs in plan view.

Caveat: the lens is **148° diagonal**, so the image corners reach ~74° and may
still clip the props. Expect to apply a static image mask rather than trying to
solve it mechanically. Going further forward costs cantilever stiffness fast.

**Superseded by measurement (2026-09-10).** Kalibr puts this lens at
**H 69° / V 42° / D 83°**, not 118° / 148° (PROJECT.md, *Intrinsics: the lens
is 69°, not 118°*). For the part as built:

- **Tilt.** With ±21° vertical, 30° nose-down puts the top of the frame ~8°
  *below* horizontal — no horizon in view, nearly all features on the ground
  plane, the degeneracy this section chose 30° to avoid. About 15° would put
  the horizon back at the top edge. A VIO robustness risk to watch in testing,
  not a failure.
- **Forward reach.** The props now sit far outside a ~35° half-FOV, so +120
  carries more cantilever than needed, and the image-mask caveat above is
  moot.

Deferred, not urgent — the bracket works as built (Luke, 2026-09-10). The cost
of deferring: a reprint invalidates the camera-IMU calibration (step 6) and
any data recorded with it, so it is cheapest to decide before step 6.

---

## 8. Damping — deliberately omitted, and how to add it

### Why the baseline is rigid

- **The usual reason does not apply.** Silicone balls on FPV builds mostly exist
  to kill rolling-shutter jello. The OV9281 is **global shutter**.
- **Motion blur is negligible.** At ~150 Hz and 2 g the displacement amplitude
  is ~22 µm — about 0.001° of blur at 1 m against a 0.18°/pixel scale.
- **The real risk is IMU aliasing and clipping, and both are configuration
  problems.** Hover puts the motor fundamental near 150 Hz and 3-blade passing
  near 450 Hz. Sample the ISM330DHCX near its top ODR (it reaches 6.66 kHz),
  then digitally filter and decimate to 200–400 Hz for the estimator. Set the
  accel range to **±16 g**: at 16-bit that is 0.49 mg/LSB against a ~1 mg noise
  floor over your bandwidth, so nothing real is lost and vibration peaks stop
  clipping. Clipping rectifies into a DC bias and is fatal to preintegration.
- **A bad isolator actively hurts this use case.** Balls under a cantilevered
  nose bracket give a pendulum mode around 5–15 Hz, lightly damped — inside the
  VIO band. Worse, EKF3 assumes `VISION_POSITION_ESTIMATE` relates to the body
  frame by a **fixed** transform. A swinging bracket breaks that assumption, and
  the rotational component appears as attitude error.

### What decides it

The hover-test accel spectrum, exactly as the Active Cooler decision is made
from logged CPU temperature. Add isolation only if you see energy you cannot
filter or peaks approaching the ±16 g rail.

**Measured (2026-09-12): the rule is met.** In flight the IMU saw 43–49 m/s²
RMS side to side, all at 176–195 Hz, with peaks at 14.2 g. A tap test puts the
lateral/twist mode at **173 Hz, Q 12**, just below that band, and the
out-of-plane mode at 119 Hz. Two things soften it: the Part 1 joint, below, and
the arm twisting through its 4 mm flange (the "above 400 Hz" in §5.2 is for
bending). PROJECT.md, *VIO alongside GPS*.

**The Part 1 joint slips.** Grabbed at the top, the plate wiggles side to side
by hand. §4.5's two M3s sit on one line, so in-plane rotation is held by
friction alone, and once that slips their 0.2 mm hole clearance is ~1° of play,
~0.8 mm at the top edge. The tap test agrees: the one gentle flick rang at
184 Hz, the hard ones at 171–175 Hz, a joint that softens the harder it is
pushed.

### Retrofit: a short arm and one isolated sensor plate

A reprint with a new Kalibr run is accepted (Luke, 2026-09-12). That removes
the reason for the Part 1 / Part 2 split, which was revising the mount without
recalibrating. The rebuild has two parts, split where the balls go:

- **Sensor plate (new Part 1).** One rigid piece carrying the camera, the IMU
  and the four ball mounts. No joint between the sensors, so nothing to slip
  and no bolt pattern to get right.
- **Arm (new Part 2).** §5.1's flange and a shortened spine ending in a flat
  ball backplate. It no longer carries the payload's mass directly, so its own
  modes sit well clear of the motor band. The neck length stops mattering for
  vibration and is set by prop clearance instead.

**Why not one rigid arm-and-plate piece with no balls.** It would need its
first twist mode above ~300 Hz to clear the motors at every throttle, not only
at hover, and even then it passes the frame's own vibration through 1:1. The
measured bracket sits at 173 Hz, ~3× short in stiffness. The balls work
wherever the arm's mode lands.

**Why not at the flange/spine junction**, as this section first said: the
payload's CG sits ~37 mm ahead of that joint and the 26 mm flange limits the
ball pattern to ~20 × 20 mm. Modelled on four balls with a 45 Hz bounce, it
rocks at **10 Hz**, the pendulum described above. The balls have to surround
the payload's CG.

**Sensor plate**
- Camera on the front (4 × M2, §4.3), IMU on the back behind it (2 × M2.5 on
  metal standoffs, retention tab and keying lip, §4.4), zip-tie slots (§4.6),
  witness marks (§4.7). Solid PETG, ≥ 4 mm, per §1.
- **Four ball holes** on a rectangle centred on the assembly's CG (~3 mm above
  the camera centre with today's parts; take it from the CAD's mass
  properties) and clear of the IMU footprint (u ±14.2, v −2.1 … +18.1 about
  the camera centre) by the ball's radius: ~40 × 48 mm on a plate of about
  46 × 62. The balls' necks set the hole size and the local plate thickness.
- Mass ≈ 45 g all in.

**Arm**
- Flange as §5.1, spine shortened per the table below.
- **Ball backplate:** flat, normal `(0, cos TILT, −sin TILT)`, four matching
  holes, and a window around the IMU with ≥ 3 mm clearance. It sits behind the
  sensor plate by the ball stack height, which must also clear the IMU's 7.6 mm
  stack if the IMU does not reach into the window.

**Neck length** is now set by the sensor plate's top corners against the front
prop discs in plan view, not by the field of view (§7: the lens is 69°).
Clearance in mm:

| `CAM_Y` | cantilever | 38 × 54 @ 30° | 46 × 62 @ 30° | 38 × 54 @ 15° | 46 × 62 @ 15° |
|---|---|---|---|---|---|
| +100 | 23 mm | 6.1 | 3.6 | 3.2 | 0.0 |
| +105 | 28 mm | 8.6 | 6.4 | 5.3 | 2.3 |
| +110 | 33 mm | 11.4 | 9.4 | 7.8 | 5.0 |
| +115 | 38 mm | 14.4 | 12.6 | 10.5 | 7.9 |
| +120 (today) | 43 mm | 17.6 | 16.1 | 13.5 | 11.0 |

Keeping ≥ 8 mm, a 46 × 62 plate at 30° lands at `CAM_Y` ≈ +110, 10 mm
shorter than today; at 15° it needs ≈ +115. **Decide `TILT` before printing:**
30° as built, or ~15° to put the horizon back in view (§7). The recalibration
happens either way.

**Stiffness.** Per ball, for 45 g on a ~40 × 48 pattern: **≈ 1.3–1.9 N/mm along
the ball's axis**, ~0.6 N/mm sideways. A tighter 26 × 31 pattern needs 2–2.7
and isolates rocking better but twist worse. Modelled (6-DOF, loss factor
0.15), all modes fall between 32 and ~120 Hz, and at 184 Hz the IMU sees
0.11–0.12 of the arm's side-to-side motion and 0.21–0.24 of its twist, against
6–11× today: roughly 25–100× less.

**Travel.** Sag is ~0.05–0.1 mm per g, and a hard landing is 10–20 g. Keep
≥ 3 mm clear between the sensor plate and anything on the arm except the
balls; any contact bypasses the isolation.

**Retention.** Silicone balls pull out under crash loads. Add a slack tether,
braided line or a zip tie through a slot in the sensor plate and one in the
arm, with ~3 mm of slack so it never goes taut in flight.

**Cables.** The CSI ribbon and IMU wires cross the isolator. Tie them to the
sensor plate's slots and to the arm with a 15–20 mm service loop between; a
taut ribbon is a stiff spring across the balls.

**Balls to buy.** Tarot's fluid-filled gimbal balls, Shore 25A, in two sizes:
small **TL10A09** (6.5 mm holes, 70 g per ball) and medium **TL10A05/TL10A10**
(9.3 mm holes, 100–180 g per ball). A load rating is not a stiffness, so buy a
pack of each and measure before fixing the hole size in CAD: one ball between
two flat plates, a known weight on top, calipers on the height. Target: **200 g
compresses one ball ~1–1.5 mm**. If both are softer, use 6–8 balls or preload
the stack 0.5–1 mm; widening the pattern raises only the rocking modes.
**Not flight-controller grommets** (M2/M3 "anti-vibration balls" for FC stacks,
~5 mm across): they are sized for a ~10 g board, and a mode that lands at
120–170 Hz amplifies 184 Hz instead of cutting it. Press-test anything before
designing around it.

### Tuning

Frequency goes as √(stiffness / mass). Stiffer balls, more balls or preload
raise every mode; spreading the balls raises only the rocking and twist modes.

1. Static test (above) to choose the ball.
2. Tap test as on 2026-09-12 (`imu_log.py --odr 833`): flick the side, tap the
   face, tap a corner. Every mode should land between 25 and 110 Hz, with
   nothing ringing at 150–200 Hz.
3. Lowest mode under ~25 Hz: stiffer. Over ~45 Hz: softer.
4. GPS hover with `vio_live` recording: the Pi IMU under ~3 m/s² RMS side to
   side, against 43–49 today.

**Downstream.** New Kalibr camera-IMU run once assembled, then update
`R_CAM_IMU` in `mavlink_bridge.py` and its tests,
`openvins/hw_pi/kalibr_imucam_chain.yaml`, the `--expect-accel` values,
`VISO_POS_X/Y/Z` from the new geometry, and `--tilt-deg` if `TILT` changes.

---

## 9. Verification checklist

- [ ] Confirm nothing is shimmed between the arm and the middle plate (`Z_MID`)
- [ ] Print Part 2 alone, offer it up to the frame, confirm the 4 flange holes
      land on the standoffs and nothing fouls the middle plate
- [ ] Tap-test the assembled bracket once the IMU streams. The rigid build as
      first printed measured **173 Hz** (2026-09-12). With the isolator: every
      mode 25–110 Hz and nothing at 150–200 Hz (§8).
- [ ] Confirm the IMU keying lip permits only one board orientation
- [ ] With the camera streaming, check the props against the frame edges before
      committing to `CAM_Y`; adjust and reprint Part 2 only
- [ ] Kalibr extrinsics should return 90° multiples. Any residual beyond a
      degree or two — stop and find the cause
- [ ] Witness-mark every screw. Do not disassemble Part 1 afterwards.

---

## 10. Mass note — correction to the payload budget

| Item | Mass |
|---|---|
| Part 1, 38 × 54 × 4 at 100% infill | **~10.4 g** |
| Part 2 | ~7 g |
| Fasteners, standoffs | ~3 g |
| **Bracket total** | **~20 g** |

[vio-quad-build-plan.md:63](../archive/vio-quad-build-plan.md) estimates ~3 g for the
plate. That was optimistic: 4 mm at 100% infill over the area needed to host a
32 × 32 camera board — with enough edge margin that the bolt holes do not crack
out — is ~10 g, and there is no way around it without giving up
the thickness³ stiffness argument that justified 4 mm in the first place.

AUW impact is negligible — roughly 660 → 667 g, hover throttle unchanged at
~39%. But the "printed tray + mounts ~35 g" line in
[PROJECT.md:289](../development-log.md) should be revised upward.
