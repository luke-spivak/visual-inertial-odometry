# Landing Leg — CAD Specification

Mid-arm clamp landing gear for TBS Source One V5, 4× identical assemblies.
Target printer: Bambu A1 mini (180×180×180). Material: PETG.

---

## 0. How to build this

- **All dimensions are millimetres.** No drafts, no scaling.
- **Model it parametrically.** `FIT_POCKET`, `FIT_HOLE`, `GROUND_CLEAR` and
  `CLAMP_S` must be editable named parameters — all four are expected to change
  after the first physical fit test.
- **`FIT_POCKET` and `FIT_HOLE` are not yet known.** Build with placeholders of
  **+0.20 mm** for both. This is deliberate and not a blocker: the calibration
  coupon in §9 is derived FROM this model, so the model has to exist first.
  Workflow is: build → export coupon → calibrate → update the two parameters →
  export the real parts.
- **Deliverables:** STEP for the assembly, plus separate STLs or 3MFs for
  Part A, Part B, and the calibration coupon (Part A's clamp block alone, no
  strut, no foot, plus the four gauge holes described in §9).

---

## 1. Design intent

Four independent landing legs, each clamping mid-arm on the 6 mm carbon arm.
Leg hangs vertically directly below the arm — pure compression, no splay,
so all four assemblies are the same part.

Motor wires run along the TOP of the arm and are NOT rerouted. The upper
clamp half is deliberately thin and ramped so the wire bundle steps over it
with a gentle bend, and a zip-tie slot captures the bundle to the clamp.

The leg contains a deliberate sacrificial neck near its top so that a hard
landing breaks the printed leg before it cracks the carbon arm.

Two parts per assembly:
- **Part A** — leg + lower clamp half (×4)
- **Part B** — upper clamp half + wire ramp (×4)

---

## 2. Coordinate system

Per-assembly local frame:
- **+X** = **inboard along the arm axis, toward the frame centre** (i.e. toward
  the arm root / increasing `s`). The arm gets WIDER with increasing X.
- **+Y** = across the arm, horizontal
- **+Z** = up
- **Origin** = centre of the clamp (s = 45.0), on the arm's mid-thickness plane

Getting +X backwards mirrors the pocket taper and the part will not seat.

---

## 3. Parameters

### From the official CAD — `SO_V5_5inch_Arm_6mm.dxf`, tag `v5`
Extracted from `github.com/tbs-trappy/source_one`. All four arms in the nest
are the same part, so all four leg assemblies are identical.

| Name | Value | Source |
|---|---|---|
| `ARM_T` | **6.00 mm** | Filename (`..._Arm_6mm`); cross plate is 6 mm too |
| `ARM_LEN` | 98.92 mm | Motor centre hole → outer root hole |
| `CLAMP_S` | **45.0 mm** | Clamp centre, measured from the motor centre hole along the arm |
| `ARM_W` @ s=32.5 | **8.993 mm** | Outboard end of clamp pocket |
| `ARM_W` @ s=45.0 | **9.631 mm** | Clamp centre |
| `ARM_W` @ s=57.5 | **10.269 mm** | Inboard end of clamp pocket |
| `ARM_TAPER` | **0.0510 mm/mm** | +1.276 mm across the 25 mm clamp |
| `CLEAR_SPAN` | s = 30 … 86 mm | Straight arm, free of the motor pad flare and root holes |
| `MOTOR_R` | 113.0 mm | Frame centre → motor centre (226 mm motor-to-motor diagonal) |
| `CLAMP_R` | **68.0 mm** | Frame centre → clamp centre |

Arm narrows to a minimum of 8.782 mm at s≈28 (just outboard of the clamp),
then widens monotonically toward the root.

### Given / decided
| Name | Value | Source |
|---|---|---|
| `GROUND_CLEAR` | **45.0 mm** | Bottom-plate underside to ground. User spec. |
| `H_ARM_UNDER` | **+3.0 mm** | Bottom-plate underside → arm underside. User measured. |
| `BATTERY_T` | 33.0 mm | Measured |
| `CLAMP_L` | 25.0 mm | Clamp length along arm |
| `BLOCK_UNDER` | 6.0 mm | Solid material below arm pocket (houses nut pockets) |
| `FOOT_T` | 3.0 mm | Foot pad thickness |
| `NECK_Z` | 8.0 mm | Neck center, below clamp block underside |

### Still to verify with calipers
| Name | Assumed | What to measure |
|---|---|---|
| `FIT_POCKET` | TBD | Arm-pocket width offset — see §9. Critical. |
| `FIT_HOLE` | TBD | Bolt-hole diameter offset — see §9. Low stakes. |

### Measured by user
| Name | Value |
|---|---|
| `BUNDLE_W` | **7.0 mm** |
| `BUNDLE_H` | **3.0 mm** |

Confirm `ARM_T` = 6.00 and `ARM_W` = 9.63 at s=45 on the physical frame before
committing — the DXF is the published V5 5-inch arm, but frames get revised.

### Derived
```
LEG_LEN   = GROUND_CLEAR + H_ARM_UNDER        = 48.0 mm   (arm underside → ground)
STRUT_LEN = LEG_LEN - BLOCK_UNDER - FOOT_T    = 39.0 mm
POCKET_D  = ARM_T/2 - 0.25                    =  2.75 mm  (each half)
BOLT_Y    = ±9.0 mm                           (clears the arm at its widest, 10.27)
CLAMP_W   = 26.0 mm
```

**Critical:** the two clamp halves must NOT touch each other. `POCKET_D` is
half the arm thickness minus 0.25 mm per side, leaving a 0.5 mm gap at the
joint line so all clamping force lands on carbon, not on plastic.

---

## 4. Part A — Leg + lower clamp half (×4, identical)

### 4.1 Clamp block
- Rectangular block: `CLAMP_L` (X) × `CLAMP_W` (Y) × (`POCKET_D` + `BLOCK_UNDER`) (Z)
- Top face carries the arm pocket, depth `POCKET_D`, full length in X, open at
  both X ends. **The pocket is TAPERED, not parallel:**
  - width at X = −12.5 (outboard) = `8.993 + FIT_POCKET`
  - width at X = +12.5 (inboard)  = `10.269 + FIT_POCKET`
  - linear between; +X is toward the frame centre
- All outer vertical edges filleted R2

### 4.2 Bolt holes — 2× M3
- Ø `3.2 + FIT_HOLE` through, at `X = 0`, `Y = ±BOLT_Y` (±9.0)
- Hex nut pocket in the **bottom** face: 6.0 mm across flats, 2.6 mm deep,
  so nuts are captive and bolts insert downward from Part B
- Fastener: **M3 × 16 mm** socket cap, nyloc or blue threadlocker
- Nut pockets must clear the strut — they sit outboard in Y, strut is at Y=0

### 4.3 Strut
- Extends downward from the clamp block underside at `X=0, Y=0`
- Cross-section: rounded rectangle, **11.0 mm (X) × 8.0 mm (Y)** at the top,
  tapering linearly to **9.0 mm × 7.0 mm** at the foot
- Corner radii R2 on the section
- Fillet **R3** where the strut meets the clamp block underside — this is the
  peak stress location after the neck and must not be a sharp junction

### 4.4 Sacrificial neck — the fuse
- Centered at `Z = -(POCKET_D + BLOCK_UNDER + NECK_Z)`
- The strut section at this station (interpolated) is 10.59 mm × 7.80 mm.
- Over a 6.0 mm length, reduce it to **80% of that area** — scale both
  dimensions by √0.8: **9.5 mm × 7.0 mm**
- Blend in and out with **R3 fillets** — a smooth waist, not a step
- Intent: this is the weakest point in the whole load path. It must fail
  before the carbon arm does.

### 4.5 Foot
- Dome cap, not a flat disc — a flat pad catches ground edges and induces tipover
- Ø16.0 mm pad, `FOOT_T` thick, underside a spherical cap of **R20**
  (sagitta 1.67 mm, leaving a 1.33 mm rim). R12 would need a 3.06 mm sagitta and
  come to a knife edge at the rim of a 3 mm pad.
- Blend to the strut with R4

---

## 5. Part B — Upper clamp half + wire ramp (×4, identical)

### 5.1 Body
- Footprint matches Part A's clamp block: `CLAMP_L` (X) × `CLAMP_W` (Y)
- Total thickness 5.0 mm
- Underside carries the mating arm pocket, depth `POCKET_D` — **same taper as
  Part A**: `8.993 + FIT_POCKET` at X = −12.5, `10.269 + FIT_POCKET` at X = +12.5
- Sits **2.25 mm proud** of the arm's top surface — deliberately thin

### 5.2 Wire ramp — top face
The wire bundle runs along X, centered on Y=0, and must cross this part
without ever bending over a sharp edge.

- Central plateau: flat, 15.0 mm long in X, centred at X=0, flat across Y
  (9.5 mm of clear flat between the zip-tie slots — the 7.0 mm bundle fits)
- **Entry and exit ramps at 30°**, descending from the plateau to the arm's top
  surface level. The drop is 2.25 mm, so each ramp runs **3.90 mm** in X.
  Total ramped length 22.8 mm inside the 25 mm block.
- Ramp-to-plateau transitions filleted **R2 minimum**
- Every edge the bundle can contact: **R2 minimum**. Printed edges are
  serrated at the layer scale and will abrade insulation under vibration.

### 5.3 Zip-tie retention
- Two through-slots, **3.5 mm (X) × 1.5 mm (Y)**, at `X = 0`,
  `Y = ±(BUNDLE_W/2 + 2.0)` = **±5.5 mm**
- This leaves 1.05 mm of wall between each slot and its adjacent bolt hole.
  Thin but printable at 4 walls — check it survives slicing before committing.
- Slot edges on the top face chamfered 0.5 mm
- Accepts a 2.5 mm zip tie looping over the bundle and down through both slots

### 5.4 Bolt holes — 2× M3
- Ø `3.2 + FIT_HOLE` through, at `X = 0`, `Y = ±BOLT_Y` (±9.0) — must align with Part A
- Counterbore on the top face for M3 socket cap head: Ø6.2 × 2.0 mm deep,
  so heads sit flush and present nothing for the wires to catch on

---

## 6. Print settings and orientation

| | Part A | Part B |
|---|---|---|
| Orientation | **On its side** — leg silhouette flat on the bed | Pocket face **down** on the bed |
| Why | Layer planes run parallel to the strut axis, so bending loads are carried in-plane instead of peeling layers | Pocket becomes a `ARM_W`-wide bridge; PETG bridges this cleanly |
| Layer height | 0.2 mm | 0.2 mm |
| Walls | 5 | 4 |
| Infill | 25% gyroid | 30% gyroid |

- Generic PETG profile, A1 mini flow calibration on
- **All overhangs must be ≤45° in the stated print orientation** — chamfer the
  clamp pocket roof on Part A rather than adding supports
- Do NOT use 100% infill here. That spec belongs only to the camera+IMU sensor
  plate, where stiffness is the criterion. Legs want wall count, not solid fill.

---

## 7. Arm taper — resolved, and it is not optional

The V5 arm tapers at **0.0510 mm/mm**, so across a 25 mm clamp the width
changes by **1.276 mm**. A parallel-sided pocket would contact at one end only
and turn the clamp into a hinge. The pocket taper in §4.1 and §5.1 is
mandatory, not a refinement.

**Useful side effect:** a correctly tapered pocket is self-locating. Slide the
clamp on from the motor end and push it inboard; it wedges at exactly s=45 and
cannot creep outboard under load. No datum feature or measuring needed at
assembly.

## 8. Why s = 45 mm

The usable straight span is s = 30 … 86 mm, so a 25 mm clamp can be centred
anywhere in s = 42.5 … 73.5. Within that window:

- **Stance favours outboard.** At s=45 the foot sits at frame radius 68 mm.
  With CG ~52 mm above ground, the static tip angle is **42.8°**. Moving the
  clamp to the middle of the window (s=58) drops that to 36.8°.
- **Arm bending is not a constraint.** The 6 mm × ~9.6 mm carbon section has a
  bending capacity around 30 N·m; even a 20 g single-leg impact at this station
  is roughly 5 N·m. There is no reason to move inboard to protect the arm.
- **s=45 is the most outboard station that still leaves clearance** to the
  motor-pad flare, which begins at s≈24.

So: as far outboard as the geometry allows, not the centre of the arm.

---

## 9. Calibrating FIT

`FIT` is not a material constant — it depends on printer, filament, nozzle
temperature, flow, **and print orientation**. A hole printed as a circle in the
bed plane and the same hole printed as a bridged horizontal bore come out
different sizes. So the calibration coupon must be printed in the **same
orientation as the real part**, or it calibrates the wrong thing.

### FIT_POCKET — use the taper as an amplifier

Do not try to caliper a printed slot. Instead exploit the fact that a tapered
pocket on a tapered arm converts a width error into a **position** error at
19.6× magnification (1 / 0.0510 mm per mm).

1. Print **one** coupon: Part A's clamp block only — no strut, no foot — with
   the pocket taper exactly as specified and `FIT_POCKET` set to a first guess
   of **+0.20 mm**. Print it in Part A's orientation (on its side). ~15 min.
2. Slide it onto a bare arm from the motor end, pocket down, and push it inboard
   with steady moderate hand pressure until it wedges.
3. Measure from the motor centre hole to the coupon's centre. Call it `s_meas`.
4. Correct:
   ```
   delta       = 0.0510 * (s_meas - 45.0)
   FIT_POCKET  = 0.20 - delta
   ```
   Wedges past s=45 → pocket is loose → reduce. Wedges short → tight → increase.

A ruler resolves ~1 mm, which is a **0.05 mm** width resolution. That beats what
calipers give on a printed slot, from a single coupon and no ladder of guesses.

Use consistent push force between attempts; the wedge point is force-dependent.

### FIT_HOLE — go/no-go with a real bolt

On the same coupon, put four Ø3.2+offset holes at +0.0, +0.1, +0.2, +0.3.
`FIT_HOLE` is the smallest offset an M3 bolt drops through under its own weight
without being forced. Wrong by 0.1 mm here costs nothing — you can drill it out.

## 10. Verification checklist

Before printing four of anything:
- [ ] Print the calibration coupon, set `FIT_POCKET` and `FIT_HOLE` per §9
- [ ] Print ONE Part A + Part B, fit to a real arm, confirm the 0.5 mm joint gap closes onto carbon
- [ ] Confirm the wire bundle sits in the ramp with no contact pressure
- [ ] Measure prop-tip-to-arm-top vertical clearance. The prop disc inner edge
      is at frame radius 49.5 mm, so the clamp at 68 mm IS under the disc — but
      the prop sits above the motor, so clearance is the motor's stack height
      against a 2.25 mm bump. Confirm, don't assume.
- [ ] Load one leg by hand until something yields; confirm it is the neck, not the arm

Then print the remaining three.

---

## 11. Note on the motor pad

The arm's motor end has a Ø7.00 mm centre bore and four slotted M3 holes on a
Ø17.5 mm bolt circle (12.37 mm square pattern). That does not match the 16×16
pattern of an EMAX ECO II 2306, so either this DXF revision differs from the
physical frame or the slots serve another purpose. It has no bearing on the leg
design — the clamp station, arm width and taper all come from the arm outline
and root holes, which match the bottom plate's 30.5/20 stack pattern exactly.

## 12. Open item

Battery clearance at `GROUND_CLEAR = 45`:
```
45 - 3 (cradle pad) - 33 (battery) = 9 mm
```
9 mm before leg deflection. Adequate for deliberate landings, thin for
anything firm. `GROUND_CLEAR` is a single parameter — raising it to 50 mm
costs ~1.5 g per leg and changes nothing else in this spec.
