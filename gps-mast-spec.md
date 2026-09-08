# GPS / Compass Mast — CAD Specification

Tail-mounted mast for the SEQURE M10-25Q GPS + QMC5883L compass on a
TBS Source One V5. One printed part. Printer: Bambu A1 mini. Material: PETG.

---

## 0. How to build this

- **All dimensions are millimetres.**
- **Model it parametrically.** `MAST_H`, `WALL_T`, `GPS_L`, `GPS_H`, `FIT_POCKET`
  and `FIT_HOLE` must be editable named parameters.
- `FIT_HOLE` = **+0.20 mm**, `FIT_POCKET` = **+0.20 mm** — placeholders carried
  from the landing-leg calibration coupon.
- **Read §2 before drawing anything.** The frame's motor positions used in the
  earlier specs were wrong; this spec uses the corrected ones.
- **Deliverables:** STEP plus STL/3MF. Single part, prints with no supports.

---

## 1. Design intent

A short vertical mast bolted to the two existing tail hardpoints on the top
plate, carrying the GPS/compass puck aft of and above everything else on the
aircraft.

**The compass is the requirement, not the GPS** ([PROJECT.md:142](PROJECT.md:142)).
The SpeedyBee F405 V4 has no onboard magnetometer, and without a compass there is
no yaw source, which locks the aircraft out of Loiter, PosHold and every auto
mode — i.e. out of the ability to bench the thing being built. So this part exists
to put a magnetometer as far from the high-current DC path as the airframe allows,
and everything else about it is secondary.

---

## 2. Frame geometry — corrected

Re-extracted from `github.com/tbs-trappy/source_one`, tag `v5`. Frame
coordinates: origin at the FC stack centre, **+Y = nose**, **+Z = up**,
Z datum = bottom-plate underside.

### 2.1 The motor positions in the earlier specs were wrong

`SO_V5_5inch_Arm_6mm.dxf` draws the four arms about a shared root pattern that is
the frame origin. The four Ø7.0 motor bores sit at:

```
(±86.563, ±72.635)        radius 113.00 mm, azimuth 40.00° from +X
track  173.13 mm across (X)  ×  145.27 mm fore-aft (Y)
```

**This is a wide-X frame, not a true X.** The camera bracket spec
([camera-imu-bracket-spec.md:62](camera-imu-bracket-spec.md:62)) and the Pi tray
spec both assumed `radius 113.0 at ±45° → (±79.9, ±79.9)`. The radius was right;
the angle was not.

The check that settles it: with motors at (±72.635, ±86.563) — the 90°-rotated
reading — the prop discs would cut **4.4 mm into the top and bottom plates**,
which is impossible. With (±86.563, ±72.635) every plate clears every disc by
**8.1–9.7 mm**, which is what a designed clearance looks like.

`MOTOR_R` = 113.0 and `CLAMP_R` = 68.0 in the landing-leg spec are radial and
survive unchanged; that spec needs no edit.

### 2.2 Top-plate hardpoints

| Feature | Position | Notes |
|---|---|---|
| Outline | X ±18.00, Y −77.29 … +81.38 | |
| **Tail pair** | **(±11.00, −72.88)** | Ø3.0 — **this part's anchor** |
| Aft pair | (±10.00, −42.88) | Ø3.0 — taken by the Pi tray |
| Forward pair | (±9.50, +32.88) | Ø3.0 — Pi tray / camera bracket |
| Nose pair | (±9.50, +76.88) | Ø3.0 — camera bracket |
| **Aft strap slots** | **2.00 × 10.00 at X ±10.60, Y −62.88 … −52.88** | **this part's tie-down** |
| Aft cable slot | 8.97 × 6.12 at X ±4.48, Y −52.00 … −45.88 | keep clear |

Both tail pairs appear at **identical coordinates in the bottom plate**, so each
is the top of a standoff column running the full height of the tail. The mast
therefore bolts to a column, not to unsupported plate — which is why a 2 mm
carbon plate can carry a 30 g mast at all.

### 2.3 Tail-boom footprint

The top plate's tail is a constant-width strip:

```
Y = −45 … −70     X = ±14.00
Y = −72.88        X = ±15.00   (the bolt boss)
Y = −74 / −75 / −76   ±14.84 / ±14.39 / ±13.51
Y = −77.29        tip
```

The foot in §5.1 is drawn to this profile.

---

## 3. The module

SEQURE M10-25Q — u-blox M10, QMC5883L, **25 × 25 × 8 mm, 12.2 g**, SH1.0-6P
connector ([PROJECT.md:49](PROJECT.md:49) and the vendor spec).

**Two assumptions to check against the part in hand before printing:**

1. **The case has no mounting holes.** These pucks are normally taped or strapped
   down. The head in §5.3 retains the module with two zip ties and does not
   depend on holes; if yours has ears, ignore them.
2. **Which edge the connector leaves from is unknown.** §5.3 gives the module a
   1.5 mm cable plenum under its whole footprint so the lead can leave any edge
   and still reach the mast cavity. If you can see the connector, still build the
   plenum — it costs 0.3 g.

**`COMPASS_ORIENT` depends on this part being built level.** The head pad is
horizontal and the module's arrow points **+Y**; that is `ROTATION_NONE` for an
external compass. Do not rake the mast — a tilted compass needs a mounting
rotation ArduPilot can only express in 45° steps.

---

## 4. Parameters

| Name | Value | Source |
|---|---|---|
| `MAST_H` | **50.0 mm** | Foot top face → module seat. §7, §8 |
| `WALL_T` | **2.5 mm** | Column wall. §7 |
| `COL_X` × `COL_Y` | **16.0 × 14.0** | Column outer section. §7 |
| `GPS_L` | 25.0 mm | Module, square in plan |
| `GPS_H` | 8.0 mm | Module thickness |
| `FOOT_T` | 3.0 mm | |
| `HEAD_T` | 2.0 mm | Head floor and walls |
| `FIT_POCKET` | +0.20 mm | Placeholder |
| `FIT_HOLE` | +0.20 mm | Placeholder |

### Derived
```
mast axis                X = 0,  Y = −72.88      (on the bolt line: zero cantilever)
column cavity            11.0 (X) × 9.0 (Y)
head outer               29.4 × 29.4,  10.4 tall  (2.0 floor + 8.4 pocket)
head spans               X ±14.70,  Y −87.58 … −58.18
module seat height       FOOT_T + MAST_H = 53.0 above the top plate's top face
foot spans               X ±14.00 (±15.00 at the boss),  Y −52.00 … −76.00
```

---

## 5. Geometry

### 5.1 Foot
- Outline: **X ±14.00 from Y −52.00 to Y −70.00**, then blending to **R15.00
  about (0, −72.88)**, truncated at **Y = −76.00**. This is the plate's own
  profile (§2.3) — the foot must not overhang it anywhere.
- Thickness **`FOOT_T` (3.0)**. Bottom face is a plain bearing face; keep it flat
  and unlightened.
- **Bolt holes** Ø `3.2 + FIT_HOLE` at **(±11.00, −72.88)**.
  Replace the existing top-plate screws at this station with ones
  **`FOOT_T` + 1 mm longer**.
- **Central relief**: **X ±7.00, from the forward edge (Y −52.00) aft to
  Y = −64.00**, full depth. This is not lightening — it is the clear path for the
  Pi tray's shutdown button (§9) and for the GPS lead.
- All edges R1.5; the underside edge that meets the plate gets a 0.5 × 45°
  chamfer, not a fillet, so it seats flat.

### 5.2 Tie-down — the reason this part does not hinge

Two bolts on one Y line is a hinge: nothing stops the foot's forward end lifting.
The fix uses a slot the frame already has.

- On each rail, a locating channel **3.5 (Y) × 1.0 (Z) deep** across the top face
  at **Y = −57.88**, centred on **X = ±10.60**.
- At assembly a **2.5 mm zip tie** passes up through the aft end of the top
  plate's strap slot, across the channel, and back down through the forward end
  of the same slot. One tie per side.

The slot is 10.0 mm long in Y and the tie loop only has to clamp 3 mm of printed
rail, so this is a short, tight loop pulling straight down — not a strap. Two of
them convert the bolt line from a hinge into a fixed joint, and they are
serviceable with cutters.

### 5.3 Column
- Hollow rectangle, outer **16.0 (X) × 14.0 (Y)**, wall **`WALL_T` (2.5)**,
  cavity 11.0 × 9.0, centred on **(0, −72.88)**.
- Rises **`MAST_H` (50.0)** from the foot's top face.
- **Root fillet R4.0** all round where it meets the foot. This is the highest
  stressed feature on the part; do not reduce it.
- The cavity is open top to bottom — it is the cable conduit (§6).

### 5.4 Head
- **Flare**: from the 16.0 × 14.0 column to the 29.4 × 29.4 head, **chamfered at
  45°** over 6.7 mm of height. A 45° flare prints unsupported; a square shoulder
  does not.
- **Floor** `HEAD_T` (2.0) thick, top face at `FOOT_T` + `MAST_H` = 53.0 above
  the plate.
- **Pocket** `25.0 + FIT_POCKET` square × **8.4 deep**, walls `HEAD_T` (2.0).
- **Cable plenum**: two rails **3.0 (X) wide × 1.5 tall** running in Y at
  **X = ±9.0**, on the pocket floor. The module sits on the rails; the 1.5 mm
  gap underneath lets the lead run from any edge to the floor slot.
- **Floor slot** 8.0 (X) × 6.0 (Y) centred on (0, −72.88), opening into the
  column cavity.
- **Forward wall lowered**: the +Y wall is **4.4 mm tall** instead of 8.4, full
  width — a cable exit that also stops the puck being trapped.
- **Zip-tie notches**: 4 × U-notch **3.5 wide × 2.5 deep** in the **top edges of
  the ±X walls**, at Y = −72.88 ± 8.0. Two ties cross the module in X.
- **Arrow**: emboss a 0.6 mm-proud arrow on the +Y outer face of the head
  pointing forward, and the text `FWD`. Build orientation is a flight-safety
  item here, not a nicety.

---

## 6. Cable

SH1.0-6P, 6 conductors. Route: module → plenum → head floor slot → column cavity
→ out through the foot's central relief → forward along the top plate → down
through the **aft cable slot at (±4.48, −52.00 … −45.88)** → FC.

The cable inside the column also **tethers the module**: if the mast ever
sacrifices itself, the puck stays with the aircraft. Leave 15 mm of slack in the
cavity and zip-tie the lead to the foot at the relief's forward edge — the tie is
the strain relief, not the connector.

---

## 7. Stiffness — why 50 mm and this section

A 12.2 g puck on the end of a printed cantilever is a resonator. First mode,
`k = 3EI/L³`, `f = (1/2π)√(k/m)`, PETG E = 1800 MPa, head + fasteners 4.5 g,
0.24 × mast mass added:

| `MAST_H` | k (N/mm) | mast mass | **f₁** |
|---|---|---|---|
| 30 | 598 | 4.8 g | **921 Hz** |
| 40 | 252 | 6.4 g | **592 Hz** |
| **50** | **129** | **7.9 g** | **419 Hz** |
| 60 | 74.8 | 9.5 g | 316 Hz |
| 70 | 47.1 | 11.1 g | 248 Hz |
| 80 | 31.5 | 12.7 g | 201 Hz |

Motor fundamental is ~160 Hz at the ~39 % hover throttle
([PROJECT.md:333](PROJECT.md:333)) and reaches ~420 Hz at full throttle on 4S at
1700 KV. **50 mm keeps the mast at 2.6× the hover fundamental**, which is the
condition it spends its life in. It is only coincident with the motor order at
full throttle, transiently.

Above ~70 mm the mast sits inside the hover band and will be excited
continuously. Do not extend it.

Bending stress is not the issue and does not drive the print orientation: at 5 g
of tail vibration the root sees ~0.07 MPa, three orders below PETG's interlayer
strength. That is why §9 can print this upright.

---

## 8. Why 50 mm and not 80 — and why the tail, not the nose

### Height buys less than the tail offset already did

Treating the battery/ESC DC loop as a dipole at (0, 0, 4) and the puck at
(0, −73, Z), field falls as 1/r³:

| `MAST_H` | head Z | r | field vs. flat on the plate |
|---|---|---|---|
| 0 | 48 | 85.2 | 1.00 × |
| 20 | 68 | 97.1 | 0.68 × |
| **50** | **98** | **119.0** | **0.37 ×** |
| 70 | 118 | 135.4 | 0.25 × |

(assumes 30 mm middle-to-top standoffs; the ratios barely move if yours differ)

**73 mm of tail offset is already doing most of the work.** Going from 50 to
70 mm buys a further 1.5× reduction and costs the resonance margin in §7. 50 mm
is where those two curves cross.

### Prop clearance is not a constraint here — unlike the Pi

With the corrected motor positions, the 29.4 mm head at (0, −72.88) clears the
rear prop discs by **+8.56 mm** in plan view. The mast column clears by more.

This is the difference between this part and the Pi tray: the Pi sits in the
gap between four discs and clears by 2.36 mm, so its height matters. The mast
sits **outside** the discs, so it can be any height. No `Z_HEAD` gate applies.

### Sky view

With the puck 53 mm above the plate and 30.5 mm aft of the Pi's board edge, the
top of an Active Cooler subtends **atan(19.6/30.5) ≈ 33°** at the antenna if the
mast were flat, and **below the horizon** at `MAST_H` = 50. The carbon top plate
under the puck acts as a ground plane, which a patch antenna wants.

---

## 9. Print settings

| | Value |
|---|---|
| Orientation | **Foot flat on the bed, column vertical, pocket opening up** |
| Supports | **None** — the only overhang is the 45° head flare (§5.4) |
| Layer height | 0.20 mm |
| Wall loops | **6** — at `WALL_T` 2.5 with a 0.4 nozzle the column is all perimeter, which is the point |
| Top / bottom shells | 5 / 5 |
| Sparse infill | 25% gyroid (only the foot and head floor have any) |
| Fan max / min | 30% / 20% |
| Nozzle | 250 °C |
| X-Y hole compensation | **0** — `FIT_HOLE` / `FIT_POCKET` are in the geometry |
| Elephant foot compensation | 0.15 mm |
| Brim | 5 mm — the footprint is small relative to the height |

**Clash to check on the printed part, not in CAD:** the Pi tray's shutdown button
sits at (0, −45) facing aft ([pi-tray-spec.md](pi-tray-spec.md) §6.1) and the
mast foot's forward edge is at Y −52.00. The §5.1 central relief (X ±7.00) is
what keeps the button reachable. Confirm with a finger before committing to the
button position.

---

## 10. Verification checklist

- [ ] Foot seats flat on the tail boom with no rock and no overhang past the
      plate edge at any station in §2.3
- [ ] Both zip ties pass through the strap slots and pull the forward end down
- [ ] Module drops into the pocket, sits on the plenum rails, and the lead
      reaches the floor slot from whichever edge it actually leaves
- [ ] Puck arrow and the moulded `FWD` arrow both point at the nose
- [ ] `COMPASS_ORIENT` = 0 gives sane headings on the bench, rotated through all
      four cardinal directions
- [ ] **CompassMot with props on and the aircraft restrained** — this is the
      measurement that says whether 50 mm was enough. If throttle-correlated
      interference is above ~30%, the answer is a longer mast and a re-run of §7,
      not a software fix
- [ ] Shutdown button still pressable with the mast fitted

---

## 11. Mass and CG

| Item | Mass |
|---|---|
| Column | 7.9 g |
| Head | 4.8 g |
| Foot | 1.9 g |
| Fillets and misc | 1.0 g |
| **Printed total** | **~16 g** |
| Fasteners and ties | ~1.5 g |
| GPS module | 12.2 g |
| **All-up at Y ≈ −73** | **~30 g** |

**This changes `PACK_Y`.** The battery deck's moment balance
([battery-deck-spec.md](battery-deck-spec.md) §8) assumed *15 g at −50*. This is
30 g at −73, and the TF-Luna moved from +34 to −78. Re-running the balance gives
**`PACK_Y` ≈ −7 mm, not −20 mm** — a 13 mm forward move of the rear-stop face.
See the TF-Luna spec §9 for the full table. It remains an estimate stacked on
estimates; the two-finger balance check before first flight is what settles it.
