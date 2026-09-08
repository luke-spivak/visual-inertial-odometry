# TF-Luna Rangefinder Mount — CAD Specification

Downward-looking mount for a Benewake TF-Luna on the tail boom of a
TBS Source One V5. One printed part. Printer: Bambu A1 mini. Material: PETG.

**Supersedes** [battery-deck-spec.md](battery-deck-spec.md) §4.6, which folded the
TF-Luna into the battery deck at Y ≈ +34. That position is no longer available —
see §2.1.

---

## 0. How to build this

- **All dimensions are millimetres.**
- **Model it parametrically.** `LUNA_L`, `LUNA_W`, `LUNA_H`, `LUNA_Y`,
  `SHELF_T`, `FIT_HOLE` must be editable named parameters.
- `FIT_HOLE` = **+0.20 mm** placeholder, carried from the landing-leg coupon.
- **Read §2 before drawing anything.** Both the frame geometry and the sensor's
  own mounting convention are the opposite of what you would assume.
- **Deliverables:** STEP plus STL/3MF. Single part, prints with no supports.

---

## 1. Design intent

A shelf hung under the tail boom, aft of the battery deck, holding the TF-Luna
lens-face-down with a clear view of the ground and nothing below it that touches
down first.

The rangefinder is not an altitude source for VIO — [PROJECT.md:823](PROJECT.md:823)
already answers that, and the error it would need to fix is in an axis it does
not measure. Its jobs are **health check** (vision says 3 m, ground says 10 m),
**landing detection**, and **metric-scale anchoring**
([PROJECT.md:242](PROJECT.md:242)). All three want it looking at undisturbed
ground, which is what fixes its position.

---

## 2. Constraints that fix the position

### 2.1 The bottom plate's Y axis was mirrored in the battery-deck spec

`SO_V5_BottomPlate_2mm.dxf` and `SO_V5_TopPlate_2mm.dxf` carry an **identical**
pair of 2.00 × 10.00 strap slots at X ±10.60, Y −62.88 … −52.88, and identical
Ø3.0 pairs at (±10.00, −42.88) and (±11.00, −72.88). Identical coordinates on
both plates means both are drawn in the same orientation, and the middle plate's
camera-cage tab slots at (±5.50, +77.54) fix **+Y = nose**.

So the true bottom-plate extent is:

```
X ±21.25,  Y −77.45 … +21.25          <-- the plate runs AFT, not forward
```

[battery-deck-spec.md:44](battery-deck-spec.md:44) has it as `Y −21.2 … +77.4`,
and every bottom-plate Y in that spec's §2 table is sign-flipped. The
consequences are in §9.

**The corrected tail boom** — constant width, and the reason this part is easy:

```
Y = −45 … −70     X = ±14.00
Y = −72.88        X = ±15.00   (the bolt boss)
Y = −74 / −75 / −76   ±14.84 / ±14.39 / ±13.51
Y = −77.45        tip
```

Both tail Ø3.0 pairs appear at the same coordinates in the **top** plate too, so
each is the bottom of a standoff column spanning the full tail height. This part
bolts to a column, and the GPS mast bolts to the top of the same column
([gps-mast-spec.md](gps-mast-spec.md) §2.2). **Use separate screws entering the
standoff from each end** and check they do not meet in the middle.

### 2.2 The forward positions are all disqualified

| Candidate | Why not |
|---|---|
| Under the deck floor, Y ≈ +30 | Deck floor bottom is at Z −39 and the ground at Z −45; a 13.5 mm sensor there hits the ground first |
| Under the nose, Y ≈ +50 | **In the camera's field of view.** The camera sits at (0, +120, +12) looking 30° down through ±59° horizontal. A static object in frame is a permanent false feature and corrupts tracking |
| Beside the deck | Deck flanges reach X ±23.0 on a plate that is ±14.00 wide there |

That leaves **aft of the deck's rear face (Y −58.5) and forward of the plate tip
(Y −77.45)** — and the sensor may overhang aft freely, because nothing is there.

### 2.3 Props are not a constraint, and it is worth saying why

With the corrected motor positions **(±86.563, ±72.635)**, radius 113.00, azimuth
40.00° (see [gps-mast-spec.md](gps-mast-spec.md) §2.1 — the earlier specs' ±45°
was wrong), the §5 shelf clears the rear prop discs by **+1.4 mm** in plan view.

That margin is irrelevant here and would be alarming anywhere else. The shelf
hangs at Z −18.5 and the prop plane is ~50 mm above it. Plan-view clearance only
constrains parts at prop height — which is the Pi tray's problem, not this one.

---

## 3. The sensor

From the Benewake mechanical drawing, `SJ-GU-TF-Luna A03`:

| | |
|---|---|
| Body | **35.0 (L) × 21.25 (W) × 13.5 (H)** — H measured from the mounting face |
| Housing (raised) | 26.65 long; ears stand proud of it to the full 35.0 |
| **Mounting** | **2 × Ø2.2 on 30.5 mm centres**, ear radius R2.25 |
| **Ears** | **2.0 mm thick, coplanar with the optical face** |
| Optics | 2 × Ø10.5 apertures, centres **10.5 apart** (tangent), symmetric about the module centre |
| Optical/hole axis | on a line **9.0 mm from one 35 mm edge, 12.25 from the other** |
| Connector | 10.5 wide, top at **7.0** from the mounting face, on the 12.25 side |
| FOV / range / mass | 2° / 0.2–8 m / <5 g |

**The ears are on the optical face.** This is the single fact that shapes the
part: the module cannot bolt flat to the underside of anything, because its
lenses would be looking into it. It must sit **on top of a shelf, body upward,
looking down through an aperture** — which is why this mount is 18.5 mm deep for
a 5 g sensor.

Datasheet nuance: the specification table says 12.5 mm high, the mechanical
drawing says 13.5. **Build to 13.5** and the 2 mm of headroom in §4 absorbs it.

---

## 4. Parameters

| Name | Value | Source |
|---|---|---|
| `LUNA_L` | 35.0 mm | Along **X** — see §5.1 |
| `LUNA_W` | 21.25 mm | Along Y |
| `LUNA_H` | 13.5 mm | Drawing, not the spec table |
| `LUNA_Y` | **−80.0 mm** | Optical/hole axis. §5.1 |
| `HOLE_PITCH` | 30.5 mm | |
| `SHELF_T` | 3.0 mm | |
| `HEADROOM` | 2.0 mm | Module top to plate underside |
| `FIT_HOLE` | +0.20 mm | Placeholder |

### Derived
```
shelf top face        Z = −(LUNA_H + HEADROOM) = −15.5
shelf bottom face     Z = −18.5
leg height            15.5
module footprint      X ±17.50,  Y −89.00 … −67.75
ear holes             (±15.25, −80.00)
lens apertures        Ø10.5 at (±5.25, −80.00)
shelf extent          X ±22.00,  Y −66.00 … −91.00
ground clearance under the shelf   45.0 − 18.5 = 26.5 mm
clearance above the pack's underside (Z −36)   17.5 mm
```

**Why `LUNA_Y` = −80.0 and not −72.88 (on the bolts).** With the lens axis on the
bolt line the Ø3.2 bolt holes and the 21.6 mm aperture want the same material.
Moving the sensor 7.1 mm aft puts the aperture's forward edge at Y −74.0, just
clear of the bolts at −72.88, and costs a 12 mm cantilever carrying 5 g.

---

## 5. Geometry

### 5.1 Orientation — 35 mm axis across the frame

The module's long axis runs in **X**. Rotating it 90° would put its forward end
at Y −55.4, inside the battery deck's rear stop at −58.5. Across the frame it
overhangs a 28 mm boom by 3.5 mm per side, which costs nothing.

Consequence: the 12.25 mm side carries the connector, so with the optical axis at
`LUNA_Y` the module spans **Y −89.00 (the 9.0 edge) to −67.75 (the connector
edge)** — the **cable leaves forward, toward the aircraft**. That is the right
way round and is not an accident of the layout; build it this way.

### 5.2 Shelf
- Flat plate **X ±22.00, Y −66.00 … −91.00**, thickness `SHELF_T` (3.0),
  top face at Z −15.5.
- **Aperture**: obround **21.6 (X) × 12.0 (Y)**, R6.0 ends, centred **(0, −80.0)**.
  This gives 0.3 mm of relief around the Ø10.5 lens pair in X and 0.75 in Y.
  Do not shrink it — the two apertures are tangent, so 21.0 is the hard minimum.
- **Sensor screws**: Ø `2.4 + FIT_HOLE` at **(±15.25, −80.0)**, counterbored
  **Ø4.4 × 2.0 deep from below**. Fasteners are **2 × M2 × 6 socket cap**, head
  under the shelf, thread up into the ear. Nothing on the shelf's top face
  interferes, because the ribs and legs are all outboard of X ±18.0.
- Outer edges R2, corners R4.
- **Do not lighten the shelf.** It is 3 mm thick, it is the part's whole bending
  section, and it is 3.3 g.

### 5.3 Legs — two, and they close at 45°
Each leg is a wall running in Y from **−68.00 to −79.00** (11 mm), rising the
full 15.5 mm from the shelf's top face to the plate:

| | at the shelf (Z −15.5) | at the plate (Z 0) |
|---|---|---|
| inner face | X = ±18.00 | X = ±8.00 |
| outer face | X = ±21.00 | X = ±14.00 |

Both faces lean **inward** with height: the outer face is self-supporting, and the
inner face overhangs at **32.8° from vertical**, inside the 45° rule. **There is
no bridge anywhere in this part** — that is why the two legs are separate rather
than a single closed box, and why the region between them (X ±8.00) is simply
open. The module's body occupies that opening.

The leg's bottom inner face at ±18.00 clears the module's ears (±17.50) by
0.5 mm.

### 5.4 Bolt pads
- Each leg's top is a pad spanning **X ±7.00 … ±14.00, Y −68.00 … −79.00**, top
  face at **Z 0**, bearing on the bottom plate's underside.
- Hole Ø `3.2 + FIT_HOLE` at **(±11.00, −72.88)**.
- Local boss **Ø8.0** about each hole, reaching X ±15.00 — exactly the plate's own
  boss width at that station (§2.1), so it bears and does not overhang.
- Replace the existing tail screws with ones **`SHELF_T` + 1 mm longer**, and see
  §2.1 on sharing the standoff with the GPS mast.

### 5.5 Aft ribs
The shelf cantilevers 12 mm aft of the legs. Two ribs, **3.0 thick, 8.0 tall**,
at **X ±19.00 … ±22.00**, running **Y −79.00 … −90.00**, ramped at 45° where they
meet the leg. They sit outboard of the module (±17.50) and print as plain
vertical walls.

### 5.6 Cable
The connector faces +Y at (0, −67.75), 7.0 above the shelf → **Z ≈ −8.5**. It
exits forward through the open gap between the legs, then up to the bottom
plate's underside.

- **Zip-tie slot pair**, 3.5 × 1.5, at **(±6.00, −68.50)** through the shelf.
- Tie the lead here, not at the connector. The TF-Luna's 1.25 mm header is the
  weakest thing in the assembly.

At Z −8.5 the lead passes **well above** the battery deck's rear stop, which
occupies Z −39 … −18 at Y −58.5. No clash.

---

## 6. Sight line

Checked against everything below the aircraft:

| Obstruction | Result |
|---|---|
| Battery pack (rear face Y −55.5, underside Z −36) | Nearest corner is **30.5° off the beam axis**; the beam half-angle is 1° |
| Deck rear stop (Y −58.5) | 21.5 mm forward of the aperture, above the beam |
| Landing legs at (±48.1, ±48.1) | Nearest is 51.7 mm away; the beam is 35 mm in radius at 1 m |
| Camera FOV | Sensor is 187 mm **behind** the camera. Not in frame |

**The shelf is not the lowest point on the aircraft.** Its underside is at
Z −18.5; the battery pack's underside is at Z −36 and the ground at Z −45. On a
hard landing the pack and the legs take it, in that order, and the sensor is
17.5 mm clear of the first of them.

---

## 7. Print settings

| | Value |
|---|---|
| Orientation | **Shelf flat on the bed, legs and ribs upward** |
| Supports | **None** — §5.3 |
| Layer height | 0.20 mm |
| Wall loops | 4 |
| Top / bottom shells | 5 / 5 |
| Sparse infill | 40% gyroid |
| Fan max / min | 30% / 20% |
| Nozzle | 250 °C |
| X-Y hole compensation | **0** — `FIT_HOLE` is in the geometry |
| Elephant foot compensation | 0.15 mm |
| Brim | none |

The shelf's bottom face is the bed face and also the face nearest the ground; a
textured-PEI finish on it is fine and slightly abrasion-resistant.

---

## 8. Wiring and parameters

TF-Luna connector, 6 pins. **Wire colours on the supplied loom are arbitrary —
go by pin number.**

| Pin | I2C function |
|---|---|
| 1 | +5 V |
| 2 | **SDA** |
| 3 | **SCL** |
| 4 | GND |
| 5 | **CFG — tie to GND** |
| 6 | not used |

**The module ships in UART mode.** Pins 4 and 5 must be joined for I2C; the
aircraft has no spare UART for it ([PROJECT.md:405](PROJECT.md:405)), which is why
this matters. Make the pin-4/pin-5 join at the FC end, not at the sensor, so it
survives a mount change.

```
RNGFND1_TYPE   = 25      # Benewake TFmini-Plus I2C — the TF-Luna's driver
RNGFND1_ADDR   = 16      # 0x10
RNGFND1_MIN_CM = 20
RNGFND1_MAX_CM = 800     # the 8 m ceiling in PROJECT.md:835
RNGFND1_POS_X  = -0.080  # body FRD, metres, from the FC — measure and confirm
RNGFND1_POS_Y  =  0.000
RNGFND1_POS_Z  =  0.027
```

Bus note: the compass shares this I2C bus ([PROJECT.md:403](PROJECT.md:403)).
The QMC5883L answers at 0x0D and the TF-Luna at 0x10, so there is no conflict —
but a 1.4 m of unshielded I2C running the length of the aircraft is a real risk.
Keep the run short, twist SDA/SCL with GND, and check for compass dropouts after
fitting this part, not before.

---

## 9. What this changes elsewhere

### 9.1 The battery deck's frame table is mirrored
Every bottom-plate Y in [battery-deck-spec.md](battery-deck-spec.md) §2 needs its
sign flipped:

| Stated | Correct |
|---|---|
| Plate outline `Y −21.2 … +77.4` | **`Y −77.45 … +21.25`** |
| Forward pair `(±10.00, +42.88)` | **`(±10.00, −42.88)`** |
| Nose pair `(±11.00, +72.88)` | **`(±11.00, −72.88)`** — it is the **tail** boom |
| Strap slots `3 × 10 at (±10.6, +57.88)` | **`2.00 × 10.00 at (±10.60, −57.88)`** |
| "there is no structure aft of Y = −21.2" | there is none **forward of Y = +21.25** |

**This is good news for that part.** The pack at `PACK_Y` −20 spans Y −55.5 …
+15.5, which under the corrected geometry is **entirely over the bottom plate**,
with hardpoints at (±10.00, −42.88) directly beneath it and strap slots just aft
of its rear face. The §4.4a forward cradle stations were added to work around a
plate edge that is not there. The 48.5 mm aft cantilever in §7 is not there
either.

### 9.2 `PACK_Y` moves forward
Re-running §8's moment balance with this part at Y −78 (11 g) and the GPS mast at
Y −73 (30 g) instead of the assumed +34 and −50:

```
nose-heavy   camera 1800 + IMU 360 + bracket 1227 + 595 + 330   = +4312
tail-heavy   GPS mast −2190,  TF-Luna −858,  Pi tray −55        = −3103
net                                                              = +1209 g·mm
PACK_Y = −1209 / 180  =  −6.7 mm      (was −20.0)
```

That is a **13 mm forward move of the rear-stop face**, from Y −55.5 to −42.3.
Still an estimate stacked on estimates — the two-finger balance check before
first flight is what settles it, and the rear stop is the one dimension that
sets it.

---

## 10. Verification checklist

- [ ] Both pads seat flat on the boom with no rock; neither boss overhangs the
      plate edge
- [ ] Module drops between the legs and both ears land on the shelf, not on a leg
- [ ] Sight straight up through the aperture with the module out — nothing but sky
- [ ] With the pack fitted, confirm 17.5 mm from the shelf's underside to the
      pack's underside
- [ ] Pins 4 and 5 joined **before** first power-up, and the sensor answers at
      0x10 on an I2C scan
- [ ] `RNGFND1_POS_*` measured from the FC, not copied from §8
- [ ] Compass still reads cleanly with the TF-Luna on the bus
- [ ] Static test: aircraft on a bench at a known height, `RFND` log message
      within ±6 cm

---

## 11. Mass

| Item | Mass |
|---|---|
| Shelf | 3.3 g |
| Legs and pads | 1.9 g |
| Aft ribs | 0.7 g |
| **Printed total** | **~6 g** |
| TF-Luna | 5 g |
| Fasteners | ~1 g |
| **All-up at Y ≈ −78** | **~12 g** |

Running total for printed parts across all six specs:

| Part | Mass |
|---|---|
| Landing legs ×4 | ~12 g |
| Camera + IMU bracket | ~20 g |
| Battery deck | ~24 g |
| Pi tray | ~11 g |
| GPS mast | ~17.5 g |
| **TF-Luna mount** | **~7 g** |
| **Total** | **~92 g** |

against the **~35 g** at [PROJECT.md:341](PROJECT.md:341). The battery-deck spec's
running total of ~71 g predates this part and assumed a 4 g GPS mast; §11 of
[gps-mast-spec.md](gps-mast-spec.md) explains why that one is 17.5 g.

AUW goes ~660 → ~717 g and hover throttle 39 % → **40.4 %** — still inside the
40–50 % target band. If §9.1 lets the deck's forward cradle go, ~6 g comes back.
