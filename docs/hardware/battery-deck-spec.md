# Battery Deck — CAD Specification

Bottom-mount battery cradle for TBS Source One V5, 4S 1500 pack.
One printed part. Printer: Bambu A1 mini. Material: PETG.

---

## 0. How to build this

- **All dimensions are millimetres.**
- **Model it parametrically.** `PACK_L`, `PACK_W`, `PACK_H`, `PACK_Y`,
  `WALL_H`, `FIT_HOLE` must be editable named parameters.
- **Pack cross-section is square, 33.0 × 33.0 — confirmed.** Every dimension in
  this spec is already built on it. One consequence: the deck cannot tell which
  way up the pack goes, so nothing binds either way — route the balance lead
  deliberately rather than relying on the cradle to set orientation.
- `FIT_HOLE` = **+0.20 mm** placeholder, carried from the landing-leg
  calibration coupon.
- **Deliverables:** STEP plus STL/3MF. Single part, no assembly.

---

## 1. Design intent

A downward-opening channel bolted under the bottom plate. The floor takes the
pack's weight, the side walls provide bending stiffness and locate the pack
laterally, and a rear stop fixes the fore-aft position so the CG is repeatable
across battery swaps.

It also absorbs two jobs that would otherwise need their own brackets: the
TF-Luna mount and a sacrificial ground-contact skid.

**Why a channel and not a flat plate** — §7. Short version: a flat 3 mm plate
puts a 180 g pack on a 26 Hz spring.

---

## 2. Frame hardpoints

Extracted from `SO_V5_BottomPlate_2mm.dxf`, tag `v5`, at
`github.com/tbs-trappy/source_one`. Frame coordinates as in the bracket spec:
origin at the FC stack centre, **+Y = nose**, **+Z = up**, Z datum =
bottom-plate underside.

| Feature | Position | Notes |
|---|---|---|
| Plate outline | X ±21.2, **Y −21.2 … +77.4** | The plate ends 21 mm behind centre |
| Stack pattern (outer) | (±15.25, ±15.25) | Ø3.0 — carries the FC/ESC preload, **do not use** |
| Stack pattern (inner) | (±10.00, ±10.00) | Ø3.0 — fallback anchor, see §5 |
| **Centre press nut** | **Ø7.0 at (0, 0)** | Preferred anchor, see §5 |
| **Forward pair** | **(±10.00, +42.88)** | Ø3.0 — second anchor station |
| Forward pair (nose) | (±11.00, +72.88) | Out on the nose boom, unused here |
| Existing strap slots | 3 × 10 at (±10.6, +57.88) | Nose boom, unusable for a centred pack |

The critical fact: **there is no structure aft of Y = −21.2.** A pack positioned
for CG hangs ~37 mm beyond the plate, which is the entire reason this part exists.

---

## 3. Parameters

| Name | Value | Source |
|---|---|---|
| `PACK_H` | **33.0 mm** | Measured — square section with `PACK_W` |
| `PACK_L` | **71.0 mm** | Measured |
| `PACK_W` | **33.0 mm** | Measured |
| `PACK_M` | 180 g | [PROJECT.md:291](../development-log.md) |
| `PACK_Y` | **−20.0 mm** | Pack centre. Derived in §8 |
| `FLOOR_T` | **3.0 mm** | Set by the clearance budget below |
| `WALL_T` | 3.0 mm | |
| `WALL_H` | **18.0 mm** | Set by the stiffness target, §7 |
| `GROUND_CLEAR` | 45.0 mm | From the landing-leg spec |

### The clearance budget — this is what fixes `FLOOR_T`
```
45.0  ground clearance (bottom-plate underside to ground)
-33.0  pack
- 3.0  floor
= 9.0  mm under the pack, for the skid and for leg deflection
```
`FLOOR_T` is not a free choice. Do not increase it.

### Derived
```
pack spans Y = PACK_Y ± PACK_L/2          = −55.5 … +15.5
rear-stop inner face  = PACK_Y - PACK_L/2 = −55.5   (pack registers here)
rear-stop outer face                      = −58.5
deck spans Y = −58.5 … +48.0
wall inner faces  X = ±(PACK_W/2 + 0.5)   = ±17.0
wall outer faces  X                       = ±20.0
floor strap flange to X                   = ±23.0   (deck 46 wide overall)
pack underside Z                          = −36.0
aft cantilever, rear anchor (±10,−10) to deck end  = 48.5 mm
```
The deck is 46 mm wide against a 42.4 mm bottom plate, overhanging 1.8 mm per
side. The arm undersides are 3 mm above the deck's top face, so this is clear —
confirm it in CAD.

---

## 4. Geometry

### 4.1 Floor
- **3.0 mm thick**, top face at Z = 0, bearing against the bottom-plate underside
- **Wide section:** X ±23.0, Y = −58.5 … +20.0 — carries the pack, and the
  outboard 3 mm beyond each wall is the strap flange (§4.4)
- **Necked section:** X ±13.0, Y = +20.0 … +48.0 — reaches the forward anchors
  and the TF-Luna, no need for full width where there is no pack
- **Lighten aggressively.** Remove ~45% of the floor area as cutouts. The pack
  needs support at stations, not a continuous surface. Keep 5 mm of material
  around every bolt hole and a continuous 5 mm rail along both floor edges where
  the walls meet it — that junction is the load path.
- All exposed edges filleted R2

### 4.2 Side walls
- **3.0 mm thick × `WALL_H` (15.0) deep**, hanging down from the floor edges
- Run Y = −62.0 … +20.0 (the pack region only)
- Inner faces at X = ±18.0, giving 0.5 mm clearance per side on a 35 mm pack
- Scallop the wall profile between load stations to shed ~30% of the material,
  but **never below 8 mm depth** anywhere — the walls are the entire bending
  stiffness of this part
- Fillet R3 at the wall-to-floor junction, both sides

### 4.3 Rear stop
- The floor turns down into a stop wall, `WALL_H` deep, full width
- **Inner face at Y = −55.5**, outer face at −58.5 (3.0 mm thick). The pack
  registers against the inner face, so this position **is** the CG setting — it
  is the one dimension to change if the aircraft does not balance
- Filleted R3 into floor and side walls

### 4.4 Strap — slots go in an outboard floor flange, not the walls
- The floor extends 3 mm past each wall to X = ±23.0
- 2 pairs of slots **through that flange**, **2.5 (X) × 20.0 (Y)**, centred at
  `X = ±21.5`, at `Y = −30.0` and `Y = −48.0`
- Path: under the pack → up the **outer** face of each wall → through the flange
  slot → across the top of the floor → down the far slot. Closure on the floor's
  top surface.
- **Both slot stations are aft of Y = −21.2**, so the strap's top run is in open
  air with no frame above it. Verified clear of the arms: at Y = −30 the arms are
  at X = ±30, outboard of the deck.
- **Why not the side walls.** Wall slots at that height would cut the
  wall-to-floor junction, which §4.1 identifies as the load path and §7 shows is
  the entire bending stiffness of the part.
- **Why not the floor inboard of the walls.** Slots there put the strap between
  the pack's top face and the floor, holding the pack down by the strap's
  thickness and costing ground clearance twice over.
- Ground clearance with the strap under the pack: 9.0 − ~1.5 = **~7.5 mm**
- **These straps only retain the aft half of the pack.** The forward retention is
  §4.4a, and it is not optional.

### 4.4a Forward cradle stations — the pack's front end needs holding up

With `PACK_L` = 71 and `PACK_Y` = −20, the pack spans −55.5 … +15.5. Everything
forward of **Y = −21.2** sits under the bottom plate, where a transverse strap
cannot pass. Both strap stations are therefore aft of the pack's centre of mass,
and the pack's own weight rotates its nose down about them. The floor is above
the pack, so nothing resists that.

Fix: two **full-depth cradle stations** where the side walls deepen and turn
inward to carry the pack's forward end.

- Two stations, each **18.0 mm long in Y**, centred at **Y = +8.0** and **Y = −8.0**
- At each, the side walls run full depth: from the floor at Z = 0 down to
  **Z = −36.0**, the pack's underside
- **Inward lip** at each: 4.0 mm wide (wall inner face ±17.0 → ±13.0), 3.0 mm
  thick, spanning Z = −36.0 … −39.0. The pack rests on these.
- Pack loads from the rear, sliding forward over the shallow walls and onto the
  lips, then registers against the rear stop
- Fillet R3 where the deepened section meets the 18 mm wall — this is a stiffness
  discontinuity and wants a smooth transition

**These lips are also the ground-contact point**, at Z = −39, i.e. **6 mm above
ground** on the 45 mm gear versus the pack's 9 mm. They take the scrape instead
of the LiPo, which is what §4.5's deleted skid was trying and failing to do.
Cost: ~6 g.

### 4.5 Ground protection

An earlier draft put a sacrificial skid on the underside of the side walls. That
could not work — 18 mm walls stop 15 mm short of a 33 mm pack, so the pack was
the lowest point regardless.

The §4.4a cradle lips solve it as a side effect: they reach Z = −39, below the
pack's −36, so they contact ground first. No separate skid part. The landing gear
still carries the primary protection — 45 mm clearance and a sacrificial neck
([landing-leg-spec.md](landing-leg-spec.md) §4.4).

### 4.6 TF-Luna
- Mounts in the necked floor section, centred at **(0, +34.0)**, forward of the
  pack, looking straight down
- Rectangular aperture through the floor sized to the sensor's optical window
  plus 1 mm, with 2 × M2 through-holes to the sensor's own pattern
- **Sight line must clear the walls and the pack.** At Y = +34 the pack ends at
  +18.5 and the walls stop at +20, so the view is open — verify in CAD

---

## 5. Anchoring — build for both schemes

Put **both** hole sets in the part and decide at assembly.

### Preferred: centre press nut + two locators
- 1 × clearance hole at **(0, 0)** sized to the press-nut thread
- 2 × Ø4.0 blind locating spigots, 2.5 mm tall, at **(±10.0, −10.0)**, entering
  the inner stack holes to react torque without loading them
- 2 × Ø `3.2 + FIT_HOLE` at **(±10.0, +42.88)** — the forward anchor station

This carries the deck on the frame's one purpose-made bottom anchor plus the
forward pair, and never touches the FC/ESC preload column.

**Find out what thread that press nut takes before ordering fasteners.** A Ø7.0
seat suggests an M3 or M4 self-clinching nut, but confirm it.

### Fallback: inner stack pattern
- 4 × Ø `3.2 + FIT_HOLE` at **(±10.0, ±10.0)**, using stack screws lengthened by
  `FLOOR_T` + 1 mm
- Plus the same forward pair at (±10.0, +42.88)

Acceptable because a battery is a distributed static load, unlike a landing
impact. Still the second choice: it adds the pack's mass to a preloaded column
that runs through the flight controller.

**Never use the outer (±15.25, ±15.25) pattern** — those are the arm clamp bolts.

---

## 6. Print settings

| | Value |
|---|---|
| Orientation | **Floor flat on the bed, walls pointing up** |
| Why | Every wall is then a vertical extrusion with no overhang; the floor's top face — the one that mates to the frame — is the bed face and therefore dead flat; and bending stress runs in-plane with the layers |
| Layer height | 0.20 mm |
| Wall loops | **4** |
| Top / bottom shell layers | 5 / 5 |
| Sparse infill | 40% gyroid |
| Fan max / min | 30% / 20% |
| Nozzle | 250 °C |
| X-Y hole compensation | **0** — `FIT_HOLE` is in the geometry |

Printing walls-up means the 3 mm walls are 3 perimeters wide with no infill,
which is exactly what you want in a bending member. No supports needed. No brim.

Skid strips print flat, 4 walls, 100% infill.

---

## 7. Why a channel, with the numbers

3 mm × 46 mm PETG (E ≈ 2 GPa), cantilevered **48.5 mm** from the rear anchor at
(±10, −10) to the deck end, carrying the 180 g pack:

| | flat 3 mm plate | channel, 18 mm walls |
|---|---|---|
| Second moment `I` | 103.5 mm⁴ | **9,699 mm⁴** |
| Stiffness `k` | 5.4 N/mm | **510 N/mm** |
| **First mode** | **~28 Hz** | **~268 Hz** |

`WALL_H` is 18 mm and not 15 because of the margin. At 15 mm the first mode lands
near 210 Hz — only 1.4× the ~150 Hz motor fundamental at hover. 18 mm puts it at
1.8×, which is where you want a mass this large.

26 Hz is 180 g of battery oscillating inside the attitude loop's bandwidth. It
shows up as mushy handling and as gyro noise nobody can source.

Ribs cannot go **up** — the arm undersides are only 3 mm above the deck. Walls
hanging **down** cost nothing from the clearance budget because they sit beside
the pack, not under it.

Do not thin the walls below 8 mm anywhere. Stiffness scales with depth cubed, so
a locally shallow section undoes the whole argument.

---

## 8. Where `PACK_Y` comes from

Forward moments about the frame centre:

| Item | Mass | Y | Moment |
|---|---|---|---|
| Camera | 15 g | +120 | +1800 |
| IMU | 3 g | +120 | +360 |
| Bracket Part 1 | 10.4 g | +118 | +1227 |
| Bracket Part 2 | 7 g | +85 | +595 |
| Bracket fasteners | 3 g | +110 | +330 |
| TF-Luna | 5 g | +34 | +170 |
| GPS mast (if tail-mounted) | 15 g | −50 | −750 |
| Landing legs | 4 × 3 g | ±48 | 0 (symmetric) |
| **Net** | | | **≈ +3730 g·mm** |

Balanced by a 180 g pack at `PACK_Y = −3730/180 ≈ −20.7 mm`, rounded to **−20.0**.

Two consequences worth noting:

- **Put the GPS mast on the tail, not the nose.** It is the only payload item
  whose position is still free, and 15 g at −50 mm removes 750 g·mm of nose-heavy
  moment for nothing.
- This is an estimate built on estimated masses. **Weigh the finished aircraft
  and balance it on two fingers under the arms before flying** — Phase 6 step 8
  already calls for this. If the CG is off, move the rear-stop face (§4.3); it is
  the single dimension that sets it.

---

## 9. Verification checklist

- [ ] Confirm the pack loads from the rear and seats on the §4.4a lips
- [ ] Identify the centre press-nut thread; choose the §5 scheme
- [ ] Dry-fit the deck empty — confirm it clears the arms (arm undersides are
      3 mm above the deck's top face) and the rear stop clears nothing structural
- [ ] Fit the pack, confirm 0.5 mm side clearance and that it seats against the
      rear stop
- [ ] Confirm the TF-Luna sight line is clear of walls and pack
- [ ] Confirm the 48 mm-wide deck clears the arm undersides (3 mm above the deck)
- [ ] Check ground clearance under the loaded pack — ~7.5 mm with the strap; press down
      on the airframe and confirm the legs deflect before the pack contacts
- [ ] Balance the finished aircraft; trim via the rear-stop face if needed

---

## 10. Mass, and the running budget

| Item | Mass |
|---|---|
| Deck, shallow-wall channel | ~16 g |
| Forward cradle stations (§4.4a) | ~6 g |
| Fasteners | ~2 g |
| **Total** | **~24 g** |

Running total for printed parts across all three specs:

| Part | Mass |
|---|---|
| Landing legs ×4 | ~12 g |
| Camera + IMU bracket | ~20 g |
| Battery deck | ~24 g |
| Pi tray (not yet specced) | ~11 g |
| GPS mast (not yet specced) | ~4 g |
| **Total** | **~71 g** |

against the **~35 g** allowed at [PROJECT.md:289](../development-log.md). That line was
written before any of these parts existed and should be revised to ~70 g.

The consequence is small: AUW goes from ~660 to ~695 g, and hover throttle from
39% to **39.8%** — still well inside the 40–50% target band. Worth tracking, not
worth optimising.
