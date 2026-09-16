# Raspberry Pi 5 Tray — CAD Specification

Top-deck tray for the Pi 5 + cooler on a TBS Source One V5.
One printed part. Printer: Bambu A1 mini. Material: PETG.

**Revision note — two passes on the first draft.**

1. **Geometry.** §2 was built on motor positions `(±79.9, ±79.9)`. Those were
   wrong; the frame is a wide X. Every clearance number has been recomputed, and
   the tray's own aft corners turned out to foul the rear discs (§5.1).
2. **Pre-handoff review.** Five further problems, three of which would have
   reached the printer: the stiffness claim in §5.4 was ~2× optimistic and
   contradicted §5.1's band width; the shutdown button in §6.1 could not be
   made as drawn; the Pi standoffs had no edge margin (§5.1); the frame-bolt
   counterbore was unnecessary and thinned the joint (§5.2); and §6.2's
   pass-through landed under the board.

3. **Height gate closed.** `Z_HEAD` was the one open measurement. Checked on the
   assembled aircraft: the props sit below the top of the frame and below the Pi,
   so `Z_HEAD` ≤ 0 and §2.2's plan-view constraint no longer binds. `PI_Y` is a
   free parameter and §8 is retained only for reference.

All are fixed below. The part now carries four small ribs and a button boss that
the first draft did not have, and has no open measurements.

---

## 0. How to build this

- **All dimensions are millimetres.**
- **Import the official Raspberry Pi 5 STEP / mechanical drawing** from
  `datasheets.raspberrypi.com` rather than modelling the board from numbers.
  The tray's access cutouts (§6) are defined by connector positions, and those
  come from the model, not from a table.
- **Model it parametrically.** `PI_Y`, `STANDOFF_H`, `TRAY_T`, `FIT_HOLE` must be
  editable named parameters.
- `FIT_HOLE` = **+0.20 mm** placeholder, carried from the landing-leg coupon.
- **Read §2 before drawing anything.** Whether the Pi's fore-aft position is free
  turns on one measurement that has not been taken.
- **Deliverables:** STEP plus STL/3MF. Single part.

---

## 1. Design intent

A flat plate bolted on top of the top plate at four existing standoff positions,
carrying the Pi 5 on standoffs above it. It keeps the microSD slot, USB and GPIO
reachable — the stated reason for putting the Pi up here at all
([vio-quad-build-plan.md:90](../archive/vio-quad-build-plan.md)) — and gives the GPIO
shutdown button a mount that can be pressed with props on.

---

## 2. Prop clearance — read this first

### 2.1 The motor positions

Re-extracted from `SO_V5_5inch_Arm_6mm.dxf`, which draws the four arms about a
shared root pattern at the frame origin. The four Ø7.0 motor bores are at:

```
(±86.563, ±72.635)      radius 113.00 mm, azimuth 40.00° from +X
track  173.13 mm across (X)  ×  145.27 mm fore-aft (Y)
prop disc radius 63.5 mm about each
```

**This is a wide-X frame, not a true X.** The first draft said `radius 113.0 at
±45° → (±79.9, ±79.9)`; the radius was right and the angle was not. The check
that settles it: the 90°-rotated reading would put the prop discs 4.4 mm *inside*
the top and bottom plates, which is impossible, while these positions clear every
plate by 8.1–9.7 mm.

### 2.2 The Pi board against the discs

The Pi 5 is 85 × 56 mm, long axis fore-aft, so its corners are at (±28, `PI_Y` ±42.5):

| `PI_Y` | nearest prop-disc clearance | |
|---|---|---|
| −12.0 | −2.19 mm | inside the disc |
| −8.0 | −0.89 mm | inside the disc |
| −5.6 | −0.01 mm | at the edge |
| −4.0 | +0.63 mm | |
| **0.0** | **+2.36 mm** | best case |
| +4.0 | +0.63 mm | |
| +5.6 | −0.01 mm | at the edge |
| +12.0 | −2.19 mm | inside the disc |

**Plan-view solution band: `PI_Y` ∈ [−5.59, +5.59] — 11.17 mm wide.**

**§2.3 has since retired this constraint** — the prop plane is below the top
plate on this build, so nothing on the tray is in the sweep at any `PI_Y`. The
table is kept because it is the only record of what the band *would* be, and
because a change of standoff length brings it straight back.

`PI_Y` = **0.0** as the default. It is now a free parameter, so it may be moved
to trim CG or a cable route.

### 2.3 Resolved — the prop plane is below the top plate

2.36 mm is not an engineering margin, so it only matters if the Pi sits at prop
height. The first draft asserted the top plate is below the prop disc, reasoning
from the plate's bounding-box corner. That was wrong — the plate is tapered
there, and its real outline clears the rear discs by **+8.06 mm** and the front by
**+9.69 mm**, so its shape proves nothing about its height.

**Checked on the assembled aircraft: the props sit below the top of the frame,
and below the Pi.** In the sign convention below that is `Z_HEAD` ≤ 0 — the first
row of the table, and the one that dismisses the whole problem.

> **`Z_HEAD` = signed distance from the top plate's top face up to the underside
> of a mounted prop.** Negative when the top plate is already above the prop plane.

| `Z_HEAD` | Verdict |
|---|---|
| **≤ 0 — this build** | **Top plate is above the prop plane. Nothing on the tray can be struck at any `PI_Y`. §2.2 does not apply, and §8 is dead** |
| ≥ 20.6 (Active) / ≥ 15.6 (passive) | Whole stack below the disc — also fine |
| 10.6 … 20.6 | Only the cooler straddles; its ~40 mm footprint clears by +21 mm |
| 0 … 10.6 | The board straddles. Not acceptable at 2.36 mm |

(the thresholds are `TRAY_T` 3.0 + `STANDOFF_H` 5.0 + PCB 1.6 + rib 1.0 = 10.6,
plus a ~10 mm Active Cooler or ~5 mm passive heatsink)

### 2.4 The one thing still worth a caliper: how far below

`Z_HEAD` ≤ 0 settles the static case. The dynamic case is **prop coning** — a
loaded blade lifts at the tip, and the tray's aft corners sit at radius ≈ 63.0 mm
from the rear motors, i.e. essentially at the blade tip.

So the margin that matters is `|Z_HEAD|`:

| `|Z_HEAD|` | What to do |
|---|---|
| **≥ 5 mm** | Nothing. Coning on a 5" PC 3-blade will not cover it |
| **< 5 mm** | Keep §5.1's aft chamfer and re-check after the first hard punch-out |

Either way **the chamfer stays in the model** — it costs 0.5 g, removes the last
radial overlap, and is free insurance if the standoffs ever change.

---

## 3. Frame hardpoints

From `SO_V5_TopPlate_2mm.dxf`, tag `v5`, registered to the middle and bottom
plates through the holes and slots all three share. Frame coordinates: origin at
the FC stack centre, **+Y = nose**, **+Z = up**.

| Feature | Position | Notes |
|---|---|---|
| Top plate outline | X ±18.00, Y −77.29 … +81.38 | Only 36 mm wide — the Pi overhangs it by 10 mm per side |
| **Aft mount pair** | **(±10.00, −42.88)** | Ø3.0, 20.0 mm spacing |
| **Forward mount pair** | **(±9.50, +32.88)** | Ø3.0, 19.0 mm spacing |
| Tail pair | (±11.00, −72.88) | Ø3.0 — **taken by the GPS mast** |
| Nose pair | (±9.50, +76.88) | Camera bracket standoff |
| Centre strap slots | 2.00 × 34.00 at X ±12.00, Y −17.90 … +16.10 | See §6.2 |
| Aft strap slots | 2.00 × 10.00 at X ±10.60, Y −62.88 … −52.88 | GPS mast tie-down |
| Aft cable slot | 8.97 × 6.12 at X ±4.48, Y −52.00 … −45.88 | Keep clear |

**The two mount pairs have different X spacing** — ±10.00 aft and ±9.50 forward.
The four tray holes are not a rectangle. Model them from these coordinates; do
not assume symmetry.

The pairs are 75.76 mm apart in Y and straddle the frame centre, which is what
makes them the right anchors: the Pi's own mounting holes sit *inboard* of the
frame bolts in Y, so the tray carries no fore-aft cantilever.

---

## 4. Parameters

| Name | Value | Source |
|---|---|---|
| `PI_Y` | **0.0 mm** | Free — §2.3 retired the prop constraint |
| `PI_X` | 0.0 mm | Centred |
| `PI_HOLES` | 58.0 × 49.0, M2.5 | [vio-quad-build-plan.md:89](../archive/vio-quad-build-plan.md) |
| `PI_BOARD` | 85.0 × 56.0 mm | Long axis fore-aft |
| `TRAY_T` | **3.0 mm** | |
| `STANDOFF_H` | 5.0 mm | M2.5 nylon, from the assortment already ordered |
| `FIT_HOLE` | +0.20 mm placeholder | |
| `Z_HEAD` | **≤ 0 — resolved on the aircraft** | §2.3. Not a gate any more |

### Derived
```
Pi mounting holes at  (±24.5, ±29.0)
Pi board spans        X ±28.0,  Y ±42.5
tray extent           X ±29.0,  Y −47.0 … +37.0
                      aft corners chamfered, edge bulged to ±30.5 at the
                      Pi standoffs — both §5.1
cantilever, frame bolt to nearest Pi standoff
                      forward 15.49 mm,  aft 20.07 mm  (the aft one governs, §5.4)
```

---

## 5. Tray geometry

### 5.1 Body
- Flat plate, **`TRAY_T` (3.0 mm)**, spanning X ±29.0, Y −47.0 … +37.0
- **Aft corners chamfered from (±29.0, −43.0) to (±26.0, −47.0).** As drawn
  square, the corners at (±29.0, −47.0) sit **0.49 mm inside the rear prop
  discs** — the tray fouls where the board does not, because it is 1 mm wider and
  reaches 4.5 mm further aft. §2.3 has since shown the tray is above the prop
  plane, so this is now insurance against coning and against a future change of
  standoff, not a hard requirement. **Keep it anyway** — 0.5 g. The aft limit is
  `|x| ≤ 28.47` at y = −47.0, and there is no constraint at all inboard of
  `|x| = 23.06`
- Forward corners are clear by 5.0 mm and need no relief
- Bottom face bears on the top plate across its full 36 mm width
- **Edge bulges at the four Pi standoffs.** At X ±29.0 there is only 4.5 mm from
  a Pi hole centre to the tray edge — 3.05 mm of material past the Ø2.9 bore,
  which is not enough to hang a standoff on in a 3 mm plate. Bulge the edge
  locally to **X ±30.5 over Y = ±29.0 ± 6.0**, giving 6.0 mm centre-to-edge.
  Props do not constrain this: the aft limit at y = −29.0 is `|x| ≤ 40.4`
- **A third bulge, for the roll hoop.** Widen to **X ±34.0 over Y = +32.88 ± 6.0**
  so the hoop's forward straps and posts have a seat out to X 33.0
  ([pi-hoop-spec.md](pi-hoop-spec.md) §5.3). Limit at y = +32.88 is `|x| ≤ 37.05`
- All outer edges filleted R2
- **Lighten to ~45% removed.** Keep a continuous 5 mm perimeter rail, 5 mm of
  material around every frame bolt, and an unbroken **12 mm-wide** band along each
  diagonal from a frame bolt to its nearest Pi standoff — those four bands are
  the load path, and §5.4 sizes them

### 5.2 Frame mounting — 4 × M3
- Ø `3.2 + FIT_HOLE` at **(±10.00, −42.88)** and **(±9.50, +32.88)**
- **No counterbore.** A Ø6.2 × 2.0 pocket leaves 1.0 mm under the head in a 3 mm
  plate, and it buys nothing: the board sits `STANDOFF_H` + boss = 5.5 mm above
  the tray and an M3 cap head is 3.0 mm, so it already clears by 2.5 mm
- Existing top-plate screws are replaced with ones **`TRAY_T` + `FOOT_T` + 1 mm
  longer** — 7 mm over stock — because the roll hoop's feet stack on top of the
  tray at all four stations ([pi-hoop-spec.md](pi-hoop-spec.md) §5.1). Without
  the hoop it is `TRAY_T` + 1
- **The forward pair is shared with the camera bracket, but not the screw.**
  [camera-imu-bracket-spec.md](camera-imu-bracket-spec.md) §2 captures Part 2's
  flange *under the middle plate* at (±9.50, +32.88), lengthening that screw by
  4 mm. This tray lengthens the **top**-plate screw by 3 mm. Two screws into
  opposite ends of the same female–female standoff — order both, and check they
  do not meet in the middle

### 5.3 Pi mounting — 4 × M2.5
- Ø `2.7 + FIT_HOLE` at **(±24.5, ±29.0)**
- **M2.5 nylon standoffs, `STANDOFF_H` (5.0 mm)** — non-structural here, and
  nylon keeps the board electrically isolated from everything
- Raise a 0.5 mm boss around each hole so the standoff seats on a defined pad
  rather than on a printed surface

### 5.4 Stiffness — four ribs, and why

The tray is a plate bolted at four points and loaded at four points; the first
mode moves the **whole** 58 g of Pi + cooler, not half of it. And the two
cantilevers are not the same length:

```
forward  (±9.50, +32.88) -> (±24.5, +29.0)   L = 15.49 mm
aft      (±10.00, −42.88) -> (±24.5, −29.0)  L = 20.07 mm   <-- the governing one
```

Stiffness goes as 1/L³, so the aft band is 2.2× softer than the forward one.
With four bands in parallel, `K = 3EI/L³` summed, `f = (1/2π)√(K/m)` on 58 g:

| band | section | **K** | **f₁** |
|---|---|---|---|
| 6 mm, plate only | 6 × 3 | 86 N/mm | **194 Hz** |
| 12 mm, plate only | 12 × 3 | 173 N/mm | 275 Hz |
| 12 mm + **2.5 mm top rib** | 12 × 5.5 | **353 N/mm** | **393 Hz** |
| 12 mm, `TRAY_T` = 4.0 instead | 12 × 4 | 271 N/mm | 344 Hz |

194 Hz is **1.2× the ~160 Hz hover fundamental** — the motors sit on it. So:

- **Load-path bands 12 mm wide** (§5.1), not 6.
- **A rib on each band, 6.0 wide × 2.5 tall, on the tray's TOP face**, running
  from the frame-bolt pad out to the Pi standoff boss.

The ribs go on top because that is the only free side: the tray's underside bears
on the top plate across ±18, and stiffening only the outboard tip does nothing —
the root carries the moment. There is `STANDOFF_H` (5.0) of air above the tray, so
a 2.5 mm rib leaves 2.5 mm to the board's underside, clear of the microSD
housing. **1.2 g for 2× the frequency.**

Verify against the imported model that no rib lands under an underside component,
and that none crosses the CSI ribbon's path.

---

## 6. Access and routing

Take every position below from the imported Pi 5 model, not from memory.

- **CSI camera connectors face forward (+Y).** This is the orientation rule:
  the 300 mm ribbon runs to the nose bracket, and pointing the connectors aft
  costs a 180° U-turn in a cable that is already strain-relieved at both ends.
- **microSD** — cut the tray fully away beneath the slot, plus a 15 mm-wide
  finger relief, so a card can be pushed and pulled with the Pi in place. Verify
  against the model once oriented; if the slot lands over a load-path band,
  reroute the band, not the relief.
- **USB-C, HDMI, USB-A, Ethernet** — no tray material within 3 mm of any
  connector opening
- **GPIO header** — clear along its whole length. The BEC 5 V lands on pins 4
  and 6 and the FC UART on GPIO14/15 — **physical pins 8 (TX) and 10 (RX)**, ground on pin 14; not physical pins 14/15, which are GND and GPIO22 —, so this edge stays permanently occupied
- **Ribbon and wiring channel** — a 10 mm-wide relief in the tray's forward edge
  at X = 0, with a zip-tie slot pair either side, so the CSI ribbon and the FC
  UART loom drop through and tie off to the tray rather than pulling on
  connectors

### 6.1 Shutdown button
- Mount on the tray's **aft edge**, facing rearward, at (0, −46)
- **It needs a boss.** A Ø12.0 recess cannot go in the edge face of a 3 mm plate —
  the face is 3 mm tall. Stand a boss **14.0 (X) × 3.0 (Y) × 12.0 (Z)** proud of
  the tray's **top** face, its rear face flush with the tray's aft edge at
  Y −47.0, and bore **Ø6.5 through it in Y** on the tray's centreline at Z +6.0
- **Teardrop the bore.** The tray prints flat, so this bore is horizontal and its
  top arc is unsupported: a plain circle droops into the 0.5 mm of slack a 6 mm
  switch has. Replace the top of the circle with a **45° peak** from the
  tangent points up to an apex at Z +9.25 — a standard teardrop
- 3.0 mm of panel is the right thickness for a 6 mm momentary switch's own nut
- The boss clears the rear prop discs by **+20.7 mm** and the Pi board's aft edge
  (Y −42.5) by 4.5 mm. Its 12 mm of height costs nothing now that §2.3 is settled
- Aft-facing is the only edge reachable with props on
- Wires to GPIO3 (pin 5) and GND (pin 9). **Not pin 6** — that is the BEC ground
- **Clearance check:** the GPS mast's foot begins at Y −52.00. Its central relief
  (X ±7.00) is what keeps a finger able to reach this button — confirm on the
  printed parts, not in CAD ([gps-mast-spec.md](gps-mast-spec.md) §9)

### 6.2 Down-pass to the FC — route it outboard, do not cut the tray

The obvious route is the top plate's own centre strap slots (2.00 × 34.00 at
X ±12.00, Y −17.90 … +16.10), unused now the battery is bottom-mounted. **Do not
use them.** They sit entirely under the Pi board (X ±28, Y ±42.5), so a loom
arriving there has to travel ~13 mm outboard to reach the GPIO header — in the
5 mm gap under the board.

The tray overhangs the 36 mm-wide top plate by **11 mm per side with open air
beneath it**. Run the FC UART and the BEC 5 V up the outside at X ≈ ±20 and
straight to the header at X ≈ ±25. Nothing is cut, nothing passes under the board.

- **Two zip-tie slot pairs**, 3.5 × 1.5, on the tray's underside at
  **(±20.0, −20.0)** and **(±20.0, +12.0)**, so the loom is anchored to the tray
  and the FC's connectors carry no load

---

## 7. Print settings

| | Value |
|---|---|
| Orientation | **Flat on the bed**, bottom face down |
| Why | The bottom face mates to the top plate and wants to be planar; bending runs across layers rather than peeling them |
| Layer height | 0.20 mm |
| Wall loops | 4 |
| Top / bottom shells | 5 / 5 |
| Sparse infill | 40% gyroid |
| Fan max / min | 30% / 20% |
| Nozzle | 250 °C |
| X-Y hole compensation | **0** — `FIT_HOLE` is in the geometry |
| Bed | **70 °C, textured PEI** — no glue stick ([PROJECT.md:44](../development-log.md)) |
| Elephant foot compensation | 0.15 mm |

At 3 mm with 5+5 shells at 0.2 mm, the part is 2 mm solid shell and 1 mm infill —
effectively solid, which is what you want in a thin plate. No brim, no supports.

**Dry the filament first.** Every structural claim in §5.4 assumes sound
layer adhesion, and wet PETG does not give it.

---

## 8. Fallback — the 45° rotation (not needed on this build)

**§2.3 resolved to `Z_HEAD` ≤ 0, so this section is dead.** It is retained only
because a change of middle-to-top standoff length would bring the height gate
back, and this is the answer if it ever lands in the `0 … 10.6` band with neither
the cooler nor the standoffs able to give it up. Rotating the Pi 45° points its
corners into the gaps between the arms:

| Orientation | `PI_Y` = 0 clearance |
|---|---|
| Axis-aligned | +2.36 mm |
| **Rotated 45°** | **+8.88 mm** |

Mounting holes move to **(±3.18, ±37.83)** and **(±37.83, ±3.18)**.

**The cost is stiffness.** The nearest frame bolt to the hole at (37.83, 3.18) is
41 mm away, versus 15.5 mm axis-aligned. A plain 3 mm tray over that span drops to
roughly 80 Hz with a quarter of the Pi's mass on it — below the ~160 Hz hover
motor fundamental and unacceptable. The rotated version therefore needs ribbed
arms or `TRAY_T` raised to 5 mm, costing ~5 g.

It also turns every connector diagonal, which makes cable routing and SD access
worse. Treat it as the answer only if the height gate genuinely fails.

---

## 9. Verification checklist

- [ ] `Z_HEAD` ≤ 0 confirmed on the aircraft — **done**. If the middle-to-top
      standoffs are ever changed, re-run §2.3 before flying
- [ ] Caliper `|Z_HEAD|` once and check it against §2.3's coning table
- [ ] Confirm the §5.1 aft chamfer is in the model
- [ ] Dry-fit the bare tray: four frame bolts land, and it bears flat on the
      36 mm top plate without rocking on a cutout edge
- [ ] With the Pi mounted, check microSD insert and removal without tools
- [ ] Confirm the shutdown button is reachable with props **and the GPS mast**
      fitted
- [ ] Fit the four §5.4 ribs before assuming the "no ribs" advice from the first
      draft — that advice was wrong
- [ ] **A USB-A plug will not fit aft with the GPS mast on.** Board edge −42.5,
      shell ≈ −44.5, plug body ~12 mm → −56.5, against a mast foot at −52.00.
      4.5 mm of interference. Bench the Pi over USB-C (forward) or off the
      airframe; this is a workflow note, not a fault
- [ ] The microSD probably needs **no** cutout: the card ejects forward past
      Y +42.5 and the tray ends at +37. Confirm on the model before cutting one
- [ ] Confirm the CSI ribbon reaches the nose bracket without tension at full
      travel, and ties off to the tray
- [ ] Confirm the §6.2 pass-through lines up with the plate's centre slots

---

## 10. Mass

| Item | Mass |
|---|---|
| Tray, 3 mm, ~45% removed | ~8.5 g |
| Four stiffening ribs (§5.4) | ~1.2 g |
| Shutdown-button boss (§6.1) | ~1.0 g |
| Standoffs and fasteners | ~2.5 g |
| **Total** | **~13 g** |

Up 2 g on the first draft, all of it in §5.4 and §6.1.

The **roll hoop** ([pi-hoop-spec.md](pi-hoop-spec.md)) adds a further ~18 g on
these same four bolts. With it, the running total for all seven printed parts is
**~112 g** and hover throttle **40.9 %**.
