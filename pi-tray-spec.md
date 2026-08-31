# Raspberry Pi 5 Tray — CAD Specification

Top-deck tray for the Pi 5 + cooler on a TBS Source One V5.
One printed part. Printer: Bambu A1 mini. Material: PETG.

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
- **Read §2 before drawing anything.** The Pi's fore-aft position is not a free
  choice on this airframe, and that is not obvious.
- **Deliverables:** STEP plus STL/3MF. Single part.

---

## 1. Design intent

A flat plate bolted on top of the top plate at four existing standoff positions,
carrying the Pi 5 on standoffs above it. It keeps the microSD slot, USB and GPIO
reachable — the stated reason for putting the Pi up here at all
([vio-quad-build-plan.md:90](vio-quad-build-plan.md:90)) — and gives the GPIO
shutdown button a mount that can be pressed with props on.

---

## 2. The prop-clearance constraint — read this first

The Pi 5 is 85 × 56 mm. The frame's motors sit at **(±79.9, ±79.9)** (radius
113.0, from `SO_V5_5inch_Arm_6mm.dxf`) and each 5" prop sweeps a **63.5 mm**
disc. Checking the Pi's four corners against those four discs:

| Pi centre `PI_Y` | nearest prop-disc clearance | |
|---|---|---|
| −10.0 | −4.81 mm | inside the disc |
| −5.0 | −2.31 mm | inside the disc |
| −2.0 | −0.67 mm | inside the disc |
| **0.0** | **+0.48 mm** | the only position that clears |
| +2.0 | −0.67 mm | inside the disc |
| +5.0 | −2.31 mm | inside the disc |

**The plan-view solution band is `PI_Y` ∈ [−0.81, +0.81] — 1.62 mm wide.**

So: **`PI_Y` = 0.0, and it is not adjustable.** Do not shift the Pi to trim CG,
tidy a cable run, or make a cutout fit. If the tray forces a compromise, move the
tray, not the Pi.

### 0.48 mm is not a margin — the real clearance must be vertical

A 0.48 mm plan-view gap is a coincidence, not an engineering margin. It only
matters if the Pi sits at prop height, so the design rule is:

> **The Pi assembly must be entirely below the prop disc. It must never straddle it.**

The top plate is itself below the prop disc — provable from its own geometry:
its nose corner at (18, +77) is 62.0 mm from the front motor centre, 1.5 mm
*inside* the 63.5 mm disc. A carbon plate cannot occupy the prop's sweep, so the
plate must sit under it. The open question is only **how much headroom exists
above the top plate before the prop plane.**

### The measurement that closes this
Measure `Z_HEAD` — top-plate top face to the underside of a mounted prop.

```
required = TRAY_T (3.0) + STANDOFF_H (5.0) + PCB (1.6) + cooler height
         = 9.6 + cooler
Active Cooler   ~10 mm  ->  ~19.6 mm needed
Passive heatsink ~5 mm  ->  ~14.6 mm needed
```

If `Z_HEAD` ≥ required, build as specified. If not, in order:

1. **Drop the Active Cooler for a low-profile passive heatsink** — buys ~7 mm,
   and [vio-quad-build-plan.md:92](vio-quad-build-plan.md:92) already proposes
   deciding this from logged CPU temperature rather than in advance.
2. **Reduce `STANDOFF_H`** to 3 mm — buys 2 mm.
3. **Rotate the Pi 45°** — see §8. Clearance jumps to +12.36 mm and the height
   question disappears entirely, at a real cost in tray stiffness.

---

## 3. Frame hardpoints

From `SO_V5_TopPlate_2mm.dxf`, tag `v5`, aligned to the middle plate via the
shared 19.0 mm standoff pairs. Frame coordinates as in the other specs: origin at
the FC stack centre, **+Y = nose**, **+Z = up**.

| Feature | Position | Notes |
|---|---|---|
| Top plate outline | X ±18.0, Y −77.3 … +81.4 | Only 36 mm wide — the Pi overhangs it by 10 mm per side |
| **Aft mount pair** | **(±10.00, −42.89)** | Ø3.0, 20.0 mm spacing |
| **Forward mount pair** | **(±9.50, +32.88)** | Ø3.0, 19.0 mm spacing |
| Tail pair | (±11.00, −72.89) | 22.0 mm spacing — unused |
| Nose pair | (±9.50, +76.88) | Carries the camera bracket standoff — unused here |

**Note the two mount pairs have different X spacing** — ±10.00 aft and ±9.50
forward. The four tray holes are not a rectangle. Model them from these
coordinates, do not assume symmetry.

The pairs are 75.77 mm apart in Y and straddle the frame centre, which is what
makes them the right anchors: the Pi's own mounting holes then sit *inboard* of
the frame bolts in Y, so the tray carries no fore-aft cantilever.

---

## 4. Parameters

| Name | Value | Source |
|---|---|---|
| `PI_Y` | **0.0 mm** | **Fixed by §2. Not adjustable.** |
| `PI_X` | 0.0 mm | Centred |
| `PI_HOLES` | 58.0 × 49.0, M2.5 | [vio-quad-build-plan.md:89](vio-quad-build-plan.md:89) |
| `PI_BOARD` | 85.0 × 56.0 mm | Long axis fore-aft |
| `TRAY_T` | **3.0 mm** | |
| `STANDOFF_H` | 5.0 mm | M2.5 nylon, from the assortment already ordered |
| `FIT_HOLE` | +0.20 mm placeholder | |
| `Z_HEAD` | **measure** | Top-plate top face → prop underside. §2 |

### Derived
```
Pi mounting holes at  (±24.5, ±29.0)
Pi board spans        X ±28.0,  Y ±42.5
tray extent           X ±29.0,  Y −47.0 … +37.0
lateral cantilever, frame bolt to nearest Pi hole  ≈ 15.5 mm
```

---

## 5. Tray geometry

### 5.1 Body
- Flat plate, **`TRAY_T` (3.0 mm)**, spanning X ±29.0, Y −47.0 … +37.0
- Bottom face bears on the top plate across its full 36 mm width
- All outer edges filleted R2, corners R4
- **Lighten to ~45% removed.** Keep 5 mm of material around every hole, a
  continuous 5 mm perimeter rail, and an unbroken 6 mm-wide band along each
  diagonal from a frame bolt to its nearest Pi standoff — those four bands are
  the load path

### 5.2 Frame mounting — 4 × M3
- Ø `3.2 + FIT_HOLE` at **(±10.00, −42.89)** and **(±9.50, +32.88)**
- Counterbore the top face Ø6.2 × 2.0 so the heads sit below the Pi
- Existing top-plate screws are replaced with ones **`TRAY_T` + 1 mm longer**

### 5.3 Pi mounting — 4 × M2.5
- Ø `2.7 + FIT_HOLE` at **(±24.5, ±29.0)**
- **M2.5 nylon standoffs, `STANDOFF_H` (5.0 mm)** — non-structural here, and
  nylon keeps the board electrically isolated from everything
- Raise a 0.5 mm boss around each hole so the standoff seats on a defined pad
  rather than on a printed surface

### 5.4 Stiffness — no ribs needed
The worst cantilever is lateral, ~15.5 mm from a frame bolt to the nearest Pi
standoff. A 3 mm plate over that span gives roughly 230 N/mm, putting the first
mode near 450 Hz with half the Pi's mass on it. Ribs would only get in the way of
the board above and the top plate below.

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
  and 6 and the FC UART on pins 14/15, so this edge stays permanently occupied
- **Ribbon and wiring channel** — a 10 mm-wide relief in the tray's forward edge
  at X = 0, with a zip-tie slot pair either side, so the CSI ribbon and the FC
  UART loom drop through and tie off to the tray rather than pulling on
  connectors

### 6.1 Shutdown button
- Mount on the tray's **aft edge**, facing rearward, at (0, −45)
- Ø12.0 recess with a Ø6.5 through-hole for a standard 6 mm momentary switch,
  retained by its own nut
- Aft-facing is the only edge reachable with props on
- Wires to GPIO3 (pin 5) and GND (pin 9). **Not pin 6** — that is the BEC ground

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
| Elephant foot compensation | 0.15 mm |

At 3 mm with 5+5 shells at 0.2 mm, the part is 2 mm solid shell and 1 mm infill —
effectively solid, which is what you want in a thin plate. No brim, no supports.

---

## 8. Fallback — the 45° rotation

If §2's height gate cannot be met even without the Active Cooler, rotating the Pi
45° points its corners into the gaps between the arms:

| Orientation | `PI_Y` = 0 clearance |
|---|---|
| Axis-aligned | +0.48 mm |
| **Rotated 45°** | **+12.36 mm** |

Mounting holes move to **(±3.18, ±37.83)** and **(±37.83, ±3.18)**.

**The cost is stiffness.** The nearest frame bolt to the hole at (37.83, 3.18) is
41 mm away, versus 15.5 mm axis-aligned. A plain 3 mm tray over that span drops to
roughly 80 Hz with a quarter of the Pi's mass on it — below the ~150 Hz motor
fundamental and unacceptable. The rotated version therefore needs ribbed arms or
`TRAY_T` raised to 5 mm, costing ~5 g.

It also turns every connector diagonal, which makes cable routing and SD access
worse. Treat it as the answer only if the height gate genuinely fails.

---

## 9. Verification checklist

- [ ] Measure `Z_HEAD` and run the §2 gate **before** printing anything
- [ ] Confirm `PI_Y` = 0.0 in the model — this is the one dimension with no
      tolerance to give
- [ ] Dry-fit the bare tray: four frame bolts land, and it bears flat on the
      36 mm top plate without rocking on a cutout edge
- [ ] With the Pi mounted, check microSD insert and removal without tools
- [ ] Confirm the shutdown button is reachable with props fitted
- [ ] Confirm the CSI ribbon reaches the nose bracket without tension at full
      travel, and ties off to the tray

---

## 10. Mass

| Item | Mass |
|---|---|
| Tray, 3 mm, ~45% removed | **~8.5 g** |
| Standoffs and fasteners | ~2.5 g |
| **Total** | **~11 g** |

Consistent with the ~11 g carried in the battery-deck spec's running budget, so
the printed-parts total stays at **~71 g** and hover throttle at ~39.8%.
