# Pi Roll Hoop — CAD Specification

Impact cage over the Raspberry Pi 5 on a TBS Source One V5.
One printed part. Printer: Bambu A1 mini. Material: PETG.

Companion to [pi-tray-spec.md](pi-tray-spec.md) — it bolts through that tray at
the same four frame stations, and requires two small changes to it (§8).

---

## 0. How to build this

- **All dimensions are millimetres.**
- **Z datum for this spec is the top plate's TOP FACE**, not the bottom-plate
  underside used elsewhere. The tray occupies Z 0 … 3.0. Every height below is
  measured from that face.
- **Model it parametrically.** `CROWN_Z`, `HOOP_X`, `SECT_W`, `SECT_H`,
  `FIT_HOLE` must be editable named parameters.
- `FIT_HOLE` = **+0.20 mm** placeholder.
- **Deliverables:** STEP plus STL/3MF. Single part, printed **upside down**, no
  supports — §7.

---

## 1. Design intent, and what this is not

An open cage that touches down before the Pi does.

**It is not a cover, and it must not become one.** The Pi sits above the prop
plane, which puts it in the rotors' inflow: induced velocity at the disc in hover
is √(T/2ρA) ≈ 7.6 m/s, and the Pi is in the region feeding that. Forced
convection at even a few m/s takes the convective coefficient from ~5–10 W/m²K to
~30–50. The airframe is cooling the Pi for free, and OpenVINS at ~41 ms/frame is
already the system bottleneck, so any thermal throttling comes straight off the
drift number. [vio-quad-build-plan.md:92](vio-quad-build-plan.md:92) defers the
Active Cooler decision to logged CPU temperature; an enclosure pre-empts that
decision in the wrong direction before the data exists.

So: no skin, no floor, no side panels. Six things stay reachable — microSD,
USB-C, the GPIO edge, the CSI ribbon, the shutdown button, and the cooler's own
fan intake.

### The threat model
Not debris. **Inverted landing.** Development flying on this project will produce
them: [PROJECT.md:9](PROJECT.md:9) records peak mid-flight excursions of 10–13 m
and milestone 5 notes vision crashing the aircraft.

The GPS mast already takes the first hit — its head sits **43.8 mm above the top
of the Active Cooler** (63.4 vs 20.6 above the top plate, a difference that does
not depend on your standoff length). It is sacrificial and tethered by its own
cable, so that is by design. But it is a single point at the tail; once it folds,
the aircraft rotates and the Pi is next. This part is what meets the ground then.

---

## 2. The geometry problem, and why the load goes through the tray

The Pi board is **85 × 56** and the carbon top plate is **36 mm wide**. Anything
that arches over the board has to stand outboard of X ±28, and there is no
hardpoint out there — the plate ends at ±18. Every candidate anchor is *under*
the board:

| Anchor | Why not |
|---|---|
| Directly on the top plate | Plate is ±18; a post there rises straight into the board |
| The tray's Pi standoffs | Load path would run **through the Pi board**. Never |
| Tail (±11.00, −72.88) / nose (±9.50, +76.88) | Taken by the GPS mast and the camera bracket |

That leaves the tray's own four frame bolts at **(±10.00, −42.88)** and
**(±9.50, +32.88)**, with the cage reaching outboard above the tray and below the
board to get clear.

**So the impact load does pass through 3 mm of printed tray, and that is the
right answer, not a compromise.** A cage that transmits an impact rigidly into
carbon protects nothing — the energy has to go somewhere. The tray is a £1 part
that deforms, absorbs it, and gets reprinted. Same reasoning as the landing legs'
sacrificial neck and the GPS mast's tether. The ultimate anchor is still four M3
screws into carbon; the printed plate between them is the fuse.

---

## 3. Parameters

| Name | Value | Source |
|---|---|---|
| `CROWN_Z` | **30.0 mm** | Crown underside above the top plate. §4.1 |
| `HOOP_X` | **33.0 mm** | Crown half-span |
| `SECT_W` | 5.0 mm | Crown / leg width |
| `SECT_H` | **6.0 mm** | Crown depth. §4.1 sizes it |
| `FOOT_T` | 3.0 mm | |
| `FIT_HOLE` | +0.20 mm | Placeholder |

### The height stack it has to clear
```
Z   0.0   top plate, top face  (tray bottom)
Z   3.0   tray top face
Z   8.5   Pi standoff + boss
Z  10.1   PCB top
Z  20.6   Active Cooler top          <-- the thing being protected
Z  24.1   GPIO header + Dupont shells
Z  30.0   CROWN_Z, crown underside   -- 9.4 mm over the cooler
Z  37.0   crown top, the part's highest point
Z  63.4   GPS mast head top          -- still 26 mm above this cage
```

---

## 4. Sizing

### 4.1 `CROWN_Z` = 30.0 is set by the print, not by the clearance

9.4 mm over the cooler is more than protection needs. It is what the leg
overhang demands: the aft legs run from a foot at (±10.00, Z 6.0) out to the
crown at (±33.0, `CROWN_Z`), and that line's angle from vertical is what decides
whether the part prints without supports.

| `CROWN_Z` | leg rise | leg run | angle from vertical |
|---|---|---|---|
| 27.0 | 21 | 23 | **47.6°** — needs supports |
| **30.0** | **24** | **23** | **43.8°** — prints clean |
| 33.0 | 27 | 23 | 40.4° — safer still, +2 g and +3 mm of height |

**30.0 is the lowest value that stays inside the 45° rule.** The generous cooler
gap is a free consequence.

### 4.2 `SECT_H` = 6.0 — and the honest limit of this part

Crown as a simply-supported beam over its 66 mm span, PETG σ_y ≈ 50 MPa:

| section | yields at | deflection at yield |
|---|---|---|
| 5 × 4 | 40 N | 5.0 mm |
| **5 × 6** | **91 N** | **3.4 mm** |
| 5 × 7 | 124 N | 2.9 mm |
| 5 × 8 | 162 N | 2.5 mm |

**Read this correctly.** On flat ground inverted, all four crown ends contact
together and the load runs **axially down the legs** — the crowns barely bend and
the section is irrelevant. Bending only happens on a *point* impact mid-span: a
rock, a branch, a fence post.

5 × 6 yields at 91 N with 3.4 mm of elastic deflection against a 9.4 mm gap, so
it starts to fold before it touches the cooler and has room to keep folding. That
is the intended behaviour. A point impact hard enough to close 9.4 mm was going to
cost you a Pi with or without this part — **do not size up chasing that case**,
you would only make the cage stiff enough to pass the load into the tray's bolt
holes instead.

---

## 5. Geometry

All members are rectangular in section with R1 edge fillets, and every
member-to-member junction gets **R3.0** fillets.

### 5.1 Feet — 4
- Pads Ø11.0 × `FOOT_T` (3.0), seating on the tray's top face, Z 3.0 … 6.0
- Holes Ø `3.2 + FIT_HOLE` at **(±10.00, −42.88)** and **(±9.50, +32.88)**
- **Underside relief slot, 6.5 wide × 3.0 deep**, across each pad, aligned with
  the tray's §5.4 stiffening rib — the rib starts at this same bolt and the pad
  cannot seat flat over it. Orient each slot along its rib's diagonal
- Fastening: the existing top-plate screws, now **`TRAY_T` + `FOOT_T` + 1 mm**
  longer than stock — 7 mm longer in total

### 5.2 Aft hoop — Y = −42.88
The Pi board's aft edge is at Y −42.5, so this plane is **0.38 mm clear of the
board** and the legs can rise straight from the feet with nothing in the way.

- **Legs**, section `SECT_W` (5.0, in Y) × 6.0 (in X), straight from
  (±10.00, Z 6.0) to (±33.00, Z 30.0)
- **Crown**, X −33.00 … +33.00, section 5.0 (Y) × `SECT_H` (6.0), Z 30.0 … 36.0

### 5.3 Forward hoop — Y = +32.88
Here the board *is* present (X ±28), so the run outboard has to happen underneath
it before anything rises.

- **Straps**, section 5.0 (Y) × 4.0 tall, on the tray's top face at Y +32.88,
  running X 9.50 → 33.00, occupying **Z 3.0 … 7.0**. The board's underside is at
  Z 8.5, so this clears it by **1.5 mm**
- **Posts**, X 28.50 … 33.00, section 4.5 (X) × 5.0 (Y), rising Z 7.0 … 30.0.
  Inner face 0.5 mm outboard of the board edge
- **Crown**, as §5.2, Z 30.0 … 36.0

### 5.4 Spines — 2
- At **X = ±22.0**, running Y −42.88 … +32.88, section 4.0 (X) × 6.0 (Z),
  Z 30.0 … 36.0
- They tie the hoops fore-and-aft and put a contact line over the middle of the
  board, which the two hoops alone leave open across a 75.8 mm gap
- **X ±22.0 straddles the cooler** rather than shading its fan, and sits inboard
  of the GPIO header at X ≈ ±24.5 with 5.9 mm over a fitted Dupont shell

---

## 6. Clearance checks

| Item | Position | Result |
|---|---|---|
| Cooler top | Z 20.6 | 9.4 mm under the crown |
| GPIO + Dupont shells | Z 24.1 | 5.9 mm under the spines |
| microSD, ejects forward | Y +42.5 | forward hoop is 9.6 mm aft of it |
| USB-C / HDMI / CSI | Y +42.5 | same — all forward of the hoop |
| Shutdown button boss | (0, −44.0 … −47.0) | aft hoop is at −42.88, 1.1 mm forward |
| Camera ribbon | forward edge, X 0 | no member within 22 mm of X 0 |
| GPS mast | Z 63.4 | 26 mm above this cage; still the first thing to land on |
| Side impact | — | motors are at radius 113 vs this cage's 33; motors hit first |

**Props:** at Y −42.88 the crown reaches X ±33.0 against a disc limit of ±30.47 —
2.5 mm radially inside the rear discs. This does not matter: the entire cage lives
at Z 30 … 36 above the top plate, and [pi-tray-spec.md](pi-tray-spec.md) §2.3
established the prop plane is **below** the top plate on this build. If the
middle-to-top standoffs are ever changed, re-check this row before flying.

---

## 7. Print settings

| | Value |
|---|---|
| Orientation | **Upside down — crowns and spines flat on the bed, feet uppermost** |
| Supports | **Off, explicitly** — not “auto”. §7.1 |
| Layer height | 0.20 mm |
| Wall loops | 4 |
| Top / bottom shells | 4 / 4 |
| Sparse infill | 40% gyroid |
| Fan max / min | 30% / 20% |
| **Bridge fan** | **100%** — for the §5.3 straps |
| Nozzle | 250 °C |
| Bed | **70 °C, textured PEI** — no glue stick ([PROJECT.md:44](PROJECT.md:44)) |
| X-Y hole compensation | **0** |
| Elephant foot compensation | 0.15 mm |
| **Brim** | **5 mm** — §7.1 |

**Why upside down.** The crowns and spines are all at Z 30 … 36, so inverted they
form one flat plane on the bed. Printed the other way up they would be 66 mm and
76 mm bridges in mid-air.

### 7.1 Why this one gets a brim, and explicit supports-off

**Brim.** The bed contact is not a plate — it is four narrow strips: two crowns
5 × 66 and two spines 4 × 76, about 1270 mm² around a 66 × 76 opening.
High-aspect-ratio strips lift at their ends, and this part stands 31 mm tall with
its mass carried inboard of them. A 5 mm brim is cheap; a part that lets go at
layer 200 is a dragged nozzle.

**Supports off by hand.** Bambu Studio’s threshold would leave the 43.8° legs
alone, but “auto” has no business near a part whose entire geometry was chosen to
avoid supports. Set it Off and check the preview.

**Do not plate this with the tray.** Both fit the A1 mini bed together (68 + 66
against 180 mm), but this is the taller part on the narrower feet; if it lets go,
the nozzle takes the tray with it. Two separate prints.

Two things to watch, both fine as drawn:

- The aft legs lean inward at **43.8°** from vertical (§4.1) — inside the 45°
  rule, but close. If the surface comes out poor, tilt the whole part 5° on the
  bed rather than changing the geometry
- The forward straps become **23.5 mm bridges** near the top of the print,
  between post and foot pad. PETG bridges that trivially; enable bridge settings

Print orientation also puts the layer lines **perpendicular to the legs' axial
load**, which is the direction that matters — an inverted landing on flat ground
loads them in compression, and compression across layers is PETG's strong case.

**Dry the filament.** §4.2’s yield numbers assume sound interlayer
bonding. The crowns bend *across* layers, so wet PETG does not degrade that
margin gracefully — it moves the failure from "folds at 91 N" to "snaps".

---

## 8. Two changes required in the tray spec

Both are in [pi-tray-spec.md](pi-tray-spec.md); this part does not work without
them.

1. **§5.1 — add a third edge bulge.** Widen the tray to **X ±34.0 over
   Y = +32.88 ± 6.0** so the forward straps and posts have a seat out to X 33.0.
   Without it the strap cantilevers 4 mm past the tray edge at ±29.0. Props do
   not constrain this: the limit at Y +32.88 is `|x| ≤ 37.05`.
2. **§5.2 — longer screws.** The four frame M3s become **`TRAY_T` + `FOOT_T` +
   1 mm** longer than stock, not `TRAY_T` + 1. The forward pair still takes a
   *separate* screw from the camera bracket's, entering the other end of the same
   female–female standoff.

The tray's §5.4 ribs are unchanged — §5.1's foot relief slots straddle them.

---

## 9. Verification checklist

- [ ] Foot pads seat flat on the tray with the relief slots straddling the ribs —
      no rock at any of the four
- [ ] With the Pi and cooler fitted, confirm **9.4 mm** under the crowns and that
      no spine shades the cooler's fan intake
- [ ] microSD in and out, USB-C in and out, shutdown button pressable — all with
      the cage on
- [ ] CSI ribbon runs to the nose bracket without touching a forward post
- [ ] Invert the whole aircraft onto a flat bench: it should rest on the GPS mast
      head and the four crown ends, **never on the cooler**
- [ ] Log CPU temperature on a full-length flight with the cage fitted and
      compare against the same flight without it. If it moves more than a few
      degrees, something is shading the fan and the design is wrong
- [ ] After the first real crash, look for yield at the crown mid-spans and at
      the leg roots — that is where it is designed to give

---

## 10. Mass

| Item | Mass |
|---|---|
| Crowns ×2 | 5.0 g |
| Spines ×2 | 4.6 g |
| Aft legs ×2 | 2.5 g |
| Forward posts ×2 | 1.4 g |
| Forward straps ×2 | 1.2 g |
| Feet ×4 and fillets | 2.1 g |
| **Printed total** | **~17 g** |
| Longer screws | ~1 g |
| **Total** | **~18 g** |

**Lightweight variant.** Deleting the two spines saves **4.6 g** and gives a
~12 g part. The cost is real: the two hoops no longer protect the 75.8 mm between
them, and they can fold fore-and-aft independently. Take it only if the mass
budget forces it.

Running total for printed parts, now seven: **~112 g** against the ~35 g at
[PROJECT.md:341](PROJECT.md:341). AUW ~737 g, hover throttle **40.9 %** — still
inside the 40–50 % target band, but this is the part to delete first if that
number needs to come down.
