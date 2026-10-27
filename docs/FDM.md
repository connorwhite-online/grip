# Printing GRIP at home (FDM edition)

GRIP was drawn for SLS nylon, but it prints fine on an ordinary filament printer. The FDM edition uses the same design and the same bought parts. Only the fits change, and the links are printed solid with infill instead of as hollow shells. One arm takes about 2.1 kg of filament (about $45), so **the whole arm costs about $540** instead of about $1,400 with bureau SLS parts.

```bash
cd cad
python build.py --fdm        # writes ../exports/fdm
```

Every STL in `exports/fdm/stl/` is already turned to its print orientation and sits on the bed at z = 0. Load it and slice it as it comes.

## What changes, and what it costs you

| | SLS edition | FDM edition |
| --- | --- | --- |
| Printed parts | about $890 from a bureau | about $45 of filament |
| Whole arm (with $492 of bought parts) | about $1,400 | about $540 |
| Links | hollow shells, 2 mm walls | solid, 4 walls + 15% gyroid, cable tunnels and driver bays cut inside |
| Moving mass (upper arm to wrist link) | 466 g | 642 g |
| Extra load on the shoulder at full reach | | +0.49 N·m, about 0.1 kg of payload |
| Gearbox running clearance | 0.15 mm | 0.20 mm (a little more backlash) |
| Press fits (dowels, rods, bearings) | drawn 0.03–0.05 mm under | drawn at nominal; FDM holes come out a little small |
| Finish | smooth, matte | layer lines and support marks on the domes |

**Does FDM have enough resolution?** Yes. The parts that need precision are the cycloid discs, the pin holes and the bearing seats, and all of them lie flat on the bed. Their accurate dimensions are traced by the nozzle in X and Y, where a tuned printer holds about ±0.1 mm. That is as good as a typical SLS bureau (about ±0.2 mm). The disc lobes are printed as solid perimeters, and the pin loads run along the layers rather than across them.

**What does get worse:**

- **Weight.** Solid links with infill are about 40% heavier than SLS shells. The shoulder gearbox has a lot of headroom (about 30 N·m), and the printed links only add 0.49 N·m to its load at full reach. That costs about 0.1 kg of payload and nothing else.
- **Wear.** PETG-CF discs on steel pins will wear faster than glass-filled nylon. They are cheap to reprint (about 20 g each). If you'd rather not think about it, order just the six pairs of discs and the eccentrics from a bureau in SLS PA12-GF (about $90) and print everything else. The FDM and SLS discs are the same part, apart from the 0.05 mm of extra clearance.
- **Finish.** The soft, organic shapes show layer lines on their shallow domes, and the tree supports leave marks underneath. A light sanding and a coat of primer hides most of it.
- **Printer.** On a 180 mm printer (Bambu A1 mini) everything prints **except two links**: the forearm (183 mm tall) and the upper arm (284 mm long). On a 220 mm printer (Ender-3 class) only the upper arm is too big; it needs a 256 mm bed (Bambu A1, P1S or X1), turned at an angle. Have the parts that don't fit printed by a local makerspace or an online FDM or SLS service (about $15–25 each in PETG). `exports/fdm/printed_parts.csv` lists the smallest printer each part fits.

## Before you start: the fit coupon

Print `fit_coupon.stl` first, in the filament and with the settings you'll use for the real parts. Each row is one fit the arm uses: Ø3, Ø4 and Ø5 dowels, the Ø6 guide rods, the Ø12 LM6UU bearings and the Ø21 6801 bearing. Each row has three holes, at 0.1 mm under, nominal (marked with a notch) and 0.1 mm over.

Push the real part into each hole. The nominal hole should take it with the vise, tight. If only the larger hole takes it, raise your slicer's hole (XY) compensation by 0.05 mm and print the coupon again. If the smaller hole takes it, lower it. Getting this right before you print 2 kg of parts is the single biggest thing you can do for the quality of the arm.

## Materials

| Material | Used for | Amount |
| --- | --- | ---: |
| PETG | links, gearbox housings and caps, palm, rod frame | 1.79 kg |
| PETG-CF (or PA-CF) | cycloid discs, eccentrics, output flanges, pinion, jaws | 0.32 kg |
| TPU 95A | fingers | 0.03 kg |

Add about 15% for supports and the odd failed print. PLA+ works for the links if you keep the arm out of hot rooms and sunlight, but it creeps under constant load, so use PETG for anything in a gearbox. PETG-CF and PA-CF need a hardened steel nozzle. Dry them before printing, at 65 °C for PETG-CF and 80 °C for PA-CF, for 6 hours.

## Print settings

| Parts | Material | Layer | Walls | Infill | Supports |
| --- | --- | --- | --- | --- | --- |
| base, turret, upper_arm, forearm, wrist_yoke, wrist_link | PETG | 0.2 mm | 4, 5 top and bottom | 15% gyroid | tree, from the build plate only |
| gearbox housing, cap | PETG | 0.2 mm | 5 | 40% gyroid | none |
| cycloid discs, eccentric, output flange | PETG-CF | 0.16 mm | 6 | 50% gyroid | none |
| gripper palm, rod frame | PETG | 0.2 mm | 4 | 25% gyroid | none |
| gripper jaws, pinion | PETG-CF | 0.16 mm | 6 | 50% gyroid | none |
| fingers | TPU 95A | 0.2 mm | 3 | 100% | none |

- **Elephant's foot.** Turn on first-layer (elephant's foot) compensation, about 0.15 mm. The discs and housings sit on their working faces, and a flared first layer will bind them.
- **Seams.** Put the seam at the back (aligned) on the discs. A random seam leaves a blob on a lobe somewhere.
- **Brims.** Use a 5 mm brim on the links. They are tall and stand on a small flat.
- **Orientation.** The links stand with the socket that takes their gearbox facing up, so the bore is round and needs no support. A link whose underside is a dome has 0.6 mm shaved off it to give it a flat to stand on. The jaws stand on their outer end so the bearing bores are vertical, and the fingers lie on their side so the fin-ray ribs are drawn in one layer. Gearbox parts lie flat as drawn.

## Prepare the parts

This replaces step 2 of the [build guide](BUILD.md#2-prepare-the-printed-parts). Everything after that is the same.

1. **Remove the supports** and clean the socket ledges and hub faces, which carry the gearboxes. A deburring tool takes the strings out of the cable tunnels.
2. **Check that the cable tunnels are clear.** Push a length of stiff wire through each link from one cable port to the motor bay. Each link has a 10 mm tunnel (8 mm in the wrist), which is plenty for the four-wire bus and the gripper's three-wire cable.
3. **Pin holes and bearing seats** as in the build guide. If you tuned your printer with the coupon, a dowel presses in with the vise and stays put. If a hole is tight, run the matching drill bit through by hand.
4. **Discs.** Scrape the bottom edge of each lobe with a deburring blade to take off any first-layer flare, then check that both discs turn freely on the ring pins before you close the gearbox.
