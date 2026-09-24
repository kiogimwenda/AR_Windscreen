# Bill of Materials

Hardware needed to take the build past Phase 1. Every line traces to a section of `BUILD_GUIDE.md`.
Sourcing is **Kenya-first**: a local supplier is listed wherever one was found online, and an import
route is given only where nothing suitable is sold locally.

Prices were researched on **2026-09-24**. Kenyan maker shops go out of stock often, and several items
below showed as sold out that day. **Before you travel to a shop, confirm stock by phone or WhatsApp.**
"Est." means no price was published and the figure is a market estimate.

**Local suppliers used**

| Shop | Where | Contact |
|---|---|---|
| K-Technics | Munyu Rd Business Centre, 2nd flr, Rm C6, Nairobi CBD | +254 712 799 123 · ktechnics.com |
| Pixel Electric | Nairobi | pixelelectric.com |
| Nerokas | Nairobi | store.nerokas.co.ke |
| ArduinoTech Kenya | Nairobi CBD pickup | +254 724 011 572 (WhatsApp) · arduinotech.co.ke |
| Jumia / Kilimall | Online, countrywide delivery | jumia.co.ke · kilimall.co.ke |
| Auto-spares strip (Kirinyaga Rd / Luthuli Ave), used-parts yards | Nairobi | walk-in |

---

## 1. Sensor & Actuator Hub (Part 4)

| # | Item | Spec that matters | Source | Price (KES) | Status |
|---|---|---|---|---|---|
| 1.1 | **STM32F401CCU6 "Black Pill" v3.0** | Cortex-M4F, 3× USART, 3× I2C, USB-C. Meets Part 4.1's minimum. | K-Technics | 900 | In stock |
| 1.1b | *Alternative:* STM32F405RGT6 dev board | Adds a hardware CAN controller, for reading OBD over CAN directly later. | K-Technics | 3,200 | In stock |
| 1.2 | ST-Link V2 (mini clone) | SWD flashing and real breakpoint debugging. USB DFU can also flash, but it cannot debug a stuck FreeRTOS task. | K-Technics | 750 | In stock |
| 1.3 | Logic-level converter, 4-ch (buy ×2) | The BTS7960 module's input buffer and 5 V relay boards do not switch reliably from 3.3 V logic. See note A. | K-Technics | 150 each | In stock |
| 1.4 | Perfboard, header pins, JST/Dupont leads, screw terminals | Hub carrier board | K-Technics / Pixel | ~1,000 est. | Common |

**Buying 1.1 changes the firmware target.** `platformio.ini` has to move from the guide's placeholder
`blackpill_f411ce` to `blackpill_f401cc`. The F411 is not sold locally right now (Nerokas lists it at
KES 2,000, out of stock). The F401 runs at 84 MHz instead of 100 MHz and has 256 KB of flash instead of
512 KB. Both are ample for this hub.

## 2. Sensors (Part 4.2, Part 6, Part 8)

| # | Item | Spec that matters | Source | Price (KES) | Status |
|---|---|---|---|---|---|
| 2.1 | **BNO085 IMU breakout** (Adafruit 4754 or equivalent) | On-chip sensor fusion. Part 4.1 already pins the `Adafruit BNO08x` library. | **Import.** No Kenyan listing found. | ~US$25 | See §7 |
| 2.1b | *Local fallback:* BNO055 breakout | Similar fusion IMU, but needs a different driver library (decision needed). | Jumia | est. 2,500–4,000 | Listed |
| 2.2 | **u-blox NEO-M8N GPS** with antenna | 72-ch multi-GNSS, UART. Get a board with a u.FL/SMA connector if the antenna will go on the roof (Part 15.2 step 1). | Pixel Electric (GY-GPSV3) / Nerokas (with EEPROM + antenna) | 1,800 | **Both sold out.** Ring them. NEO-6M (~1,100) is a fallback at lower accuracy. |
| 2.3 | **ELM327 OBD-II adapter, Bluetooth** | Get one with a genuine PIC18F25K80 ("v1.5"). Cheap "v2.1" clones drop commands and often lack protocol support. | Jumia / Kilimall | est. 1,000–2,000 | Listed |
| 2.4 | HC-05 Bluetooth module | Master mode, pairs with the ELM327 and bridges it to UART3. This is the Bluetooth-SPP option Part 4.2 permits. | K-Technics 650 / Pixel 600 | 600–650 | Listed |
| 2.5 | APDS-9960 gesture sensor | I2C, shares the bus with the IMU | Pixel Electric | 800 | **Sold out.** Ring them, or check Jiji. PAJ7620 (Pixel) is a fallback but needs a different library. |
| 2.6 | **USB camera, 1440p ("2K")** | Must allow **manual focus lock** over UVC/V4L2. Autofocus invalidates the Part 12.1 intrinsics every time it moves. Check this before buying. | Jumia (e.g. UGREEN CM778, other 2K models) | 6,000–13,000 | Listed |
| 2.7 | **Livox Mid-360 LiDAR** | 360°×59° FoV, 9–27 V DC, 100 Mbit Ethernet, the Livox-SDK2 device Part 2.8 installs | **Import.** No Kenyan stockist found. | ~US$749 list | See §7 |
| 2.8 | Livox three-wire aviation cable (M12 → RJ45 + power + function) | **Not included** with the Mid-360. Without it you cannot power or talk to the LiDAR. | Import with 2.7 (DJI or third-party) | ~US$20 | See §7 |
| 2.9 | USB 3.0 → Gigabit Ethernet adapter | **Only if your laptop has no RJ45 port.** Part 2.2/Appendix C also use it as the `usbipd` fallback. | Jumia (UGREEN, generic) | est. 1,000–2,500 | Listed |

## 3. Actuation and safety (Part 4.2, Part 13.3, Part 15.2)

> **Do not buy the brake actuator yet.** See note B. The actuator's mechanical design is an open
> safety decision, and the obvious local purchase would violate the guide's hard rule.

| # | Item | Spec that matters | Source | Price (KES) | Status |
|---|---|---|---|---|---|
| 3.1 | **BTS7960 43 A H-bridge module** | PWM plus direction, with built-in current-sense (IS) pins | Jumia (listed) / Pixel (1,800, sold out) | est. 1,200–1,800 | Listed on Jumia |
| 3.2 | ACS712-20A current sensor | An independent, calibratable current reading for Part 4.5's overcurrent latch and Part 13.3 logging. The BTS7960's IS pins vary ±20 % part-to-part. | Nerokas / K-Technics / Jumia | est. 400–600 | Listed |
| 3.3 | **Brake actuator** | *Pending note B.* Pull-only coupling, bounded force, fast enough to matter. | — | — | **Blocked on design** |
| 3.4 | **Emergency-stop switch**, 22 mm mushroom, twist-release, **NC contact + a second contact block** | Driver-reach kill switch (Part 0 property 2). The second contact drives the hub's sense line. | Industrial electrical suppliers (Luthuli Ave / Industrial Area). Not found online. | est. 800–1,500 | Walk-in |
| 3.5 | 12 V automotive relay, 40 A, SPST-NO, with socket (×2) | The E-stop switches this relay's **coil**, and the relay carries the actuator current. Contacts are closed only while the E-stop is released, so the kill path fails safe. | Auto-spares strip | est. 300–500 each | Common |
| 3.6 | 4-ch 5 V relay module, opto-isolated | Indicators, hazards, horn, high beam (Part 4.2) | K-Technics / Pixel (550) / Nerokas | 550 | Listed |
| 3.7 | PC817 optocoupler module (2-ch) | *Recommended addition, not in the guide.* Reads the car's brake-light switch so the hub knows the driver is braking. See note C. | Pixel / K-Technics | est. 150–300 | Common |

## 4. Power (Part 15.2 step 8)

| # | Item | Spec that matters | Source | Price (KES) | Status |
|---|---|---|---|---|---|
| 4.1 | 12 V → 5 V buck converter, 5 A, automotive input range (6–32 V) | Powers the hub, sensors and relay coils | Jumia / Nerokas / Pixel | est. 500–1,000 | Listed |
| 4.2 | Inline blade-fuse holders plus assorted ATO fuses | Separate fuses for the peripheral rail, the actuator rail and the LiDAR | Auto-spares strip | est. 500 | Common |
| 4.3 | Wire: 1.5 mm² and 2.5 mm² automotive, red and black; heat-shrink; crimp ring terminals | Actuator and LiDAR runs | Auto-spares / electrical shops | est. 1,500 | Common |
| 4.4 | **Laptop in-car power**. See note D. | The guide's USB-C PD car charger will not run this laptop under GPU load. | — | — | **Decision needed** |

## 5. Bench rig (Part 4.7, Part 13.3)

| # | Item | Spec that matters | Source | Price (KES) | Status |
|---|---|---|---|---|---|
| 5.1 | Bench 12 V supply, ≥10 A (or a car battery plus charger) | Powers the actuator, LiDAR and hub off-vehicle | Jumia / electrical shops | est. 2,500–5,000 | Common |
| 5.2 | Load cell, 50 kg, with HX711 amplifier | Measures the **applied force** Part 13.3 step 2 logs. Without it, "force" is only inferred from current. | Nerokas (50 kg cell 1,700; HX711 separately) | ~2,000 | Check stock |
| 5.3 | Used brake-pedal assembly (any car) | The "pedal fixture" in Part 13.3 step 1. A real pedal, pivot and return spring. | Used-parts yards | est. 1,000–3,000 | Walk-in |
| 5.4 | Plywood/steel base, brackets, bolts | Rigid mounting so the rig measures the actuator, not the rig flexing | Hardware store | est. 1,500 | Common |
| 5.5 | Multimeter (skip if you own one) | — | K-Technics / Pixel | est. 1,500 | Common |

## 6. Installation (Part 15)

| # | Item | Source | Price (KES) |
|---|---|---|---|
| 6.1 | Adjustable laptop mount (seat-bolt or passenger-side floor pole) | Jumia | est. 3,000–6,000 |
| 6.2 | Camera mount behind the mirror (suction or adhesive GoPro-style) | Jumia | est. 500–1,000 |
| 6.3 | LiDAR mount (roof bar clamp or magnetic base plus plate) | Hardware/fabricator | est. 1,500–3,000 |
| 6.4 | Add-a-fuse tap for the switched ACC circuit | Auto-spares strip | est. 300 |
| 6.5 | Cable ties, split loom, trim tools | Auto-spares strip | est. 800 |

---

## 7. Import order: bundle it into one shipment

Four items were not found in any Kenyan shop: the **Mid-360 (2.7)**, its **cable (2.8)**, the
**BNO085 (2.1)**, and, depending on note B, the **actuator (3.3)**. Put them in **one** order. Duty and
clearing costs are per consignment, so four separate parcels cost roughly four times as much to clear.

Rough landed cost for the Mid-360 bundle, assuming ≈ US$810 CIF and an exchange rate of about
129 KES/USD:

| Charge | Basis | Approx. |
|---|---|---|
| Import duty | 0–25 % of CIF depending on how KRA classifies a LiDAR; assume the worst case | up to US$200 |
| IDF | 2.5 % of CIF | US$20 |
| RDL | 2 % of CIF | US$16 |
| VAT | 16 % of (CIF + duty + IDF + RDL) | ~US$168 |
| **Landed** | | **≈ US$1,050–1,220 → KES 135k–157k** |

**Get a written quote from a clearing agent before paying for anything.** A university purchase order
can sometimes qualify for education duty relief. Ask your department before you order, not after the
parcel is held at customs.

DJI's own store returned "not available in your country/region" when fetched on 2026-09-24. Verify from
Kenya. If it holds, buy from an authorised Livox reseller that ships to Kenya, and confirm it is a
genuine unit with a serial number. The Mid-360's default IP is derived from that serial number.

## 8. Budget summary

| Block | Approx. KES |
|---|---|
| Hub + sensors bought locally (§1, §2 excl. 2.1/2.7/2.8) | 12,000 – 22,000 |
| Actuation + safety, excluding the actuator (§3) | 4,500 – 7,500 |
| Power, excluding the laptop solution (§4) | 2,500 – 3,000 |
| Bench rig (§5) | 8,000 – 13,000 |
| Installation (§6) | 6,000 – 11,000 |
| **Local subtotal** | **≈ 33,000 – 56,000** |
| Import bundle (§7), dominated by the LiDAR | ≈ 135,000 – 160,000 |
| Actuator + laptop power | open, see notes B and D |
| **Total** | **≈ 170,000 – 220,000**, plus the notes B/D items |

The LiDAR is about three-quarters of the budget. If that is a problem, raise it with your supervisor
**before** buying. Dropping it breaks Part 8, the near-field ground model in Part 11.4, and the
guide's headline navigation-precision objective. It is not a cheap substitution.

---

## Notes

**A. 3.3 V logic against 5 V modules.** The STM32 drives 3.3 V. Most BTS7960 boards put a 74HC-family
buffer in front of the IC, powered at 5 V, and that buffer needs about 3.5 V to see a logic "high".
3.3 V sits just below that line, so the output works on some boards and fails on others. Opto-isolated
relay boards have the reverse problem: with the input driven to 3.3 V, the opto LED can stay partly
lit, so the relay chatters or never fully releases. On a brake PWM line, "works on some boards" is
not acceptable. Put a level converter on every output that goes to these modules.

**B. The brake actuator: do not buy the obvious one.** The local listings (Nerokas 900 N lead-screw
actuator, Jumia generic 12 V actuators) are all **lead-screw** designs. Three problems follow, and
each one conflicts with the guide's hard rule:

1. **A lead screw is not back-drivable.** When power is cut, it stays wherever it stopped. If it is
   rigidly linked to the pedal, a kill-switch cut or watchdog release leaves the brake **applied**.
   That is the exact opposite of "defaults released on any fault."
2. **900 N is well above a driver's push.** A rigid linkage at that force can out-force the driver.
3. **It is too slow.** These actuators move at 5–15 mm/s, so pressing a pedal 40 mm takes 3–8 s. That
   is useless for a TTC-triggered brake.

A design direction worth taking to your supervisor, not a decision made here:
- a **pull-only coupling**: a cable or strap that can pull the pedal down but goes slack if the driver
  presses further, so the pedal linkage can never be blocked;
- a **fail-safe release in that cable**: for example, a 12 V holding electromagnet powered through the
  kill-switch relay (3.5). If power is lost for any reason, the cable drops free. Its rated holding
  force also becomes a **mechanical force ceiling** that no firmware bug can exceed. That would be a
  third ceiling after the arbiter and `kMaxSafeIntensity`;
- a **faster, lower-force actuator**, around 50–100 mm/s and 100–200 N, which on a boosted brake is
  already enough for firm braking. These are mostly import items (AliExpress "high speed linear
  actuator"), so fold one into the §7 bundle once the design is agreed.

Measure the chosen design on the §5 rig, per Part 13.3, before anything else depends on it.

**C. Brake-pedal detection.** Part 3.1's `obdBrakePedalActive` comes from an OBD PID that many cars do
not expose. In that case it reads `0xFF` and the arbiter cannot tell whether the driver is already
braking. Tapping the brake-light switch through an optocoupler (3.7) works on every car and is
galvanically isolated from the vehicle wiring. This is a suggested addition to Part 4.2, not a change
made yet.

**D. Laptop power.** A laptop with an RTX 5060 and a Core Ultra 7 275HX ships with a roughly 200–280 W
barrel-jack adapter. A 100–140 W USB-C PD car charger (Part 15.2 step 8) cannot keep up under
inference load, so the battery drains during a drive. Options:
- a **300 W+ pure-sine inverter** wired to the battery through its own fuse, running the laptop's own
  adapter (a cigarette-lighter socket is usually fused at 10–15 A, which is too little);
- accept **battery-only** runs for the short Stage B–D test windows, and charge between runs.
Either way, record it in `decisions.md` when Phase 15 arrives. The second option costs nothing and
may be sufficient.
