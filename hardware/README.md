# Hardware

The custom hardware of the two-box hub. See `docs/BUILD_GUIDE.md` Part 4.8 for the design and
Part 14, Phase 12B, for when it is built. Nothing here exists yet; this file records the layout
the Phase 12B work fills in.

| Directory | Contents |
|---|---|
| `pod_board/` | KiCad project: STM32F405RGT6, 8 MHz crystal, USB-C, diode-OR'd 5 V → 3.3 V, BNO085 and NEO-M8N (v1: breakouts on standoffs), gesture connector, DB-25, status LEDs. 4-layer. |
| `power_board/` | KiCad project: 12 V protection and 60 V-rated 5 V buck, 74HCT244 with input pull-downs, BTS7960 and ACS712 modules, magnet and relay MOSFETs, 40 A kill relay, SN65HVD230 beside the OBD lead, presence MOSFET, brake-light optocoupler, LiDAR feed, DB-25. 2-layer, 2 oz. |
| `enclosures/` | Printable parts (ASA or PETG, never PLA): windscreen pod with pitch-locking carrier plate and camera hood, power box, gesture puck, and the drawing for the 3 mm aluminium 15° LiDAR wedge plate. |

Two sources of truth must agree before any board is ordered:
- `firmware/sensor_actuator_hub/include/hub/Config.h` (MCU pins; it is the pod board's netlist);
- the inter-box cable pinout, `docs/BUILD_GUIDE.md` Part 4.8.3.

Physical work (soldering, assembly, fitting in the car) is Ian's.
