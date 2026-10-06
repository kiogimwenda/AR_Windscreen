"""The windscreen pod board (BUILD_GUIDE Part 4.8.2). 4-layer, about 70 x 50 mm.

Every MCU pin assignment here must equal firmware/sensor_actuator_hub/include/hub/Config.h, and
every DB-25 pin must equal BUILD_GUIDE 4.8.3; check_nets.py verifies both from the exported
netlist. Net names: the MCU side of a cable signal is its plain name (BRAKE_RPWM); the cable side,
after the series resistor or input filter, is prefixed X_ (X_BRAKE_RPWM).
"""

from parts import (C, C0805, C1206, CONN, D, DSUB25, HOLES, LED, R, SMA, header)

# Cable signals driven by the pod: series resistor values. 1 k limits the current any line can
# push into an unpowered power box (~3 mA, Part 4.8.4), while keeping 20 kHz PWM edges clean
# over 2 m of cable. CAN_TX carries 500 kbit/s: 330 ohm keeps its edges fast.
OUTPUTS = {  # signal: (DB-25 pin, MCU pin number on LQFP64, series ohms)
    "BRAKE_RPWM": (5, "41", "1k"),     # PA8
    "BRAKE_LPWM": (7, "37", "1k"),     # PC6
    "BRAKE_EN": (8, "33", "1k"),       # PB12
    "MAGNET_EN": (9, "35", "1k"),      # PB14
    "RELAY_LEFT": (14, "26", "1k"),    # PB0
    "RELAY_RIGHT": (15, "27", "1k"),   # PB1
    "RELAY_HAZARD": (16, "9", "1k"),   # PC1
    "RELAY_HORN": (17, "10", "1k"),    # PC2
    "RELAY_BEAM": (18, "11", "1k"),    # PC3
    "CAN_TX": (20, "62", "330"),       # PB9
    "CAN_STBY": (22, "53", "1k"),      # PC12
    "HX711_SCK": (24, "20", "1k"),     # PA4
}
# Cable signals read by the pod: 10 k pull-up on the cable side (fail-safe reading when the
# cable is out), then 1 k + 10 nF into the pin (a 10 us filter against PWM crosstalk). CAN_RX and
# HX711_DOUT are data: series resistor only, no capacitor.
INPUTS = {  # signal: (DB-25 pin, MCU pin, pull-up?, filter capacitor?)
    "KILL_SENSE": (12, "36", True, True),      # PB15, NC to ground at the box: open = engaged
    "BOX_PRESENT": (13, "8", True, True),      # PC0, pulled low by the box while powered
    "BRAKE_LIGHT": (19, "57", True, True),     # PB5, optocoupler: low = pedal pressed
    "CAN_RX": (21, "61", True, False),         # PB8, recessive (high) when the cable is out
    "HX711_DOUT": (23, "15", False, False),    # PA1
}

parts = []
groups = []

# --- Power ------------------------------------------------------------------------------------
power = [
    D("D1", "SS14", "VBUS", "VIN_POD", fp=SMA, sym="Device:D_Schottky",
      desc="USB VBUS into the pod's supply (diode-OR)"),
    D("D2", "SS14", "X_5V_CABLE", "VIN_POD", fp=SMA, sym="Device:D_Schottky",
      desc="+5 V from the power box into the pod's supply (diode-OR)"),
    C("C1", "10u", "VIN_POD", "GND", C1206),
    {"sym": "Regulator_Linear:AP2112K-3.3", "ref": "U3", "value": "AP2112K-3.3",
     "fp": "Package_TO_SOT_SMD:SOT-23-5", "desc": "3.3 V, 600 mA LDO",
     "nets": {"1": "VIN_POD", "2": "GND", "3": "VIN_POD", "5": "+3V3"}},
    C("C2", "10u", "+3V3", "GND", C1206),
    R("R30", "1k", "+3V3", "LED_PWR_A"),
    LED("D3", "green", "LED_PWR_A", "GND"),
]
groups.append(("POWER: USB VBUS OR cable +5 V -> 3.3 V", power, 4))

# --- MCU --------------------------------------------------------------------------------------
mcu_nets = {
    # supplies
    "1": "+3V3", "19": "+3V3", "32": "+3V3", "48": "+3V3", "64": "+3V3", "13": "+3V3",
    "18": "GND", "63": "GND", "12": "GND",
    "31": "VCAP1", "47": "VCAP2",
    "7": "NRST", "60": "BOOT0", "5": "OSC_IN", "6": "OSC_OUT",
    # USB, SWD
    "44": "USB_DM", "45": "USB_DP", "46": "SWDIO", "49": "SWCLK",
    # on-board sensors and buses
    "59": "IMU_SDA", "58": "IMU_SCL", "24": "IMU_INT", "25": "IMU_RST",     # PB7 PB6 PC4 PC5
    "30": "GEST_SDA", "29": "GEST_SCL", "38": "GEST_INT",                   # PB11 PB10 PC7
    "16": "GPS_TX", "17": "GPS_RX", "56": "GPS_PPS",                        # PA2 PA3 PB4
    "51": "OBD_TX", "52": "OBD_RX",                                         # PC10 PC11 (ELM327)
    "2": "LED_LINK", "39": "LED_ARMED", "40": "LED_FAULT",                  # PC13 PC8 PC9
    "14": "BRAKE_CURRENT",                                                  # PA0
}
for sig, (_, pin, _) in OUTPUTS.items():
    mcu_nets[pin] = sig
for sig, (_, pin, _, _) in INPUTS.items():
    mcu_nets[pin] = sig
mcu = {"sym": "MCU_ST_STM32F4:STM32F405RGTx", "ref": "U1", "value": "STM32F405RGT6",
       "fp": "Package_QFP:LQFP-64_10x10mm_P0.5mm", "nets": mcu_nets}
decoupling = [
    C("C3", "100n", "+3V3", "GND"), C("C4", "100n", "+3V3", "GND"),
    C("C5", "100n", "+3V3", "GND"), C("C6", "100n", "+3V3", "GND"),
    C("C7", "4.7u", "+3V3", "GND", C1206, "bulk, beside the MCU"),
    C("C8", "1u", "+3V3", "GND", desc="VDDA"), C("C9", "100n", "+3V3", "GND", desc="VDDA"),
    C("C10", "2.2u", "VCAP1", "GND", desc="core regulator, low-ESR ceramic"),
    C("C11", "2.2u", "VCAP2", "GND", desc="core regulator, low-ESR ceramic"),
    {"sym": "Device:Crystal", "ref": "Y1", "value": "8MHz", "fp": "Crystal:Crystal_SMD_HC49-SD",
     "desc": "HSE; firmware assumes 8 MHz (HSE_VALUE). Load caps for CL = 10 pF.",
     "nets": {"1": "OSC_IN", "2": "OSC_OUT"}},
    C("C12", "15p", "OSC_IN", "GND", desc="crystal load"),
    C("C13", "15p", "OSC_OUT", "GND", desc="crystal load"),
    C("C14", "100n", "NRST", "GND"),
    {"sym": "Switch:SW_Push", "ref": "SW1", "value": "RESET", "fp": "Button_Switch_SMD:SW_SPST_TL3342",
     "nets": {"1": "NRST", "2": "GND"}},
    R("R1", "10k", "BOOT0", "GND", desc="BOOT0 low: run from flash"),
    {"sym": "Switch:SW_Push", "ref": "SW2", "value": "BOOT", "fp": "Button_Switch_SMD:SW_SPST_TL3342",
     "desc": "hold at reset for the USB DFU bootloader", "nets": {"1": "BOOT0", "2": "+3V3"}},
]
groups.append(("MCU: STM32F405RGT6", [mcu], 1))
groups.append(("MCU support: decoupling, crystal, reset, boot", decoupling, 5))

# --- USB --------------------------------------------------------------------------------------
usb = [
    {"sym": "Connector:USB_C_Receptacle_USB2.0_16P", "ref": "J1", "value": "USB-C",
     "fp": "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
     "desc": "to the laptop: the Part 3 protocol; also powers the pod when the cable is out",
     "nets": {"A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND",
              "A4": "VBUS", "A9": "VBUS", "B4": "VBUS", "B9": "VBUS",
              "A5": "USB_CC1", "B5": "USB_CC2", "A6": "USB_DP", "B6": "USB_DP",
              "A7": "USB_DM", "B7": "USB_DM", "SH": "USB_SHIELD"}},
    R("R2", "5.1k", "USB_CC1", "GND", desc="identifies a USB-C sink: the laptop supplies 5 V"),
    R("R3", "5.1k", "USB_CC2", "GND"),
    R("R4", "1M", "USB_SHIELD", "GND"), C("C15", "4.7n", "USB_SHIELD", "GND"),
    {"sym": "Power_Protection:USBLC6-2SC6", "ref": "U4", "value": "USBLC6-2SC6",
     "fp": "Package_TO_SOT_SMD:SOT-23-6", "desc": "USB ESD protection",
     "nets": {"1": "USB_DM", "6": "USB_DM", "3": "USB_DP", "4": "USB_DP", "2": "GND", "5": "VBUS"}},
]
groups.append(("USB-C to the laptop", usb, 3))

# --- Sensors and local connectors ------------------------------------------------------------
sensors = [
    CONN("J2", "IMU (BNO085 breakout)", ["+3V3", "GND", "IMU_SCL", "IMU_SDA", "IMU_INT", "IMU_RST"],
         desc="carrier header: match to the purchased breakout's pin order before layout; "
              "mount the breakout on standoffs, rigidly"),
    R("R5", "4.7k", "+3V3", "IMU_SCL"), R("R6", "4.7k", "+3V3", "IMU_SDA"),
    CONN("J3", "GNSS (NEO-M8N breakout)", ["+3V3", "GND", "GPS_RX", "GPS_TX", "GPS_PPS"],
         desc="MCU GPS_TX goes to the module's RX: names are the MCU's view. Match the breakout's "
              "pin order before layout"),
    CONN("J4", "Gesture puck (JST-GH)", ["+3V3", "GND", "GEST_SCL", "GEST_SDA", "GEST_INT"],
         fp="Connector_JST:JST_GH_SM05B-GHS-TB_1x05-1MP_P1.25mm_Horizontal",
         desc="APDS-9960 on its own ~1.5 m cable, I2C2 at 100 kHz"),
    R("R7", "2.2k", "+3V3", "GEST_SCL", desc="stronger pull-up for the cable"),
    R("R8", "2.2k", "+3V3", "GEST_SDA"),
    CONN("J5", "SWD (ST-Link)", ["+3V3", "SWDIO", "SWCLK", "GND", "NRST"]),
    CONN("J6", "ELM327 fallback (USART3)", ["+3V3", "GND", "OBD_TX", "OBD_RX"],
         desc="HC-05 bridge to a Bluetooth ELM327, env:hub_elm327 only (Part 4.8.5)"),
]
groups.append(("Sensors and local connectors", sensors, 3))

# --- Status LEDs ------------------------------------------------------------------------------
leds = []
for i, (net, colour) in enumerate([("LED_LINK", "blue"), ("LED_ARMED", "amber"),
                                    ("LED_FAULT", "red")]):
    leds += [R(f"R{31 + i}", "1k", net, f"{net}_A"), LED(f"D{4 + i}", colour, f"{net}_A", "GND")]
groups.append(("Status LEDs (driver-facing edge)", leds, 2))

# --- Cable interface --------------------------------------------------------------------------
cable_nets = {"1": "X_5V_CABLE", "2": "X_5V_CABLE", "3": "GND", "4": "GND", "6": "GND",
              "11": "GND", "25": "GND", "SH": "GND", "10": "X_BRAKE_CURRENT"}
cable_parts = []
n = 10
for sig, (dbpin, _, ohms) in OUTPUTS.items():
    cable_nets[str(dbpin)] = "X_" + sig
    cable_parts.append(R(f"R{n}", ohms, sig, "X_" + sig, desc=f"series, DB-25 pin {dbpin}"))
    n += 1
for sig, (dbpin, _, pullup, cap) in INPUTS.items():
    cable_nets[str(dbpin)] = "X_" + sig
    cable_parts.append(R(f"R{n}", "1k" if cap else "100", "X_" + sig, sig,
                         desc=f"series, DB-25 pin {dbpin}"))
    n += 1
    if pullup:
        cable_parts.append(R(f"R{n}", "10k", "+3V3", "X_" + sig,
                             desc="pull-up: the safe reading when the cable is out"))
        n += 1
    if cap:
        cable_parts.append(C(f"C{n}", "10n", sig, "GND"))
        n += 1
# Current sense: analogue, filtered to remove the 20 kHz PWM ripple (1 k x 100 nF = 1.6 kHz)
cable_parts += [R(f"R{n}", "1k", "X_BRAKE_CURRENT", "BRAKE_CURRENT"),
                C(f"C{n + 1}", "100n", "BRAKE_CURRENT", "GND"),
                R(f"R{n + 2}", "100k", "X_BRAKE_CURRENT", "GND",
                  desc="defined reading (0 V) when the cable is out; firmware ignores it then")]
db25 = {"sym": "Connector:DB25_Socket_MountingHoles", "ref": "J7", "value": "DB-25 to power box",
        "fp": DSUB25, "desc": "inter-box cable, BUILD_GUIDE 4.8.3", "nets": cable_nets}
groups.append(("Inter-box cable (DB-25), series resistors and input filters", [db25], 1))
groups.append(("Cable line conditioning", cable_parts, 6))

groups.append(("Mounting", HOLES("H", 4), 4))

PROJECT = "pod_board"
TITLE = "Windscreen pod board (MCU, sensors, USB, cable)"
POWER_FLAGS = ["GND", "VBUS", "VIN_POD", "X_5V_CABLE"]
