// Gesture puck: the small printed housing for the APDS-9960 gesture sensor, on the steering-column
// shroud with the sensor facing up at the driver's hand (BUILD_GUIDE 4.8.1, 4.8.6). Its own
// ~1.5 m cable (JST-GH 5-pin: 3V3, GND, SCL, SDA, INT) runs along the shroud and the A-pillar to the
// pod's J4. See hardware/enclosures/README.md.
//
// Print in black ASA (matte, so stray light does not bounce into the sensor). Two printed parts:
//   shell  the cover: an open window over the sensor (no lens or clear plate: a cover sheet
//          reflects the sensor's own IR LED back into it and blinds the gesture engine), the cable
//          exit with a cable-tie anchor
//   base   the floor: two posts press the module up against the shell's ceiling, two M2
//          self-tapping screws into the shell; bonded to the shroud with 3M VHB
// Frame: the base's underside on z = 0 (on the shroud), the sensor face up; x along the module.
//
//   part = "assembly" | "shell" | "base"
//
// *** Measure the module you buy and correct the lines marked VERIFY ***

part = "assembly";

// --- The module (the common purple "GY-9960" breakout; VERIFY on yours) --------------------------
mod = [20.0, 15.3, 1.6];      // board: length (x), width (y), thickness (VERIFY)
sensor_x = -5.5;               // the APDS-9960's centre from the board's centre, along x (VERIFY)
sensor = [3.94, 2.36, 1.35];   // the IC (datasheet: 3.94 x 2.36 x 1.35)
pads_x = 8.6;                  // the 6-pin row (VL GND VCC SDA SCL INT) from the centre (VERIFY)
under = 4.0;                   // room under the board for the soldered leads
post_xy = [for (x = [-mod[0] / 2 + 1.5, 2], y = [-mod[1] / 2 + 1.5, mod[1] / 2 - 1.5]) [x, y]];  // clamp points (VERIFY: bare board)

// --- The housing -----------------------------------------------------------------------------------
wall = 2;
top_t = 1.2;                   // the ceiling over the sensor: thin, so the window's walls stay out of its view
inner = [mod[0] + 10, mod[1] + 2, under + mod[2] + sensor[2] + 0.3];  // room for the screw bosses at the ends
base_t = 2;
outer = [inner[0] + 2 * wall, inner[1] + 2 * wall, base_t + inner[2] + top_t];
r = 3;
window = [8, 6];               // over the IC; flared 45 degrees outwards through the ceiling
screw_x = inner[0] / 2 - 2;    // the base's two M2 screws, at the ends (bosses merged into the end walls)
cable_d = 4.5;                 // the five-core lead (or the JST-GH crimped wires, twisted)
cable_y = inner[1] / 2 - cable_d / 2 - 0.5;  // beside the end's screw boss

module rounded(size, h, rr = r) {
    linear_extrude(h) offset(rr) offset(-rr) square(size, center = true);
}

module shell() {
    difference() {
        union() {
            difference() {
                translate([0, 0, base_t]) rounded([outer[0], outer[1]], outer[2] - base_t);
                translate([0, 0, base_t - 0.01]) rounded([inner[0], inner[1]], inner[2] + 0.01, 1);
            }
            // screw bosses at both ends, down from the ceiling to the base
            for (sx = [-1, 1]) translate([sx * screw_x, 0, base_t]) cylinder(d = 5, h = inner[2] + 0.1, $fn = 24);
            // pins down from the ceiling onto the board's top, over the base's posts: the board is
            // clamped between them (keep them off the module's parts: VERIFY post_xy on yours)
            for (xy = post_xy) translate([xy[0], xy[1], base_t + under + mod[2] + 0.1])
                cylinder(d = 2.5, h = inner[2] - under - mod[2], $fn = 16);
        }
        for (sx = [-1, 1]) translate([sx * screw_x, 0, base_t - 0.01]) cylinder(d = 1.7, h = inner[2] - 1, $fn = 16);
        // the window over the IC, flared outwards
        translate([sensor_x, 0, outer[2] - top_t - 0.01]) hull() {
            linear_extrude(0.01) square(window, center = true);
            translate([0, 0, top_t + 0.02]) linear_extrude(0.01) square(window + 2 * [top_t, top_t], center = true);
        }
        // the cable's exit at the pads' end, low, beside the screw boss (a cable tie round the lead
        // inside, against the wall, is its strain relief)
        translate([outer[0] / 2 - wall - 1, cable_y, base_t + cable_d / 2]) rotate([0, 90, 0]) cylinder(d = cable_d, h = wall + 2, $fn = 24);
        translate([outer[0] / 2 - wall - 1, cable_y - cable_d / 2, base_t - 0.01]) cube([wall + 2, cable_d, cable_d / 2]);
        // a shallow arrow towards the car's front, so the puck goes on the right way round
        // (the firmware's left/right/up/down assume it: GestureDriver)
        translate([5.5, 0, outer[2] - 0.6]) linear_extrude(1) polygon([[-2, -2.5], [-2, 2.5], [3, 0]]);
    }
}

module base() {
    difference() {
        union() {
            rounded([inner[0] - 0.4, inner[1] - 0.4], base_t + 1, 1);  // a plug that locates in the shell
            translate([0, 0, 0]) rounded([outer[0], outer[1]], base_t);
            // posts pressing the board up against the ceiling (between the IC and the pads)
            for (xy = post_xy) translate([xy[0], xy[1], 0]) cylinder(d = 2.5, h = base_t + under, $fn = 16);
        }
        // M2 x 8 self-tapping screws from below, countersunk
        for (sx = [-1, 1]) translate([sx * screw_x, 0, -0.01]) {
            cylinder(d = 2.3, h = base_t + 2, $fn = 16);
            cylinder(d1 = 4.4, d2 = 2.3, h = 1.1, $fn = 16);
        }
        // the cable's exit continues through the plug
        translate([outer[0] / 2 - wall - 3, cable_y - cable_d / 2, base_t - 0.01]) cube([wall + 4, cable_d, 2]);
        // the bosses pass through the plug
        for (sx = [-1, 1]) translate([sx * screw_x, 0, base_t - 0.01]) cylinder(d = 5.4, h = 2, $fn = 24);
    }
}

module module_model() {  // the bought module, for the picture only
    z = base_t + under;
    color("purple") translate([-mod[0] / 2, -mod[1] / 2, z]) cube(mod);
    color("black") translate([sensor_x - sensor[0] / 2, -sensor[1] / 2, z + mod[2]]) cube(sensor);
    color("gold") for (i = [0 : 5]) translate([pads_x, (i - 2.5) * 2.54, z - 0.5]) cylinder(d = 1.2, h = mod[2] + 1, $fn = 12);
}

module assembly() {
    color("dimgray") shell();
    color("gray") base();
    module_model();
    color("black") translate([outer[0] / 2, cable_y, base_t + cable_d / 2]) rotate([0, 90, 0]) cylinder(d = 3.5, h = 25, $fn = 16);
}

echo(str("Gesture puck outside: ", outer[0], " x ", outer[1], " x ", outer[2], " mm"));
if (part == "assembly") assembly();
else if (part == "shell") shell();
else if (part == "base") base();
