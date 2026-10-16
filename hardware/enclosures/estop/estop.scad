// E-stop housing: a printed box for the 22 mm mushroom emergency stop (BOM 3.4: twist-release,
// NC contact + a second contact block), beside the steering column within the driver's reach
// (BUILD_GUIDE Part 0 property 2, 4.8.1). See hardware/enclosures/README.md.
//
// Print it in YELLOW ASA (ISO 13850: a red actuator on a yellow background; ASA because the cabin
// passes 70 C). Two printed parts:
//   body  a box open at the back; the button mounts through its front face, the contact blocks
//         hang inside it; the cable leaves through an M16 gland in its side
//   base  the back plate, screwed into M3 heat-set inserts in the body's corners and fixed to a
//         solid dash panel (never the steering column) with two M4 screws
// Frame: the box's back face on z = 0 (against the dash), the button face up at z = H (towards
// the driver); x, y across the face.
//
//   part = "assembly" | "body" | "base" | "holes"
//
// *** Measure the E-stop you buy and correct the lines marked VERIFY ***

part = "assembly";

// --- The button (VERIFY on yours: IEC 60947-5-1 22 mm series) --------------------------------
hole_d = 22.3;          // the standard 22 mm panel hole
key_w = 3.2;            // anti-rotation notch: 3.2 mm wide, out to 24.1 mm diameter (VERIFY)
key_r = 24.1 / 2;
panel_t = 3;            // the face: within the 1-6 mm the button's nut accepts (VERIFY)
mushroom_d = 40;        // the head, for the model and the face size (VERIFY)
mushroom_h = 30;        // head above the face, released (VERIFY)
block_depth = 48;       // behind the face: bezel base + two contact blocks + their screws (VERIFY)
block_w = 42;           // across the base with two blocks side by side (VERIFY)

// --- The box -----------------------------------------------------------------------------------
wall = 3;
inner = [max(block_w, mushroom_d) + 8, max(block_w, mushroom_d) + 8];  // inside, x and y
H = block_depth + panel_t + 8;  // outside height: the blocks, the face, room for the wiring
outer = [inner[0] + 2 * wall, inner[1] + 2 * wall];
base_t = 4;
r = 4;                  // corner radius
insert_d = 4.0;         // M3 heat-set insert: 4.0 mm hole, 5.7 mm long
boss_d = 8;
gland_d = 16.3;         // M16 x 1.5 cable gland
mount_pitch = 40;       // two M4 screws through the base into the dash panel
m4 = 4.5;

module rounded(size, h, rr = r) {
    linear_extrude(h) offset(rr) offset(-rr) square(size, center = true);
}

module bosses(h) {
    for (sx = [-1, 1], sy = [-1, 1])
        translate([sx * (inner[0] / 2 - boss_d / 2 + 1), sy * (inner[1] / 2 - boss_d / 2 + 1), 0])
            children();
}

module body() {
    difference() {
        union() {
            difference() {
                translate([0, 0, base_t]) rounded(outer, H - base_t);
                translate([0, 0, base_t - 0.01]) rounded(inner, H - base_t - panel_t + 0.01, r - wall / 2);
            }
            // corner bosses for the base's screws (heat-set inserts from the open back)
            bosses() translate([0, 0, base_t]) cylinder(d = boss_d, h = 12, $fn = 32);
        }
        // the button hole and its anti-rotation notch
        translate([0, 0, H - panel_t - 1]) {
            cylinder(d = hole_d, h = panel_t + 2, $fn = 96);
            translate([-key_w / 2, 0, 0]) cube([key_w, key_r, panel_t + 2]);
        }
        // inserts
        bosses() translate([0, 0, base_t - 0.01]) cylinder(d = insert_d, h = 6, $fn = 24);
        // the cable gland in the side wall, low, near the base
        translate([outer[0] / 2 - wall - 1, 0, base_t + gland_d / 2 + 6]) rotate([0, 90, 0])
            cylinder(d = gland_d, h = wall + 2, $fn = 64);
        // "STOP" legend sunk into the face, below the mushroom
        translate([0, -(mushroom_d / 2 + (outer[1] / 2 - mushroom_d / 2) / 2), H - 0.8])
            linear_extrude(1) text("STOP", size = 4, halign = "center", valign = "center",
                                   font = "Liberation Sans:style=Bold");
    }
}

module base() {
    difference() {
        rounded(outer, base_t);
        // M3 screws into the body's inserts, countersunk from the dash side
        bosses() translate([0, 0, -0.01]) {
            cylinder(d = 3.4, h = base_t + 1, $fn = 24);
            cylinder(d1 = 6.5, d2 = 3.4, h = 1.8, $fn = 24);
        }
        // two M4 screws to the dash panel, countersunk from inside the box
        for (sx = [-1, 1]) translate([sx * mount_pitch / 2, 0, -0.01]) {
            cylinder(d = m4, h = base_t + 1, $fn = 24);
            translate([0, 0, base_t - 2.2]) cylinder(d1 = m4, d2 = 8.6, h = 2.21, $fn = 24);
        }
    }
}

module estop_model() {  // the bought switch, for the picture only
    color("red") translate([0, 0, H]) {
        cylinder(d = 26, h = 8, $fn = 64);
        translate([0, 0, 8]) cylinder(d1 = mushroom_d - 6, d2 = mushroom_d, h = 10, $fn = 64);
        translate([0, 0, 18]) cylinder(d = mushroom_d, h = mushroom_h - 18, $fn = 64);
    }
    color("silver") translate([-block_w / 2, -12, H - panel_t - block_depth]) cube([block_w, 24, block_depth - 2]);
}

module assembly() {
    color("gold") body();
    color("goldenrod") base();
    estop_model();
    color("dimgray") translate([outer[0] / 2, 0, base_t + gland_d / 2 + 6]) rotate([0, 90, 0])
        cylinder(d = 20, h = 14, $fn = 6);
}

module holes_echo() {
    // a template for the dash panel: the base's outline and its two M4 holes, seen from the driver
    echo(str("PANEL,dash_panel,", outer[0], ",", outer[1], ",dash panel behind the E-stop base, seen from the driver"));
    for (sx = [-1, 1])
        echo(str("HOLE,dash_panel,", outer[0] / 2 + sx * mount_pitch / 2, ",", outer[1] / 2, ",", m4, ",M4 to the dash panel"));
}

echo(str("E-stop box outside: ", outer[0], " x ", outer[1], " x ", H, " mm (", H + mushroom_h, " with the head)"));
if (part == "holes") holes_echo();
else if (part == "assembly") assembly();
else if (part == "body") body();
else if (part == "base") base();
