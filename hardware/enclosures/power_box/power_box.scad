// Power box: a printed chassis plate inside a commercial ABS junction box (about 250 x 150 x 100 mm
// outside, IP65 class). See BUILD_GUIDE Part 4.8.4/4.8.6 and hardware/enclosures/README.md.
//
// Why a bought box: the contents (power board 110 x 80, BTS7960 module with its heatsink, ACS712,
// six automotive relays in 28 mm sockets, eight cable glands) need about 240 x 140 mm of floor; a
// printed box that size exceeds most printer beds (220 x 220) and would be weaker and leakier than
// an ABS junction box. The printed chassis plate (210 x 130 mm) carries everything and is screwed to
// the box floor from below; the box walls and floor are drilled from 1:1 templates
// (make_templates.py).
//
// Frame: box interior. X along the 240 mm length, Y across from the FRONT wall (Y = 0: the DB-25
// side), Z up from the inside floor. The board's outline and holes come from ../common/boards.scad
// (generated with the KiCad outline), so the standoffs follow the board.
//
// The power board lies component side up with its top edge (DB-25, fuses) against the front wall.
// Seen from above, that is KiCad's view turned 180 degrees: KiCad x runs right-to-left here
// (board_to_plate()), which puts the module headers (KiCad's right edge) on the LEFT, beside the
// BTS7960 and ACS712.
//
//   part = "assembly" | "chassis" | "holes"
//
// *** Measure the box, modules and relay sockets you buy, and correct the lines marked VERIFY ***

include <../common/boards.scad>

part = "assembly";

// --- The box (VERIFY: measure the one you buy) ---------------------------------------------------
box_in = [240, 140, 95];       // interior length, width, depth (to the lid seal)
box_wall = 3;                  // wall and floor thickness

// --- Chassis plate ---------------------------------------------------------------------------------
plate = [210, 130];
plate_t = 4;
plate_xy = [15, 0];            // in box coordinates: centred along X, against the front wall
floor_fix = [[90, 6], [90, 70], [6, 86], [204, 86]];  // M4 from below the box into inserts
fix_boss_h = 8;                // the plate's 4 mm + 4 mm boss above: room for an M4 x 8.1 insert

// Power board (outline and holes from boards.scad)
board_x = 96;                  // plate X of the board's KiCad right edge (it lies flipped, see above)
board_standoff = 6;            // M3 heat-set inserts in 7 mm bosses
function board_to_plate(p) = [board_x + power_board[0] - p[0], p[1]];

// BTS7960 module ("IBT-2", 50 x 50 board): VERIFY the hole positions on yours
bts_xy = [6, 10];              // module's corner on the plate
bts_size = [50, 50];
bts_holes = [[3, 3], [47, 3], [3, 47], [47, 47]];
bts_standoff = 5;
bts_heatsink_h = 40;           // VERIFY: height of the heatsink above the module's board

// ACS712-20A module (31 x 13): VERIFY hole positions (often two 3 mm holes 25 mm apart)
acs_xy = [62, 30];
acs_size = [31, 13];
acs_holes = [[3, 6.5], [28, 6.5]];

// Relay sockets: five signal relays and the 40 A kill relay (the last, red in the assembly),
// automotive "mini" relays in 28 x 28 mm sockets with one M4 mounting tab each (VERIFY).
relay_count = 6;
relay_pitch = 33;
relay_row = [8, 92];           // first socket's corner on the plate
relay_size = [28, 28];
relay_tab = [14, -5];          // the tab's hole, relative to the socket's corner

// Cable glands, M16 x 1.5 (cable 4-8 mm), in the rear wall (Y = box_in.y)
// (left to right seen from inside; the battery feed and actuator nearest the kill relay)
glands = ["LiDAR 12 V (M12 lead)", "OBD-II lead (CAN, keep it ~0.5 m)", "brake-light switch",
          "relay contacts to the car's switch wiring", "12 V ACC (switched) in", "E-stop (kill relay coil)",
          "actuator + cable magnet", "12 V battery feed (fused at the battery)"];
gland_d = 16.3;
gland_z = 60;                  // centre height above the inside floor: above the relays (36 mm)
function gland_x(i) = 20 + i * (box_in[0] - 40) / (len(glands) - 1);

// DB-25 (shell size B, right-angle PCB socket with board locks): VERIFY against your connector
db25_shell_h = 12.5;           // shell height above the board
db25_z_on_board = 6.3;         // shell centre above the board's top face
db25_x = plate_xy[0] + board_to_plate([power_board_keeps[0][0] + power_board_keeps[0][2] / 2, 0])[0];
db25_z = plate_t + board_standoff + power_board[2] + db25_z_on_board;

// Vents: slots in the left wall, level with the BTS7960 heatsink
vent_y = [plate_xy[1] + bts_xy[1] + 6, plate_xy[1] + bts_xy[1] + bts_size[1] - 6];  // span along Y
vent_z = plate_t + bts_standoff + 2 + bts_heatsink_h / 2;                          // centre height
vent_slot = [4, 26];

insert_m3 = 4.0;               // M3 x 5.7 brass insert hole (VERIFY the insert's datasheet)
insert_m4 = 5.6;               // M4 x 8.1
$fn = 32;

module boss(h, d = 7, hole = insert_m3, depth = 6) {
    difference() {
        cylinder(d = d, h = h);
        translate([0, 0, h - depth]) cylinder(d = hole, h = depth + 1);
    }
}

// --- Chassis (printed, ASA or PETG) ---------------------------------------------------------------
module chassis() {
    difference() {
        union() {
            linear_extrude(plate_t) offset(4) offset(-4) square(plate);
            for (p = floor_fix) translate(p) cylinder(d = 11, h = fix_boss_h);
        }
        // M4 inserts from below (the screws come up through the box floor)
        for (p = floor_fix) translate([p[0], p[1], -1]) cylinder(d = insert_m4, h = 9.1);
        // cable-tie slots between the relays, and along the board's left edge
        for (i = [0:relay_count]) translate([relay_row[0] - 4 + i * relay_pitch, relay_row[1] + 8, -1])
            cube([3, 6, plate_t + 2]);
        for (y = [20, 50]) translate([board_x - 5, y, -1]) cube([3, 6, plate_t + 2]);
    }
    for (h = power_board_holes) translate(concat(board_to_plate(h), plate_t)) boss(board_standoff);
    for (h = bts_holes) translate([bts_xy[0] + h[0], bts_xy[1] + h[1], plate_t]) boss(bts_standoff);
    for (h = acs_holes) translate([acs_xy[0] + h[0], acs_xy[1] + h[1], plate_t]) boss(4, 6);
    for (i = [0:relay_count - 1])
        translate([relay_row[0] + i * relay_pitch + relay_tab[0], relay_row[1] + relay_tab[1], plate_t])
            boss(5, 10, insert_m4, 5);
}

// --- Assembly (for checking only) -------------------------------------------------------------
module keep(k, z, hgt) {  // a board "keep" box, placed through the flip
    a = board_to_plate([k[0], k[1]]);
    translate([a[0] - k[2], a[1], z]) cube([k[2], k[3], hgt]);
}

module assembly() {
    %difference() {
        translate([-box_wall, -box_wall, -box_wall]) cube(box_in + [2 * box_wall, 2 * box_wall, box_wall]);
        cube(box_in + [0, 0, 1]);
    }
    translate([plate_xy[0], plate_xy[1], 0]) {
        color("orange") chassis();
        zb = plate_t + board_standoff;
        color("green") translate([board_x, 0, zb]) cube(power_board);
        color("silver") keep(power_board_keeps[0], zb + power_board[2], db25_shell_h);
        color("gold") keep(power_board_keeps[1], zb + power_board[2], 12);
        color("limegreen") keep(power_board_keeps[2], zb + power_board[2], 12);
        color("white") keep(power_board_keeps[3], zb + power_board[2], 9);
        color("darkred") translate([bts_xy[0], bts_xy[1], plate_t + bts_standoff]) cube([bts_size[0], bts_size[1], 2]);
        color("gray") translate([bts_xy[0] + 5, bts_xy[1] + 5, plate_t + bts_standoff + 2])
            cube([40, 40, bts_heatsink_h]);
        color("blue") translate([acs_xy[0], acs_xy[1], plate_t + 4]) cube([acs_size[0], acs_size[1], 8]);
        for (i = [0:relay_count - 1])
            color(i == relay_count - 1 ? "red" : "black")
                translate([relay_row[0] + i * relay_pitch, relay_row[1], plate_t]) cube([relay_size[0], relay_size[1], 32]);
    }
    for (i = [0:len(glands) - 1])
        color("dimgray") translate([gland_x(i), box_in[1] - 12, gland_z]) rotate([-90, 0, 0])
            cylinder(d = 22, h = 12 + box_wall + 10);
}

// --- Hole lists for the drilling templates (make_templates.py) ---------------------------------
// Each wall is drawn as seen from OUTSIDE the box, origin at its bottom-left outside corner; the
// floor is drawn as seen from INSIDE (lay the template on the floor, front wall at the bottom).
module holes_echo() {
    W = box_wall;
    L = box_in[0] + 2 * W;
    D = box_in[1] + 2 * W;
    H = box_in[2] + W;
    echo(str("PANEL,front_wall,", L, ",", H, ",front wall (DB-25), seen from outside"));
    echo(str("DSUB,front_wall,", db25_x + W, ",", db25_z + W, ",DB-25 to the pod: VERIFY the shell against the cut-out"));
    echo(str("PANEL,rear_wall,", L, ",", H, ",rear wall (cable glands), seen from outside"));
    for (i = [0:len(glands) - 1])  // seen from outside the rear, X runs right-to-left
        echo(str("HOLE,rear_wall,", box_in[0] - gland_x(i) + W, ",", gland_z + W, ",", gland_d,
                 ",M16 gland: ", glands[i]));
    echo(str("PANEL,left_wall,", D, ",", H, ",left wall (vents by the BTS7960 heatsink), seen from outside"));
    n = 6;
    for (k = [0:n - 1])  // seen from outside the left wall, Y runs right-to-left
        echo(str("SLOT,left_wall,", box_in[1] - (vent_y[0] + k * (vent_y[1] - vent_y[0]) / (n - 1)) + W, ",",
                 vent_z + W, ",", vent_slot[0], ",", vent_slot[1], ",vent slot"));
    echo(str("PANEL,floor,", box_in[0], ",", box_in[1], ",box floor, seen from INSIDE with the front wall at the bottom (or mark through the printed chassis)"));
    for (p = floor_fix)
        echo(str("HOLE,floor,", plate_xy[0] + p[0], ",", plate_xy[1] + p[1], ",4.5,M4 to the chassis insert"));
}

if (part == "holes") holes_echo();
else if (part == "assembly") assembly();
else if (part == "chassis") chassis();
