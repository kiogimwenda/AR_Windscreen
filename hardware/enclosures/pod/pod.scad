// Windscreen pod: the enclosure behind the rear-view mirror for the pod board, the camera, the IMU
// and the GNSS module (BUILD_GUIDE 4.8.1, 4.8.2, 4.8.6). See hardware/enclosures/README.md.
//
// Print in ASA (light colour for the shell; black for the front plate and its hood); the glass
// zone passes 70 C. Parts:
//   rear_cover   the driver's side: status-LED windows, USB-C / gesture / DB-25 openings (open
//                slots, so it slides on over the connectors), RESET/BOOT pin-holes, top vents
//   front_shell  the rigid core: its CARRIER WALL holds the pod board (behind) and the camera (in
//                front), so camera and IMU share one rigid plate; the GNSS shelf; the pivot and
//                lock inserts; bottom vents
//   front_plate  black: the lens window and a short hood towards the glass (foam-sealed)
//   cradle       slides onto the rail and locks with a thumbscrew into the rail's insert (so the
//                pod comes off and returns to the same place); its two ears carry the pod's pitch pivot and
//                the lock screw in an arc slot (pitch the camera, lock it, mark it)
//   rail         screwed under the glass plate
//   glass_plate  3 mm aluminium, bonded to the glass with 3M VHB in the uncoated area (drilling
//                template, or have it cut)
// Frame (the pod level): x across the car (right +), y forward (the driver behind, y < 0), z up.
// The pod box spans x -W/2..W/2, y 0..D, z 0..H. The board's outline and holes come from
// ../common/boards.scad (generated with the KiCad board), so the standoffs follow the board.
//
//   part = "assembly" | "rear_cover" | "front_shell" | "front_plate" | "cradle" | "rail" | "holes"
//
// *** Measure the car's glass, the camera module and the GNSS board, and correct VERIFY lines ***

include <../common/boards.scad>

part = "assembly";

// --- The car and the bought parts (VERIFY) --------------------------------------------------------
glass_angle = 27;        // windscreen from horizontal (VERIFY on the car: 25-35 is usual)
glass_gap = 8;           // pod's top-front edge to the glass (pitch room: keep >= 6)
cam = [38, 38, 1.6];     // UVC board camera, M12 lens (BOM 2.6b; ELP 38 x 38 is common: VERIFY)
cam_holes = 34;          // its four M2 holes, square pitch (VERIFY)
lens_d = 14;             // M12 lens barrel (VERIFY)
lens_len = 22;           // lens holder + lens in front of the camera board (VERIFY)
gnss = [36, 26, 13];     // NEO-M8N board with its stacked patch antenna (BOM 2.2: VERIFY)
gnss_holes = [30, 0];    // its two M3 holes, along x (VERIFY; many boards have none: use a strap)

// --- The box ---------------------------------------------------------------------------------------
wall = 3;
board = pod_board;                     // [70, 53, 1.6] from boards.scad
W_in = board[0] + 3;                   // the board edge-to-edge, 1.5 mm each side
W = W_in + 2 * wall;
H = 75;                                // the board, the DB-25 under it, the GNSS shelf above
D = 52;
split = 26;                            // rear_cover y 0..split, front_shell split..D-3, front_plate D-3..D
board_bottom = wall + 0.5;             // the board's bottom edge (DB-25 flange) just above the floor
board_top = board_bottom + board[1];
carrier = [30, 35];                    // the carrier wall, y from..to
standoff = 5;
board_back = carrier[0] - standoff;    // the board's back face; its component side faces the driver
board_face = board_back - board[2];
cam_z = H - wall - 1 - cam[1] / 2;     // the camera as high as it goes (the lens near the glass)
cam_y = carrier[1] + 4;                // on 4 mm standoffs
pivot = [38, 60];                      // the pitch axis (y, z), along x through both side walls
lock_r = 22;                           // the lock screw's radius from the axis (arc slot in the ear)
pitch_range = 8;                       // +- degrees
insert_m3 = 4.0;
insert_m4 = 5.6;

// board (KiCad) -> pod: board x right = pod x right; board y down = pod z down
function bx(x) = -W_in / 2 + 1.5 + x;
function bz(y) = board_top - y;

// the glass plane, through the point glass_gap above the pod's top-front edge
glass_p = [0, D, H + glass_gap];
module on_glass() { translate(glass_p) rotate([-glass_angle, 0, 0]) children(); }
module below_glass(h = 200) {  // everything on the cabin side of the glass
    on_glass() translate([-150, -300, -h]) cube([300, 600, h]);
}

module rbox(p0, p1, r0 = 2) {  // a box with rounded vertical edges, from corner p0 to p1
    // (the radius shrinks to fit a thin box: a 3 mm plate with r = 2 would otherwise vanish)
    r = min(r0, (p1[0] - p0[0]) / 2 - 0.01, (p1[1] - p0[1]) / 2 - 0.01);
    translate([p0[0], p0[1], p0[2]]) linear_extrude(p1[2] - p0[2])
        translate([r, r]) offset(r) square([p1[0] - p0[0] - 2 * r, p1[1] - p0[1] - 2 * r]);
}

// --- rear cover --------------------------------------------------------------------------------------
// How the rear cover is held: the board fills the cover from wall to wall, so only the TOP two
// screws can run front-to-back (through tubes above the board, into inserts at the split); at the
// bottom, two tabs on the cover's floor lap under the front shell's floor (in a rebate) and take a
// screw each from below.
screw_x = W_in / 2 - 3;                  // all the cover and plate screws, x = +-screw_x
screw_top = H - wall - 3;
tab = [8, 8, 1.5];                        // the bottom lap tabs: x, y (past the split), z
module rear_cover() {
    difference() {
        union() {
            difference() {
                rbox([-W / 2, 0, 0], [W / 2, split, H]);
                rbox([-W_in / 2, wall, wall], [W_in / 2, split + 1, H - wall], 1);
            }
            for (sx = [-1, 1]) {
                translate([sx * screw_x, wall - 0.1, screw_top]) rotate([-90, 0, 0])
                    cylinder(d = 7, h = split - wall + 0.1, $fn = 24);
                translate([sx * (W_in / 2 - tab[0] / 2), split - 1 + tab[1] / 2, tab[2] / 2])
                    cube([tab[0], tab[1] + 2, tab[2]], center = true);
            }
        }
        // the top screws (M3 x 30, counterbored) and the tabs' screws (M3 x 8 from below)
        for (sx = [-1, 1]) {
            translate([sx * screw_x, -1, screw_top]) rotate([-90, 0, 0]) {
                cylinder(d = 3.4, h = split + 2, $fn = 16);
                cylinder(d = 6.5, h = 2.5, $fn = 24);
            }
            translate([sx * screw_x, split + tab[1] / 2, -1]) cylinder(d = 3.4, h = tab[2] + 2, $fn = 16);
        }
        // status LEDs D3-D6 (board top edge): windows in the rear wall
        for (x = [10.0, 14.2, 18.4, 22.6]) translate([bx(x), -1, bz(2.1)]) rotate([-90, 0, 0])
            cylinder(d = 2.6, h = wall + 2, $fn = 24);
        // their names, read downwards under each window (they are only 4.2 mm apart)
        for (i = [0 : 3]) translate([bx([10.0, 14.2, 18.4, 22.6][i]), 0.6, bz(2.1) - 2.5]) rotate([90, 0, 0])
            rotate([0, 0, -90]) linear_extrude(1) text(["PWR", "LINK", "ARM", "FAULT"][i], size = 1.8,
                                   halign = "left", valign = "center", font = "Liberation Sans:style=Bold");
        // USB-C (J1, right edge, board y 13.5) and the gesture cable (J4, board y 25.5): open slots
        // from the split, so the cover slides on over them
        translate([W_in / 2 - 1, board_face - 9, bz(13.5) - 3.75]) cube([wall + 2, split, 7.5]);
        translate([W_in / 2 - 1, board_face - 8, bz(25.5) - 3.5]) cube([wall + 2, split, 7]);
        // RESET / BOOT plungers (left edge, board y 24.2 and 30.0): pin-holes, 2 mm
        for (y = [24.2, 30.0]) translate([-W / 2 - 1, board_face - 1.0, bz(y)]) rotate([0, 90, 0])
            cylinder(d = 2.2, h = wall + 2, $fn = 16);
        translate([-W / 2 + 0.6, board_face - 6, bz(27.1)]) rotate([90, 0, -90])
            linear_extrude(1) text("RST BOOT", size = 1.8, halign = "center", valign = "center",
                                   font = "Liberation Sans:style=Bold");
        // DB-25 (J7, bottom edge, centred): an open slot under its shell, from the split
        translate([-21, board_face - 13, -1]) cube([42, split, wall + 2]);
        // top vents (the chimney's outlet)
        for (k = [-3 : 3]) translate([k * 8 - 2, 6, H - wall - 1]) cube([4, 14, wall + 2]);
    }
}

// --- front shell, with the carrier wall --------------------------------------------------------------
module front_shell() {
    difference() {
        union() {
            difference() {
                rbox([-W / 2, split, 0], [W / 2, D - wall, H]);
                rbox([-W_in / 2, split - 1, wall], [W_in / 2, D, H - wall], 1);
            }
            // the carrier wall: the rigid plate camera and board both bolt to
            translate([-W_in / 2, carrier[0], wall]) cube([W_in, carrier[1] - carrier[0], H - 2 * wall]);
            // the pod board's standoffs (its four holes), from the carrier back to the board
            for (h = pod_board_holes) translate([bx(h[0]), board_back, bz(h[1])]) rotate([-90, 0, 0])
                cylinder(d = 7, h = carrier[0] - board_back + 0.1, $fn = 32);
            // the camera's standoffs
            for (sx = [-1, 1], sz = [-1, 1]) translate([sx * cam_holes / 2, carrier[1] - 0.1, cam_z + sz * cam_holes / 2])
                rotate([-90, 0, 0]) cylinder(d = 5, h = cam_y - carrier[1] + 0.1, $fn = 24);
            // the GNSS shelf, above the board, behind the carrier
            translate([-gnss[0] / 2 - 4, wall + 1, board_top + 1.5]) cube([gnss[0] + 8, carrier[0] - wall - 1, 2.5]);
            // bosses: the rear cover's four screws (at the split) and the front plate's (at the front)
            for (sx = [-1, 1]) {
                translate([sx * screw_x, split, screw_top]) rotate([-90, 0, 0])
                    cylinder(d = 7, h = carrier[0] - split, $fn = 24);
                // the bottom tab's screw goes up into this block (the rebate is cut under it)
                translate([sx * (W_in / 2 - 4) - 4, split, wall - 0.1]) cube([8, carrier[0] - split, 7]);
            }
            for (sx = [-1, 1], sz = [-1, 1])
                translate([sx * screw_x, carrier[1], H / 2 + sz * (H / 2 - wall - 3)]) rotate([-90, 0, 0])
                    cylinder(d = 7, h = D - wall - carrier[1], $fn = 24);
            // pivot and lock bosses on the side walls (outside, flat faces for the ears)
            for (sx = [-1, 1]) for (z = [pivot[1], pivot[1] - lock_r])
                translate([sx * (W / 2 - 0.5), pivot[0], z]) rotate([0, sx * 90, 0]) cylinder(d = 10, h = 2.5, $fn = 32);
        }
        // inserts: board standoffs (M3), camera (M2), cover and plate screws (M3), pivot/lock (M4)
        for (h = pod_board_holes) translate([bx(h[0]), board_back - 0.1, bz(h[1])]) rotate([-90, 0, 0])
            cylinder(d = insert_m3, h = 6, $fn = 24);
        for (sx = [-1, 1], sz = [-1, 1]) translate([sx * cam_holes / 2, cam_y + 0.1, cam_z + sz * cam_holes / 2])
            rotate([90, 0, 0]) cylinder(d = 3.2, h = 5, $fn = 16);
        for (sx = [-1, 1]) {
            translate([sx * screw_x, split - 0.1, screw_top]) rotate([-90, 0, 0]) cylinder(d = insert_m3, h = 6, $fn = 24);
            // the rebate for the rear cover's tab (0.3 mm clearance) and the tab screw's insert
            translate([sx * (W_in / 2 - tab[0] / 2), split + (tab[1] + 0.3) / 2 - 0.5, tab[2] / 2 - 0.5])
                cube([tab[0] + 0.6, tab[1] + 1.3, tab[2] + 1.1], center = true);
            translate([sx * screw_x, split + tab[1] / 2, tab[2]]) cylinder(d = insert_m3, h = 6, $fn = 24);
        }
        for (sx = [-1, 1], sz = [-1, 1])
            translate([sx * screw_x, D - wall + 0.1, H / 2 + sz * (H / 2 - wall - 3)]) rotate([90, 0, 0])
                cylinder(d = insert_m3, h = 6, $fn = 24);
        for (sx = [-1, 1]) for (z = [pivot[1], pivot[1] - lock_r])
            translate([sx * (W / 2 + 2.1), pivot[0], z]) rotate([0, -sx * 90, 0]) cylinder(d = insert_m4, h = 8, $fn = 24);
        // GNSS: its lead to J3 (board top edge) and its two screws
        for (x = [-gnss_holes[0] / 2, gnss_holes[0] / 2]) translate([x, wall + 1 + gnss[1] / 2, board_top]) cylinder(d = insert_m3, h = 6, $fn = 24);
        // the pod board's cable passages through the carrier wall: none needed (all on the board's side)
        // bottom vents (the chimney's inlet), in front of the carrier
        for (k = [-3 : 3]) translate([k * 8 - 2, carrier[1] + 2, -1]) cube([4, D - wall - carrier[1] - 6, wall + 2]);
        // the camera's own USB lead: out through the floor, beside the vents
        translate([W_in / 2 - 10, carrier[1] + 3, -1]) cube([6.3, 9, wall + 2]);  // clear of the boss's axis
    }
}

// --- front plate and hood --------------------------------------------------------------------------
hood_len = 30;     // along the camera axis; cut off by the glass where it reaches it
hfov = 90;         // the lens's horizontal field (BOM: ~90 deg)
vfov = 55;
module front_plate() {
    difference() {
        union() {
            rbox([-W / 2, D - wall, 0], [W / 2, D, H]);
            // the hood: a tray below and beside the lens, splayed to stay out of the image, cut by
            // the glass (leave ~3 mm for a foam strip on its rim)
            intersection() {
                translate([0, D, cam_z]) hull() {
                    translate([0, 0, 0]) cube([lens_d + 8, 0.1, lens_d + 8], center = true);
                    translate([0, hood_len, 0])
                        cube([lens_d + 8 + 2 * hood_len * tan(hfov / 2 + 4), 0.1,
                              lens_d + 8 + 2 * hood_len * tan(vfov / 2 + 4)], center = true);
                }
                translate([0, 0, -3]) below_glass();
            }
        }
        // inside the hood: hollow, open towards the glass
        intersection() {
            translate([0, D - 0.01, cam_z]) hull() {
                cube([lens_d + 4, 0.1, lens_d + 4], center = true);
                translate([0, hood_len + 1, 0])
                    cube([lens_d + 4 + 2 * (hood_len + 1) * tan(hfov / 2 + 4), 0.1,
                          lens_d + 4 + 2 * (hood_len + 1) * tan(vfov / 2 + 4)], center = true);
            }
            translate([0, 0, 50]) below_glass(400);
        }
        // the lens window
        translate([0, D - wall - 1, cam_z]) rotate([-90, 0, 0]) cylinder(d = lens_d + 3, h = wall + 2, $fn = 48);
        // its four screws into the front shell
        for (sx = [-1, 1], sz = [-1, 1]) translate([sx * (W_in / 2 - 3), D - wall - 1, H / 2 + sz * (H / 2 - wall - 3)])
            rotate([-90, 0, 0]) { cylinder(d = 3.4, h = wall + 2, $fn = 16);
                                  translate([0, 0, wall - 1.6]) cylinder(d1 = 3.4, d2 = 6.6, h = 1.7, $fn = 16); }
    }
}

// --- the mount: glass plate, rail, cradle with its ears ----------------------------------------
plate = [50, 60, 3];         // aluminium: x, along the glass, thickness
plate_v = -40;               // its centre, along the glass from glass_p (towards the roof)
rail = [16, 54, 8];          // printed dovetail rail under the plate
dove = 3;                    // dovetail undercut each side
ear_t = 5;
ear_x = W / 2 + 2 + ear_t / 2;  // the ears' mid-planes, outside the pod's side walls
module glass_plate() {
    on_glass() difference() {
        translate([-plate[0] / 2, plate_v - plate[1] / 2, -plate[2]]) cube(plate);
        for (u = [-20, 20], x = [-0, 0]) translate([x, plate_v + u, -plate[2] - 1]) cylinder(d = 2.5, h = 5, $fn = 16);  // M3 tapped
    }
}
module rail_shape(grow = 0) {  // dovetail: wide at the bottom (away from the glass)
    polygon([[-rail[0] / 2 + dove - grow, 0], [rail[0] / 2 - dove + grow, 0],
             [rail[0] / 2 + grow, -rail[2] - grow], [-rail[0] / 2 - grow, -rail[2] - grow]]);
}
module rail() {
    on_glass() difference() {
        translate([0, plate_v + rail[1] / 2, -plate[2]]) rotate([90, 0, 0]) linear_extrude(rail[1]) rail_shape();
        for (u = [-20, 20]) translate([0, plate_v + u, -plate[2] - rail[2] - 1]) {
            cylinder(d = 3.4, h = rail[2] + 2, $fn = 16);
            cylinder(d = 6.5, h = rail[2] - 3, $fn = 24);  // counterbore: M3 x 6 into the plate's thread
        }
        // the thumbscrew's insert (M4) and the locating stop pin (3 mm) at the roof end
        translate([0, plate_v + rail[1] / 2 - 6, -plate[2] - rail[2] - 1]) cylinder(d = insert_m4, h = 7, $fn = 24);
    }
}
module cradle() {
    difference() {
        union() {
            // the saddle: under the rail, its top parallel to the glass, as wide as the ears
            on_glass() translate([-ear_x - ear_t / 2, plate_v - rail[1] / 2 + 6, -plate[2] - rail[2] - 8])
                cube([2 * ear_x + ear_t, rail[1] - 6, rail[2] + 8 - 0.5]);
            // the ears: straight side plates, from the saddle down to the pivot and the lock arc
            for (sx = [-1, 1]) hull() {
                on_glass() translate([sx * ear_x - ear_t / 2, plate_v - rail[1] / 2 + 6, -plate[2] - rail[2] - 8])
                    cube([ear_t, rail[1] - 6, 4]);
                translate([sx * ear_x - ear_t / 2, pivot[0] - 9, pivot[1] - lock_r - 7])
                    cube([ear_t, 18, lock_r + 14]);
            }
        }
        // the rail's dovetail groove, open at the bumper end so the cradle slides on
        on_glass() translate([0, plate_v + rail[1] / 2 + 1, -plate[2]]) rotate([90, 0, 0])
            linear_extrude(rail[1] + 2) rail_shape(0.3);
        // the pod: clearance through its whole pitch range
        for (a = [-pitch_range, 0, pitch_range]) translate([0, pivot[0], pivot[1]]) rotate([a, 0, 0])
            translate([0, -pivot[0], -pivot[1]]) rbox([-W / 2 - 1.5, -2, -2], [W / 2 + 1.5, D + 2, H + 1]);
        // the thumbscrew (M4, through the saddle into the rail's insert) and the stop pin
        on_glass() translate([0, plate_v + rail[1] / 2 - 6, -plate[2] - rail[2] - 20]) cylinder(d = 4.5, h = 30, $fn = 24);
        // pivot holes and the lock arc slot (M4)
        for (sx = [-1, 1]) {
            translate([sx * (W / 2 + 1), pivot[0], pivot[1]]) rotate([0, 90, 0]) cylinder(d = 4.5, h = 2 * ear_t + 4, center = true, $fn = 24);
            for (a = [-pitch_range : 1 : pitch_range]) hull() for (b = [a, min(a + 1, pitch_range)])
                translate([sx * (W / 2 + 1), pivot[0] + lock_r * sin(b), pivot[1] - lock_r * cos(b)])
                    rotate([0, 90, 0]) cylinder(d = 4.5, h = 2 * ear_t + 4, center = true, $fn = 16);
        }
    }
}

// --- the bought parts, for the picture -------------------------------------------------------------
module boards_model() {
    color("darkgreen") translate([bx(0), board_back - board[2], bz(board[1])]) cube([board[0], board[2], board[1]]);
    color("silver") translate([-20, board_face - 12.5, board_bottom - 3]) cube([40, 12.5, 12]);    // DB-25
    color("royalblue") translate([bx(29.3) - 2, board_face - 14, bz(30)]) cube([25.4, 1.6, 17.8]);  // IMU breakout
    color("dimgray") translate([-cam[0] / 2, cam_y, cam_z - cam[1] / 2]) cube([cam[0], cam[2], cam[1]]);
    color("black") translate([0, cam_y, cam_z]) rotate([-90, 0, 0]) cylinder(d = lens_d, h = lens_len, $fn = 48);
    color("tan") translate([-gnss[0] / 2, wall + 1 + (carrier[0] - wall - 1 - gnss[1]) / 2, board_top + 4]) cube(gnss);
}
module glass_model() {
    color("lightblue", 0.25) on_glass() translate([-80, -120, 0]) cube([160, 200, 5]);
}
module assembly() {
    color("whitesmoke") rear_cover();
    color("lightgray") front_shell();
    color([0.15, 0.15, 0.15]) front_plate();
    boards_model();
    color("gold") cradle();
    color("orange") rail();
    color("silver") glass_plate();
    glass_model();
}

module holes_echo() {
    // the glass plate, flat: its outline and the two M3 tapped holes (2.5 mm drill, then tap M3)
    echo(str("PANEL,glass_plate,", plate[0], ",", plate[1], ",aluminium 3 mm, bonded to the glass (VHB), seen from the cabin; roof end up"));
    for (u = [-20, 20]) echo(str("HOLE,glass_plate,", plate[0] / 2, ",", plate[1] / 2 + u, ",2.5,drill 2.5, tap M3 (rail screws)"));
}

echo(str("Pod box: ", W, " wide x ", D, " deep x ", H, " high mm; glass ", glass_angle, " deg, ", glass_gap, " mm above its top-front edge"));
if (part == "holes") holes_echo();
else if (part == "assembly") assembly();
else if (part == "rear_cover") rear_cover();
else if (part == "front_shell") front_shell();
else if (part == "front_plate") front_plate();
else if (part == "cradle") cradle();
else if (part == "rail") rail();
