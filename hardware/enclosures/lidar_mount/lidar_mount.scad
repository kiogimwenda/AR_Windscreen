// LiDAR roof mount for the Livox Mid-360, tilted 15 degrees forward. See BUILD_GUIDE Part 4.8.6
// and hardware/enclosures/README.md.
//
// Three parts, stacked: base plate (3 mm aluminium, on four rubber-coated pot magnets) ->
// printed ASA wedge (15 degrees) -> top plate (3 mm aluminium: the LiDAR's heatsink, as the Livox
// manual requires: >= 3 mm thick, >= 10,000 mm^2 exposed). The LiDAR's connector faces the rear.
//
// Frame: X forward (car), Y left, Z up; origin at the centre of the base plate's top face.
//
// Render one part:  openscad -D 'part="wedge"' -o wedge.stl lidar_mount.scad
//   part = "assembly" | "wedge" | "top_plate_2d" | "base_plate_2d"
//
// Livox Mid-360 (user manual v1.2, Appendix: dimensions): 65 x 65 x 60 mm; bottom face 4 x M3,
// 5 mm deep, on 48 (along the connector axis) x 36; locating holes 3.0 mm (+0.1), 1.8 deep: a round
// one 16 mm and a slot 23 mm either side of centre, on the line across the connector axis; optical
// centre 47 mm above the bottom face; connector 14.3 mm up, protruding 8 mm.

part = "assembly";

tilt = 15;            // degrees, forward (Part 4.8.6: road visible from ~4-5 m)
plate_t = 3;          // aluminium plates
top_size = 130;       // top plate: 130^2 - 65^2 = 12,675 mm^2 exposed on top
base_size = 150;
wedge_size = 120;
wedge_min = 12;       // wedge thickness at its front edge (room for a 9 mm insert)
corner_r = 8;

// LiDAR (in the top plate's frame: X along the plate's forward slope, connector towards -X)
lidar_body = 65;
lidar_h = 60;
lidar_m3 = [[24, 18], [24, -18], [-24, 18], [-24, -18]];
dowel_round = [0, -16];   // 3.0 mm, press-fit 3 x 5 mm dowel pin
dowel_slot = [0, 23];     // 3.0 mm wide slot along Y (the manual's slot), dowel pin in a 3.0 hole
optical_centre_h = 47;

// Fixings
m4_clear = 4.5;
insert_d = 5.6;           // M4 x 8.1 brass heat-set insert: check the insert's datasheet
insert_depth = 9;
top_inserts = [[48, 48], [48, -48], [-48, 48], [-48, -48]];     // top plate -> wedge
base_inserts = [[35, 35], [35, -35], [-35, 35], [-35, -35]];    // base plate -> wedge (from below)
magnet_holes = [[60, 60], [60, -60], [-60, 60], [-60, -60]];    // M6 studs of D43 pot magnets
magnet_d = 43;
tether_hole = [-62, 0];
cable_slots = [[-50, 40], [-50, -40]];

$fn = 48;

function wedge_h(x) = wedge_min + (wedge_size / 2 - x) * tan(tilt);  // thicker at the rear

module rounded_square(s, r) {
    offset(r) square(s - 2 * r, center = true);
}

// --- Top plate ------------------------------------------------------------------------------
module top_plate_2d() {
    difference() {
        rounded_square(top_size, corner_r);
        for (p = lidar_m3) translate(p) circle(d = 3.4);
        translate(dowel_round) circle(d = 3.0);
        translate(dowel_slot) circle(d = 3.0);
        for (p = top_inserts) translate(p) circle(d = m4_clear);
    }
}

// --- Wedge (printed) ------------------------------------------------------------------------
module wedge() {
    s = wedge_size / 2;
    difference() {
        // the wedge: a block cut by the sloped plane z = wedge_h(x)
        intersection() {
            translate([-s, -s, 0]) cube([wedge_size, wedge_size, wedge_h(-s) + 1]);
            translate([s, 0, wedge_min]) rotate([0, tilt, 0]) translate([-500, -500, -1000]) cube(1000);
        }
        // top inserts, perpendicular to the sloped face
        for (p = top_inserts) on_slope(p) translate([0, 0, -insert_depth]) cylinder(d = insert_d, h = insert_depth + 1);
        // pockets for the M3 screw heads that hold the LiDAR (screws go in from below the plate)
        for (p = lidar_m3) on_slope(p) translate([0, 0, -4]) cylinder(d = 7, h = 5);
        // base inserts, from below
        for (p = base_inserts) translate([p[0], p[1], -1]) cylinder(d = insert_d, h = insert_depth + 1);
        // lightening: a pocket under the middle, away from every fixing
        translate([0, 0, -1]) linear_extrude(wedge_min - 4) offset(5) square([40, 30], center = true);
    }
}

// places children on the wedge's sloped top face at plate coordinates p, aligned with it
module on_slope(p) {
    x = p[0] * cos(tilt);  // plate coordinates measured along the slope
    translate([x, p[1], wedge_h(x)]) rotate([0, tilt, 0]) children();
}

// --- Base plate -----------------------------------------------------------------------------
module base_plate_2d() {
    difference() {
        rounded_square(base_size, corner_r + 2);
        for (p = base_inserts) translate(p) circle(d = m4_clear);
        for (p = magnet_holes) translate(p) circle(d = 6.5);
        translate(tether_hole) circle(d = 6.5);
        for (p = cable_slots) translate(p) square([4, 10], center = true);
    }
}

// --- Assembly (for checking only) ---------------------------------------------------------
module lidar() {
    color("dimgray") translate([0, 0, lidar_h / 2]) cube([lidar_body, lidar_body, lidar_h], center = true);
    color("silver") translate([-lidar_body / 2 - 8, 0, 14.3]) rotate([0, 90, 0]) cylinder(d = 12, h = 8);  // M12 connector, rear
    color("red") translate([0, 0, optical_centre_h]) sphere(d = 4);  // optical centre
}

module assembly() {
    for (p = magnet_holes) color("black") translate([p[0], p[1], -plate_t - 9]) cylinder(d = magnet_d, h = 9);
    color("lightsteelblue") translate([0, 0, -plate_t]) linear_extrude(plate_t) base_plate_2d();
    color("orange") wedge();
    on_slope([0, 0]) {
        color("lightsteelblue") linear_extrude(plate_t) top_plate_2d();
        translate([0, 0, plate_t]) lidar();
    }
}

// Hole lists for the drilling templates (hardware/enclosures/make_templates.py): one ECHO line per
// hole, "plate,x,y,diameter,label", read by the script, so the templates come from this file.
module holes_echo() {
    for (p = lidar_m3) echo(str("HOLE,top_plate,", p[0], ",", p[1], ",3.4,M3 clear (LiDAR, from below)"));
    echo(str("HOLE,top_plate,", dowel_round[0], ",", dowel_round[1], ",3.0,dowel 3x5 press (check against the LiDAR first)"));
    echo(str("HOLE,top_plate,", dowel_slot[0], ",", dowel_slot[1], ",3.0,dowel 3x5 press (check against the LiDAR first)"));
    for (p = top_inserts) echo(str("HOLE,top_plate,", p[0], ",", p[1], ",4.5,M4 clear (to wedge inserts)"));
    echo(str("OUTLINE,top_plate,", top_size, ",", corner_r));
    for (p = base_inserts) echo(str("HOLE,base_plate,", p[0], ",", p[1], ",4.5,M4 clear (to wedge inserts)"));
    for (p = magnet_holes) echo(str("HOLE,base_plate,", p[0], ",", p[1], ",6.5,M6 (pot magnet stud)"));
    echo(str("HOLE,base_plate,", tether_hole[0], ",", tether_hole[1], ",6.5,tether"));
    for (p = cable_slots) echo(str("SLOT,base_plate,", p[0], ",", p[1], ",4,10,cable-tie slot"));
    echo(str("OUTLINE,base_plate,", base_size, ",", corner_r + 2));
}

if (part == "holes") holes_echo();
else if (part == "assembly") assembly();
else if (part == "wedge") wedge();
else if (part == "top_plate_2d") top_plate_2d();
else if (part == "base_plate_2d") base_plate_2d();
