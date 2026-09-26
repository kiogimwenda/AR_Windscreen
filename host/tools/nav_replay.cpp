// nav_replay — replays a recorded GPS track through NavigationEngine + MapMatcher on the offline
// map, and draws the result. This is Phase 8's exit check (docs/BUILD_GUIDE.md Part 15): "a
// recorded or live GPS/EKF track fed through MapMatcher visibly snaps onto the correct road
// edge, with distanceAlongRouteM increasing monotonically".
//
//   build/host/nav_replay --gpx drive.gpx [--map data/maps/current/nairobi.osrm]
//                         [--dest LAT,LON] [--out replay.html]
//
// Any phone GPS-logger app can record the GPX (trkpt lat/lon with <time>). The route runs from
// the first fix to --dest (default: the last fix). Speed and heading come from successive fixes,
// the way SensorFusion would supply them. Output:
//   - a summary: fixes, matched, off-route, and whether progress ever decreased (it must not);
//   - an HTML map (Leaflet + OpenStreetMap tiles, needs internet to show the tiles): the route in
//     blue, raw fixes in red, matched positions in green, each raw fix joined to its match.

#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "ar_drive_assist/nav/MapMatcher.h"
#include "ar_drive_assist/nav/NavigationEngine.h"

using namespace ar_drive_assist;

namespace {

struct Fix {
    GeoPoint p;
    std::uint64_t ms = 0;
};

std::uint64_t parseIsoUtcMs(const std::string& s) {
    std::tm tm{};
    double frac = 0;
    if (std::sscanf(s.c_str(), "%d-%d-%dT%d:%d:%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
                    &tm.tm_hour, &tm.tm_min, &tm.tm_sec) != 6)
        return 0;
    const auto dot = s.find('.');
    if (dot != std::string::npos)
        frac = std::atof(
            ("0" + s.substr(dot, s.find_first_not_of("0123456789", dot + 1) - dot)).c_str());
    tm.tm_year -= 1900;
    tm.tm_mon -= 1;
    return static_cast<std::uint64_t>(timegm(&tm)) * 1000 + static_cast<std::uint64_t>(frac * 1000);
}

std::vector<Fix> readGpx(const std::string& path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string x = ss.str();
    static const std::regex pt(
        R"re(<trkpt[^>]*?lat="([-0-9.]+)"[^>]*?lon="([-0-9.]+)"[^>]*>([\s\S]*?)</trkpt>)re");
    static const std::regex ptLonFirst(
        R"re(<trkpt[^>]*?lon="([-0-9.]+)"[^>]*?lat="([-0-9.]+)"[^>]*>([\s\S]*?)</trkpt>)re");
    static const std::regex time(R"re(<time>([^<]+)</time>)re");
    std::vector<Fix> out;
    auto scan = [&](const std::regex& re, bool lonFirst) {
        for (auto it = std::sregex_iterator(x.begin(), x.end(), re); it != std::sregex_iterator();
             ++it) {
            Fix fx;
            fx.p.lat = std::stod((*it)[lonFirst ? 2 : 1]);
            fx.p.lon = std::stod((*it)[lonFirst ? 1 : 2]);
            std::smatch tm;
            const std::string body = (*it)[3];
            if (std::regex_search(body, tm, time)) fx.ms = parseIsoUtcMs(tm[1]);
            out.push_back(fx);
        }
    };
    scan(pt, false);
    if (out.empty()) scan(ptLonFirst, true);
    return out;
}

std::string latLngs(const std::vector<GeoPoint>& ps) {
    std::ostringstream s;
    s.precision(8);
    s << '[';
    for (std::size_t i = 0; i < ps.size(); ++i)
        s << (i ? "," : "") << '[' << ps[i].lat << ',' << ps[i].lon << ']';
    s << ']';
    return s.str();
}

}  // namespace

int main(int argc, char** argv) {
    std::string gpx, map = "data/maps/current/nairobi.osrm", out = "nav_replay.html";
    bool haveDest = false;
    GeoPoint dest;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--gpx")
            gpx = next();
        else if (a == "--map")
            map = next();
        else if (a == "--out")
            out = next();
        else if (a == "--dest") {
            haveDest = std::sscanf(next().c_str(), "%lf,%lf", &dest.lat, &dest.lon) == 2;
        } else {
            std::cerr << "unknown argument " << a << "\n";
            return 2;
        }
    }
    const std::vector<Fix> fixes = readGpx(gpx);
    if (fixes.size() < 2) {
        std::cerr << "need a GPX track with at least 2 <trkpt> fixes: " << gpx << "\n";
        return 2;
    }
    for (const Fix& f : fixes) {
        if (f.ms == 0) {
            std::cerr << "every fix needs a <time> (speed and matching timestamps come from it)\n";
            return 2;
        }
    }
    if (!haveDest) dest = fixes.back().p;

    NavigationEngine nav;
    if (!nav.init(map)) {
        std::cerr << "map: " << nav.lastError() << "\n";
        return 1;
    }
    const Route route = nav.route(fixes.front().p, dest);
    if (!route.valid) {
        std::cerr << "route: " << nav.lastError() << "\n";
        return 1;
    }
    MapMatcher mm;
    if (!mm.init(map)) {
        std::cerr << "MapMatcher: cannot open " << map << "\n";
        return 1;
    }

    const LocalFrame frame(fixes.front().p);  // as SensorFusion: origin at the first fix
    std::vector<GeoPoint> raw, matched;
    std::ostringstream links;
    links.precision(8);
    double prevAlong = -1, heading = 0;
    int nMatched = 0, nOff = 0, decreases = 0;
    for (std::size_t i = 0; i < fixes.size(); ++i) {
        double e, n;
        frame.toLocal(fixes[i].p, e, n);
        double speed = 0;
        if (i > 0) {
            const double d = geo::distanceM(fixes[i - 1].p, fixes[i].p);
            const double dt = (fixes[i].ms - fixes[i - 1].ms) / 1000.0;
            if (dt > 0) speed = d / dt;
            if (d > 1.0) heading = geo::headingDeg(fixes[i - 1].p, fixes[i].p);
        }
        VehiclePose pose;
        pose.timestamp_ms = fixes[i].ms;
        pose.x = e;
        pose.y = n;
        pose.heading_deg = static_cast<float>(heading);
        pose.speed_kph = static_cast<float>(speed * 3.6);
        const auto m = mm.match(pose, frame, route);
        raw.push_back(fixes[i].p);
        if (!m.valid) continue;
        ++nMatched;
        nOff += !m.onRoute;
        if (m.distanceAlongRouteM < prevAlong) ++decreases;
        prevAlong = m.distanceAlongRouteM;
        matched.push_back({m.latitude, m.longitude});
        links << (links.tellp() > 0 ? "," : "") << "[[" << fixes[i].p.lat << ',' << fixes[i].p.lon
              << "],[" << m.latitude << ',' << m.longitude << "]]";
    }

    std::printf("route: %.2f km, %zu steps\n", route.lengthM() / 1000, route.steps.size());
    std::printf(
        "fixes: %zu, matched: %d, off-route: %d, progress decreases: %d, final progress: "
        "%.0f m of %.0f m\n",
        fixes.size(), nMatched, nOff, decreases, prevAlong, route.lengthM());

    std::ofstream html(out);
    html << R"(<!doctype html><html><head><meta charset="utf-8"><title>Nav replay</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>html,body,#m{height:100%;margin:0} .k{position:absolute;z-index:999;top:8px;right:8px;
background:#fff;padding:6px 10px;font:13px sans-serif;border-radius:4px}</style></head><body>
<div id="m"></div><div class="k"><b style="color:#2563eb">route</b> &middot;
<b style="color:#dc2626">raw fixes</b> &middot; <b style="color:#16a34a">matched</b></div><script>
const m=L.map('m');L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,
attribution:'&copy; OpenStreetMap contributors'}).addTo(m);
const route=)"
         << latLngs(route.geometry) << ";\nconst raw=" << latLngs(raw)
         << ";\nconst matched=" << latLngs(matched) << ";\nconst links=[" << links.str() << R"(];
const r=L.polyline(route,{color:'#2563eb',weight:6,opacity:.6}).addTo(m);
links.forEach(l=>L.polyline(l,{color:'#888',weight:1}).addTo(m));
raw.forEach(p=>L.circleMarker(p,{radius:3,color:'#dc2626'}).addTo(m));
matched.forEach(p=>L.circleMarker(p,{radius:3,color:'#16a34a'}).addTo(m));
m.fitBounds(r.getBounds());</script></body></html>
)";
    std::printf("map written to %s\n", out.c_str());
    return decreases == 0 ? 0 : 1;
}
