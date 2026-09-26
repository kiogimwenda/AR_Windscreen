#!/usr/bin/env bash
# refresh_osm.sh — builds the offline routing data for Nairobi and everything within 100 km of its
# CBD. See docs/BUILD_GUIDE.md Part 11.2 and 11.2.1 (layer 1: refresh, don't freeze).
#
#   host/scripts/refresh_osm.sh              download today's Kenya file, build, test, switch
#   host/scripts/refresh_osm.sh --no-download   rebuild from the Kenya file already on disk
#
# Steps, each of which must succeed before the next:
#   1. Download Geofabrik's daily Kenya extract and verify its published MD5.
#   2. Clip it to the 100 km box with osmium (complete_ways: a road crossing the edge is kept
#      whole, so routes near the boundary do not end at a cut).
#   3. osrm-extract (car profile) -> osrm-partition -> osrm-customize: the MLD pipeline. MLD, not
#      CH, because the live layer (11.2.1 layer 2) and local closures re-weight the graph with
#      osrm-customize, which only MLD supports.
#   4. Smoke test: route Nairobi CBD -> Thika with osrm-routed on the NEW data. It must succeed
#      with a plausible length.
#   5. Only then switch data/maps/current to the new build (an atomic rename of a symlink). A
#      failure anywhere leaves the previous map in service.
# The two newest builds are kept; older ones are deleted.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
MAPS="$ROOT/data/maps"
BBOX="35.92,-2.19,37.72,-0.38"  # lon/lat box enclosing 100 km around 1.2864 S, 36.8172 E
URL="https://download.geofabrik.de/africa/kenya-latest.osm.pbf"
PROFILE="${OSRM_PROFILE:-/usr/local/share/osrm/profiles/car.lua}"
PORT="${SMOKE_PORT:-5099}"

log() { echo "== $(date '+%F %T') $*"; }
mkdir -p "$MAPS"
cd "$MAPS"

if [[ "${1:-}" != "--no-download" ]]; then
    log "downloading $URL"
    curl -sSfL -o kenya-latest.osm.pbf.part "$URL"
    curl -sSfL -o kenya-latest.osm.pbf.md5 "$URL.md5"
    want="$(cut -d' ' -f1 kenya-latest.osm.pbf.md5)"
    have="$(md5sum kenya-latest.osm.pbf.part | cut -d' ' -f1)"
    [[ "$want" == "$have" ]] || { log "MD5 MISMATCH ($have != $want); keeping previous map"; exit 1; }
    mv kenya-latest.osm.pbf.part kenya-latest.osm.pbf
fi
[[ -f kenya-latest.osm.pbf ]] || { log "no kenya-latest.osm.pbf; run without --no-download"; exit 1; }

NEW="nairobi-$(date +%Y%m%d-%H%M%S)"
mkdir "$NEW"
trap 'log "FAILED; previous map left in service"; rm -rf "$MAPS/$NEW"' ERR

log "clipping to $BBOX"
osmium extract --bbox "$BBOX" --strategy complete_ways --overwrite \
    -o "$NEW/nairobi.osm.pbf" kenya-latest.osm.pbf

log "osrm-extract / partition / customize (MLD)"
osrm-extract -p "$PROFILE" "$NEW/nairobi.osm.pbf" > "$NEW/build.log" 2>&1
osrm-partition "$NEW/nairobi.osrm" >> "$NEW/build.log" 2>&1
osrm-customize "$NEW/nairobi.osrm" >> "$NEW/build.log" 2>&1
rm -f "$NEW/nairobi.osm.pbf"  # the .osrm files are all routing needs

log "smoke test: Nairobi CBD -> Thika"
osrm-routed --algorithm mld --port "$PORT" "$NEW/nairobi.osrm" > "$NEW/smoke.log" 2>&1 &
pid=$!
ok=""
for _ in $(seq 60); do
    if r="$(curl -sf "http://127.0.0.1:$PORT/route/v1/driving/36.8252,-1.2864;37.0693,-1.0333?overview=false")"; then
        ok="$r"
        break
    fi
    sleep 1
done
kill "$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true
km="$(python3 -c 'import json,sys; r=json.loads(sys.argv[1]); print(round(r["routes"][0]["distance"]/1000,1) if r.get("code")=="Ok" else -1)' "${ok:-{\}}")"
# Plausible: the road distance is about 45 km (straight line ~39 km). Outside 35-70 km, something
# is wrong with the data even though OSRM answered.
python3 -c "import sys; sys.exit(0 if 35 <= float('$km') <= 70 else 1)" \
    || { log "smoke test failed (route km: $km)"; false; }
log "smoke test ok: $km km"

cat > "$NEW/BUILD_INFO" <<EOF
built:        $(date -Is)
source:       $URL
source_date:  $(osmium fileinfo -g header.option.osmosis_replication_timestamp kenya-latest.osm.pbf 2>/dev/null || echo unknown)
source_md5:   $(md5sum kenya-latest.osm.pbf | cut -d' ' -f1)
bbox:         $BBOX
profile:      $PROFILE
osrm:         $(osrm-extract --version 2>/dev/null | head -1)
smoke_km:     $km
EOF

trap - ERR
ln -sfn "$NEW" current.tmp && mv -Tf current.tmp current
log "switched data/maps/current -> $NEW"

ls -1d nairobi-* | sort | head -n -2 | xargs -r rm -rf
