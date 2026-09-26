#pragma once
// OsrmEngine — private helpers shared by NavigationEngine and MapMatcher's OSRM backend: opening
// the routing data and reading OSRM's JSON results. Kept out of the public headers so that
// nothing outside src/nav/ (and no unit test) needs the OSRM headers.

#include <memory>
#include <osrm/engine_config.hpp>
#include <osrm/json_container.hpp>
#include <osrm/osrm.hpp>
#include <stdexcept>
#include <string>
#include <string_view>

#include "ar_drive_assist/common/Geo.h"

namespace ar_drive_assist::osrm_detail {

// Opens `<dir>/nairobi.osrm` data (or any `.osrm` base path) with the MLD algorithm, from files.
// MLD matches how scripts/refresh_osm.sh builds the data (Part 11.2: only MLD can be re-weighted).
inline std::shared_ptr<osrm::OSRM> open(const std::string& osrmBasePath) {
    osrm::EngineConfig cfg;
    cfg.storage_config = osrm::storage::StorageConfig(osrmBasePath);
    cfg.use_shared_memory = false;
    cfg.algorithm = osrm::EngineConfig::Algorithm::MLD;
    if (!cfg.IsValid())
        throw std::runtime_error("OSRM: incomplete routing data at " + osrmBasePath);
    return std::make_shared<osrm::OSRM>(cfg);
}

// Typed access into OSRM's JSON. Each throws std::runtime_error naming the missing key, so a
// changed OSRM response format fails loudly instead of producing an empty route.
inline const osrm::json::Value& at(const osrm::json::Object& o, std::string_view key) {
    const auto it = o.values.find(key);
    if (it == o.values.end())
        throw std::runtime_error("OSRM JSON: missing '" + std::string(key) + "'");
    return it->second;
}
inline bool has(const osrm::json::Object& o, std::string_view key) {
    return o.values.find(key) != o.values.end();
}
inline const osrm::json::Object& obj(const osrm::json::Value& v) {
    return std::get<osrm::json::Object>(v);
}
inline const osrm::json::Array& arr(const osrm::json::Value& v) {
    return std::get<osrm::json::Array>(v);
}
inline double num(const osrm::json::Value& v) {
    return std::get<osrm::json::Number>(v).value;
}
inline std::string str(const osrm::json::Value& v) {
    return std::get<osrm::json::String>(v).value;
}
inline GeoPoint lonLat(const osrm::json::Value& v) {  // OSRM's [lon, lat] pair
    const auto& a = arr(v).values;
    return {num(a.at(1)), num(a.at(0))};
}
inline osrm::util::Coordinate coord(const GeoPoint& p) {
    return {osrm::util::FloatLongitude{p.lon}, osrm::util::FloatLatitude{p.lat}};
}

}  // namespace ar_drive_assist::osrm_detail
