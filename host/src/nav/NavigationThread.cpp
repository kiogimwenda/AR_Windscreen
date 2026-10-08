// NavigationThread — see include/ar_drive_assist/nav/NavigationThread.h.
#include "ar_drive_assist/nav/NavigationThread.h"

#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>

#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {

NavigationThread::NavigationThread(NavigationThreadConfig cfg, EgoEstimator& ego, SceneBus* scenes,
                                   NavBus& out, EventLog* log)
    : cfg_(std::move(cfg)), ego_(ego), scenes_(scenes), out_(out), log_(log) {
    if (!nav_.init(cfg_.osrmData))
        throw std::runtime_error("navigation: cannot load " + cfg_.osrmData + ": " +
                                 nav_.lastError());
    if (!matcher_.init(cfg_.osrmData))
        throw std::runtime_error("navigation: map matcher cannot load " + cfg_.osrmData);
    projector_.init(cfg_.projection);
}

void NavigationThread::run(const std::atomic<bool>& stop) {
    using Clock = std::chrono::steady_clock;
    GroundPlaneModel road;  // flat until the first fitted surface arrives with a scene (Part 8.1)
    Route route;
    bool haveRoute = false;
    DetectionFrame lanes;
    SceneSnapshot scene;
    auto next = Clock::now();
    while (!stop.load()) {
        std::this_thread::sleep_until(next);
        next += std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(cfg_.periodS));
        if (scenes_ && scenes_->popLatest(scene)) {
            lanes = scene.detections;
            if (scene.groundValid) road = scene.ground;
        }

        const std::uint64_t now = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch())
                .count());
        const EgoEstimator::Snapshot ego = ego_.snapshot(now);
        if (!ego.valid) continue;  // no GNSS fix yet

        if (!haveRoute) {
            route = nav_.routeTo(cfg_.destLat, cfg_.destLon, ego.pose, ego.frame);
            if (!route.valid) {
                if (log_) log_->logGeneral("navigation: no route: " + nav_.lastError());
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            matcher_.reset();
            projector_.reset();
            haveRoute = true;
            if (log_) {
                char b[120];
                std::snprintf(b, sizeof b, "navigation: route %.0f m, %zu steps", route.lengthM(),
                              route.steps.size());
                log_->logGeneral(b);
            }
        }
        const MapMatcher::MatchedPosition matched = matcher_.match(ego.pose, ego.frame, route);
        if (matched.valid && !matched.onRoute) {
            if (log_) log_->logGeneral("navigation: off the route, rerouting");
            haveRoute = false;
            continue;
        }
        out_.push(projector_.project(route, matched, road, lanes, ego.pose, ego.frame, cfg_.camera,
                                     cfg_.cameraFromVehicle));
    }
}

}  // namespace ar_drive_assist
