#pragma once
// Optimal assignment (the Hungarian / Kuhn-Munkres algorithm). See docs/BUILD_GUIDE.md Part 9.1.
//
// Given a cost matrix cost[i][j] (track i, measurement j), find the one-to-one pairing with the
// smallest total cost. Rows and columns may differ in number; the smaller side is fully
// assigned.
//
// Why not greedy (repeatedly take the cheapest remaining pair)? Two pedestrians walking side by
// side are the classic case: greedy can give pedestrian A's measurement to B's track because it
// happened to be marginally cheaper, forcing A's track onto a far worse match or losing it. The
// optimal assignment minimises the total and swaps identities less often. That matters here,
// because a track's history (velocity, risk) must belong to the right person.
//
// This is the O(n^3) shortest-augmenting-path formulation with row/column potentials (the
// Jonker-Volgenant style of Kuhn-Munkres). With tens of objects per frame it runs in microseconds.

#include <vector>

namespace ar_drive_assist {

// Returns, for each row, the assigned column or -1 (only when rows > columns). Costs must be
// finite. Forbidden pairs (outside the gate) should carry a large cost and be rejected by the
// caller after assignment.
std::vector<int> hungarian(const std::vector<std::vector<double>>& cost);

}  // namespace ar_drive_assist
