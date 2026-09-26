// Hungarian assignment tests. The decisive one compares against brute force (every permutation)
// on many random matrices of every shape up to 6x6. For small sizes brute force is exactly
// optimal, so any disagreement in total cost is a bug.

#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

#include "ar_drive_assist/safety/Hungarian.h"

using ar_drive_assist::hungarian;
using Matrix = std::vector<std::vector<double>>;

namespace {

double total(const Matrix& c, const std::vector<int>& a) {
    double t = 0;
    for (size_t i = 0; i < a.size(); ++i)
        if (a[i] >= 0) t += c[i][a[i]];
    return t;
}

// Minimum total cost over all one-to-one assignments covering min(rows, cols) pairs.
double bruteForce(const Matrix& c) {
    const int r = int(c.size()), k = int(c[0].size());
    const int n = std::max(r, k);
    std::vector<int> perm(n);
    std::iota(perm.begin(), perm.end(), 0);
    double best = 1e300;
    do {
        double t = 0;
        for (int i = 0; i < r; ++i)
            if (perm[i] < k) t += c[i][perm[i]];
        best = std::min(best, t);
    } while (std::next_permutation(perm.begin(), perm.end()));
    return best;
}

}  // namespace

TEST(Hungarian, TwoPedestriansGreedyGetsWrong) {
    // Greedy takes the single cheapest pair (0,0 = 1.0) and is then forced into (1,1 = 10): total
    // 11. The optimal pairing (0,1) + (1,0) costs 2 + 2 = 4.
    const Matrix c = {{1.0, 2.0}, {2.0, 10.0}};
    const auto a = hungarian(c);
    EXPECT_EQ(a, (std::vector<int>{1, 0}));
    EXPECT_DOUBLE_EQ(total(c, a), 4.0);
}

TEST(Hungarian, MatchesBruteForceOnRandomMatricesOfEveryShape) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> u(0.0, 100.0);
    for (int r = 1; r <= 6; ++r) {
        for (int k = 1; k <= 6; ++k) {
            for (int trial = 0; trial < 40; ++trial) {
                Matrix c(r, std::vector<double>(k));
                for (auto& row : c)
                    for (auto& x : row) x = u(rng);
                const auto a = hungarian(c);
                // Valid: one-to-one, and exactly min(r, k) pairs.
                std::vector<int> seen;
                for (int x : a)
                    if (x >= 0) seen.push_back(x);
                std::sort(seen.begin(), seen.end());
                ASSERT_EQ(std::adjacent_find(seen.begin(), seen.end()), seen.end())
                    << r << "x" << k;
                ASSERT_EQ(int(seen.size()), std::min(r, k)) << r << "x" << k;
                ASSERT_NEAR(total(c, a), bruteForce(c), 1e-9)
                    << r << "x" << k << " trial " << trial;
            }
        }
    }
}

TEST(Hungarian, EmptyInputs) {
    EXPECT_TRUE(hungarian({}).empty());
    const Matrix noCols = {{}, {}};
    EXPECT_EQ(hungarian(noCols), (std::vector<int>{-1, -1}));
}
