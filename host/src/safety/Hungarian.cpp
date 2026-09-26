#include "ar_drive_assist/safety/Hungarian.h"

#include <limits>
#include <stdexcept>

namespace ar_drive_assist {

std::vector<int> hungarian(const std::vector<std::vector<double>>& cost) {
    const int rows = static_cast<int>(cost.size());
    if (rows == 0) return {};
    const int cols = static_cast<int>(cost[0].size());
    for (const auto& r : cost) {
        if (static_cast<int>(r.size()) != cols) throw std::invalid_argument("hungarian: ragged");
    }
    if (cols == 0) return std::vector<int>(rows, -1);

    // The algorithm below assigns every row of an n x m matrix with n <= m. When there are more
    // rows than columns, solve the transpose and invert the result.
    const bool transposed = rows > cols;
    const int n = transposed ? cols : rows, m = transposed ? rows : cols;
    auto c = [&](int i, int j) { return transposed ? cost[j - 1][i - 1] : cost[i - 1][j - 1]; };

    // 1-based arrays, following the classic formulation: u, v are row/column potentials; p[j]
    // is the row matched to column j (0 = none); way[j] is the previous column on the
    // augmenting path.
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);
    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(m + 1, INF);
        std::vector<char> used(m + 1, 0);
        do {
            used[j0] = 1;
            const int i0 = p[j0];
            double delta = INF;
            int j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (used[j]) continue;
                const double cur = c(i0, j) - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {  // flip the augmenting path
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> result(rows, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] == 0) continue;
        if (transposed) {
            result[j - 1] = p[j] - 1;  // original row j-1 <- original column p[j]-1
        } else {
            result[p[j] - 1] = j - 1;
        }
    }
    return result;
}

}  // namespace ar_drive_assist
