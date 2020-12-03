#pragma once

#include <vector>

namespace apollo {
namespace perception {

// bfs based component analysis
void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>* components);

}  // namespace perception
}  // namespace apollo

