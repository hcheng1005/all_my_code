#pragma once

#include <algorithm>
#include <iterator>
#include <limits>
#include <vector>

void Hungarian(int n, const double* edge, int* perm, double* lx, double* ly);

void HungarianMaximize(const std::vector<std::vector<double>>& similarities,
                       const double min_similarity,
                       std::vector<std::pair<uint32_t, uint32_t>>* associations,
                       std::vector<uint32_t>* unassociated_rows,
                       std::vector<uint32_t>* unassociated_cols);

void HungarianMinimize(const std::vector<std::vector<double>>& distance,
                       const double max_distance,
                       std::vector<std::pair<uint32_t, uint32_t>>* associations,
                       std::vector<uint32_t>* unassociated_rows,
                       std::vector<uint32_t>* unassociated_cols);