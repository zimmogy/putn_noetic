/*
 * FeatureExtractor.cpp
 *
 *  Modified by: Haoran Wang
 *  Revision date: 2026-08-12
 */

#include "lesta/core/FeatureExtractor.h"
#include <Eigen/Dense>

namespace lesta {

FeatureExtractor::FeatureExtractor(const Config &cfg) : cfg(cfg) {}

void FeatureExtractor::ensureFeatureLayers(HeightMap &map) {
  // Basic feature layers
  map.addLayer(layers::Feature::STEP);
  map.addLayer(layers::Feature::SLOPE);
  map.addLayer(layers::Feature::ROUGHNESS);
  map.addLayer(layers::Feature::CURVATURE);
  
  // Extended feature layers.
  map.addLayer(layers::Feature::VARIANCE);
  map.addLayer(layers::Feature::INTENSITY_MEAN);
  map.addLayer(layers::Feature::INTENSITY_VAR);
  map.addLayer(layers::Feature::SPARSITY);

  // Layers for visualization of normal vector
  map.addLayer(layers::Feature::NORMAL_X);
  map.addLayer(layers::Feature::NORMAL_Y);
  map.addLayer(layers::Feature::NORMAL_Z);
}

void FeatureExtractor::extractFeatures(HeightMap &map) {

  ensureFeatureLayers(map);
}

void FeatureExtractor::extractFeatures(
    HeightMap &map,
    const std::vector<grid_map::Index> &measured_indices) {

  ensureFeatureLayers(map);

  for (const auto &index : measured_indices) {
    if (map.isEmptyAt(index))
      continue;

    const auto &neighbors = map.getNeighborHeights(index, cfg.pca_radius);
    if (neighbors.size() < 4)
      continue;

    // 1. Compute Covariance Matrix
    Eigen::Matrix3d covariance;
    Eigen::Vector3d sum_neighbors(Eigen::Vector3d::Zero());
    Eigen::Matrix3d squared_sum_neighbors(Eigen::Matrix3d::Zero());
    for (const auto &neighbor : neighbors) {
      sum_neighbors += neighbor;
      squared_sum_neighbors.noalias() += neighbor * neighbor.transpose();
    }
    const auto mean_neighbors = sum_neighbors / neighbors.size();
    covariance = squared_sum_neighbors / neighbors.size() -
                 mean_neighbors * mean_neighbors.transpose();

    // Check if covariance matrix is degenerated using trace
    if (covariance.trace() < std::numeric_limits<float>::epsilon())
      continue;

    // Compute Eigenvectors and Eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.computeDirect(covariance, Eigen::DecompositionOptions::ComputeEigenvectors);
    const auto &eigenvectors = solver.eigenvectors();
    const auto &eigenvalues = solver.eigenvalues();

    // Line feature: second eigen value is near zero -> normal is not defined
    if (eigenvalues(1) < 1e-8)
      continue;

    // Check direction of the normal vector and flip the sign towards the user defined
    // direction.
    Eigen::Vector3d normal_vector = eigenvectors.col(0);
    Eigen::Vector3d positive_normal_vector(Eigen::Vector3d::UnitZ());
    if (normal_vector.dot(positive_normal_vector) < 0.0)
      normal_vector *= -1;

    // Calculate step
    auto minMax =
        std::minmax_element(neighbors.begin(),
                            neighbors.end(),
                            [](const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
                              return lhs(2) < rhs(2); // Compare z-components.
                            });

    double minZ = (*minMax.first)(2);  // Minimum z-component.
    double maxZ = (*minMax.second)(2); // Maximum z-component.
    map.at(layers::Feature::STEP, index) = maxZ - minZ;

    map.at(layers::Feature::SLOPE, index) =
        std::acos(std::abs(normal_vector(2))) * 180 / M_PI;
    map.at(layers::Feature::ROUGHNESS, index) = std::sqrt(eigenvalues(0));
    map.at(layers::Feature::CURVATURE, index) =
        std::abs(eigenvalues(0) / covariance.trace());
    map.at(layers::Feature::VARIANCE, index) = covariance(2, 2);

    const float GRID_RES = static_cast<float>(cfg.grid_res);
    int fit_num = static_cast<int>(cfg.pca_radius / GRID_RES);
    int grid_size = 2 * fit_num + 1;
    
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> vac(grid_size, grid_size);
    vac.setConstant(false);
    int vac_count = grid_size * grid_size;

    for (const auto &neighbor : neighbors) {
      int u = std::round((neighbor(0) - mean_neighbors(0)) / GRID_RES) + fit_num;
      int v = std::round((neighbor(1) - mean_neighbors(1)) / GRID_RES) + fit_num;
      
      if (u >= 0 && u < grid_size && v >= 0 && v < grid_size) {
        if (!vac(u, v)) {
          vac(u, v) = true;
          vac_count--;
        }
      }
    }

    float sparsity = 0.0f;
    if (vac_count > 0) {
      Eigen::MatrixXd M_vac(2, vac_count);
      int col_idx = 0;
      for (int i = 0; i < grid_size; i++) {
        for (int j = 0; j < grid_size; j++) {
          if (!vac(i, j)) {
            M_vac(0, col_idx) = static_cast<double>(i);
            M_vac(1, col_idx) = static_cast<double>(j);
            col_idx++;
          }
        }
      }

      Eigen::Vector2d mean_vac = M_vac.rowwise().mean();
      Eigen::MatrixXd centered_M_vac = M_vac.colwise() - mean_vac;
      Eigen::Matrix2d cov_vac = (centered_M_vac * centered_M_vac.transpose()) / static_cast<double>(vac_count);
      
      float trace = static_cast<float>(cov_vac.trace());
      float ratio = static_cast<float>(vac_count) / (grid_size * grid_size);

      const float RATIO_MAX = 0.40f;
      const float RATIO_MIN = 0.25f;
      const float CONV_THRE = 0.0014f;

      if (ratio > RATIO_MAX) {
        sparsity = 1.0f;
      } else if (ratio > RATIO_MIN && ratio <= RATIO_MAX && (1.0f / (trace + 1e-6f)) > CONV_THRE) {
        sparsity = (ratio - RATIO_MIN) / (RATIO_MAX - RATIO_MIN);
      } else {
        sparsity = 0.0f; 
      }
    }
    
    map.at(layers::Feature::SPARSITY, index) = sparsity;

    map.at(layers::Feature::NORMAL_X, index) = normal_vector(0);
    map.at(layers::Feature::NORMAL_Y, index) = normal_vector(1);
    map.at(layers::Feature::NORMAL_Z, index) = normal_vector(2);
    if (!map.isValid(index, layers::Feature::INTENSITY_MEAN)) {
        map.at(layers::Feature::INTENSITY_MEAN, index) = 0.0f;
        map.at(layers::Feature::INTENSITY_VAR, index) = 0.0f;
  }
}
}
} // namespace lesta
