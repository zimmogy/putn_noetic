/*
 * FeatureExtractor.cpp
 *
 *  Created on: Feb 07, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
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
  
  // 确保新的特征层被初始化
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
  // TODO: implement feature extraction for entire map (less efficient)
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
    // 补充：几何方差（Z轴高度方差）
    map.at(layers::Feature::VARIANCE, index) = covariance(2, 2);

    // ==================================================================
    // 新增 ：引入 PUTN 的协方差迹稀疏度评估，解决长尾分布导致的 Logit 爆炸问题
    // ==================================================================
    // 1. 定义局部栅格参数 (建议将 resolution 等参数移至 yaml 配置中)
    const float GRID_RES = static_cast<float>(cfg.grid_res); // 局部栅格分辨率 0.1m
    int fit_num = static_cast<int>(cfg.pca_radius / GRID_RES);
    int grid_size = 2 * fit_num + 1;
    
    // 2. 初始化局部空缺矩阵 (false 表示 vacant 空缺)
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> vac(grid_size, grid_size);
    vac.setConstant(false);
    int vac_count = grid_size * grid_size;

    // 3. 将邻域点云投影到局部 2D 栅格中，消除被覆盖的空缺
    for (const auto &neighbor : neighbors) {
      // 相对于局部点云均值的 XY 像素坐标
      int u = std::round((neighbor(0) - mean_neighbors(0)) / GRID_RES) + fit_num;
      int v = std::round((neighbor(1) - mean_neighbors(1)) / GRID_RES) + fit_num;
      
      if (u >= 0 && u < grid_size && v >= 0 && v < grid_size) {
        if (!vac(u, v)) {
          vac(u, v) = true; // 标记为被点云击中（占用）
          vac_count--;      // 空缺数量减少
        }
      }
    }

    float sparsity = 0.0f;
    if (vac_count > 0) {
      // 4. 提取所有空缺网格的 2D 坐标
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

      // 5. 计算空缺分布的协方差矩阵的迹 (Trace)
      Eigen::Vector2d mean_vac = M_vac.rowwise().mean();
      Eigen::MatrixXd centered_M_vac = M_vac.colwise() - mean_vac;
      // 除以空缺总数求协方差
      Eigen::Matrix2d cov_vac = (centered_M_vac * centered_M_vac.transpose()) / static_cast<double>(vac_count);
      
      float trace = static_cast<float>(cov_vac.trace());
      float ratio = static_cast<float>(vac_count) / (grid_size * grid_size);

      // 6. PUTN 稀疏度判定超参数 (推荐从 cfg 中读取，此处使用原论文默认值)
      const float RATIO_MAX = 0.40f;   // 极度空缺阈值
      const float RATIO_MIN = 0.25f;   // 空缺容忍底线
      const float CONV_THRE = 0.0014f; // 集中度阈值 (1/Trace)，评估是否为集中大坑

      // 7. 评估并赋值
      if (ratio > RATIO_MAX) {
        sparsity = 1.0f; // 绝对稀疏（如悬崖边缘、大面积盲区）
      } else if (ratio > RATIO_MIN && ratio <= RATIO_MAX && (1.0f / (trace + 1e-6f)) > CONV_THRE) {
        // 空洞分布集中，按比例计算稀疏度危险值
        sparsity = (ratio - RATIO_MIN) / (RATIO_MAX - RATIO_MIN);
      } else {
        // 零碎空洞（如草丛散射）或比例极低，视为安全
        sparsity = 0.0f; 
      }
    }
    
    // 图层名仍保持 SPARSITY 以兼容 Python 端网络配置
    map.at(layers::Feature::SPARSITY, index) = sparsity;

    // =========================================================
    map.at(layers::Feature::NORMAL_X, index) = normal_vector(0);
    map.at(layers::Feature::NORMAL_Y, index) = normal_vector(1);
    map.at(layers::Feature::NORMAL_Z, index) = normal_vector(2);
    // 新增 2：预留强度特征槽位
    // 假设上游的 HeightMapper 已经将 INTENSITY_MEAN 和 INTENSITY_VAR 写入 GridMap，
    // 这里只需确保提取时包含这些层即可。如果没有写入，可以在此处设置默认值避免报错：
    if (!map.isValid(index, layers::Feature::INTENSITY_MEAN)) {
        map.at(layers::Feature::INTENSITY_MEAN, index) = 0.0f; // 或对接你的点云强度提取逻辑
        map.at(layers::Feature::INTENSITY_VAR, index) = 0.0f;
  }
}
}
} // namespace lesta
