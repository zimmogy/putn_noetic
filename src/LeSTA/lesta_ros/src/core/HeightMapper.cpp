/*
 * HeightMapping.cpp
 *
 *  Modified by: Haoran Wang
 *  Revision date: 2026-08-12
 */

#include "lesta/core/HeightMapper.h"
#include "lesta/types/layer_definitions.h"

namespace lesta {

HeightMapper::HeightMapper(const Config &cfg)
    : cfg{cfg}, heightFilter_{cfg.min_height, cfg.max_height} {

  initMap();
  initHeightEstimator();
}

void HeightMapper::initMap() {

  // Check parameter validity
  if (cfg.grid_resolution <= 0) {
    throw std::invalid_argument(
        "[height_mapping::HeightMapper]: Grid resolution must be positive");
  }
  if (cfg.map_length_x <= 0 || cfg.map_length_y <= 0) {
    throw std::invalid_argument(
        "[height_mapping::HeightMapper]: Map dimensions must be positive");
  }

  // Initialize map geometry
  map_.setFrameId(cfg.frame_id);
  map_.setGeometry(grid_map::Length(cfg.map_length_x, cfg.map_length_y),
                   cfg.grid_resolution);
  
  // Initialize intensity feature layers.
  map_.addLayer(layers::Feature::INTENSITY_MEAN);
  map_.addLayer(layers::Feature::INTENSITY_VAR);
}

void HeightMapper::initHeightEstimator() {

  // Set height estimator
  // - Kalman Filter
  // - Moving Average
  // - StatMean (by default)

  if (cfg.estimator_type == "KalmanFilter") {
    height_estimator_ = std::make_unique<height_mapping::KalmanEstimator>();

    std::cout << "\033[1;33m[height_mapping::HeightMapper]: Height estimator "
                 "type --> KalmanFilter \033[0m\n";

  } else if (cfg.estimator_type == "MovingAverage") {
    height_estimator_ = std::make_unique<height_mapping::MovingAverageEstimator>();

    std::cout << "\033[1;33m[height_mapping::HeightMapper]: Height estimator "
                 "type --> MovingAverage \033[0m\n";

  } else if (cfg.estimator_type == "StatMean") {
    height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();

    std::cout << "\033[1;33m[height_mapping::HeightMapper]: Height estimator "
                 "type --> StatisticalMeanEstimator "
                 "\033[0m\n";

  } else {
    height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();

    std::cout << "\033[1;33m[height_mapping::HeightMapper] Invalid height "
                 "estimator type. Set Default: StatMeanEstimator \033[0m\n";
  }
}

template <typename PointT>
typename boost::shared_ptr<pcl::PointCloud<PointT>> HeightMapper::heightMapping(
    const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cloud) {

  // 1. Rasterize pointcloud
  auto cloud_rasterized = cloudRasterization<PointT>(cloud, cfg.grid_resolution);
  if (cloud_rasterized->empty()) {
    std::cout << "\033[1;31m[height_mapping::HeightMapper]: Height estimation failed. "
                 "Rasterized cloud is empty\033[0m\n";
    return nullptr;
  }

  // 2. Update height map
  height_estimator_->estimate(map_, *cloud_rasterized);

  // 3. Update intensity feature layers when the point type provides intensity.
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::intensity>::value) {
      std::unordered_map<std::pair<int, int>, std::vector<float>, pair_hash> intensity_grid;
      grid_map::Position measuredPosition;
      grid_map::Index measuredIndex;

      for (const auto &point : *cloud) {
          measuredPosition << point.x, point.y;
          
          if (!map_.getIndex(measuredPosition, measuredIndex)) {
              continue; 
          }

          auto gridIndex = std::make_pair(measuredIndex.x(), measuredIndex.y());
          intensity_grid[gridIndex].push_back(point.intensity);
      }

      for (const auto &[index_pair, intensities] : intensity_grid) {
          grid_map::Index gridIndex(index_pair.first, index_pair.second);

          float sum = 0.0f;
          for (float i : intensities) sum += i;
          float mean = sum / intensities.size();

          float var = 0.0f;
          if (intensities.size() > 1) {
              float var_sum = 0.0f;
              for (float i : intensities) {
                  var_sum += (i - mean) * (i - mean);
              }
              var = var_sum / (intensities.size() - 1); 
          }

          map_.at(layers::Feature::INTENSITY_MEAN, gridIndex) = mean;
          map_.at(layers::Feature::INTENSITY_VAR, gridIndex) = var;
      }
  }
  return cloud_rasterized;
}

template <typename PointT>
void HeightMapper::fastHeightFilter(
    const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cloud,
    typename boost::shared_ptr<pcl::PointCloud<PointT>> &filtered_cloud) {

  heightFilter_.filter<PointT>(cloud, filtered_cloud);
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
HeightMapper::cloudRasterization(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                                 float gridSize) {

  if (cloud->empty()) {
    return cloud;
  }

  // grid cell: first, second -> (x_index, y_index), point
  std::unordered_map<std::pair<int, int>, PointT, pair_hash> grid_map;
  for (const auto &point : *cloud) {
    int x_index = std::floor(point.x / gridSize);
    int y_index = std::floor(point.y / gridSize);

    auto grid_key = std::make_pair(x_index, y_index);
    auto [iter, inserted] = grid_map.try_emplace(grid_key, point);

    if (!inserted && point.z > iter->second.z) {
      iter->second = point;
    }
  }

  auto cloud_downsampled = boost::make_shared<pcl::PointCloud<PointT>>();
  cloud_downsampled->reserve(grid_map.size());
  for (const auto &grid_cell : grid_map) {
    cloud_downsampled->points.emplace_back(grid_cell.second);
  }

  cloud_downsampled->header = cloud->header;
  return cloud_downsampled;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
HeightMapper::cloudRasterizationAlt(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                                    float gridSize) {

  if (cloud->empty()) {
    return cloud;
  }

  std::unordered_map<std::pair<int, int>, PointT, pair_hash> gridCells;
  grid_map::Position measuredPosition;
  grid_map::Index measuredIndex;

  for (const auto &point : *cloud) {
    measuredPosition << point.x, point.y;

    if (!map_.getIndex(measuredPosition, measuredIndex)) {
      continue; // Skip points outside map bounds
    }

    auto gridIndex = std::make_pair(measuredIndex.x(), measuredIndex.y());
    auto [iter, inserted] = gridCells.try_emplace(gridIndex, point);

    if (inserted) {
      // If new point, get exact grid center position
      iter->second = point;
      map_.getPosition(measuredIndex, measuredPosition);
      iter->second.x = measuredPosition.x();
      iter->second.y = measuredPosition.y();
    } else if (point.z > iter->second.z) {
      // If higher point, update z while keeping grid center position
      iter->second = point;
      map_.getPosition(measuredIndex, measuredPosition);
      iter->second.x = measuredPosition.x();
      iter->second.y = measuredPosition.y();
    }
  }

  auto cloudDownsampled = boost::make_shared<pcl::PointCloud<PointT>>();
  cloudDownsampled->reserve(gridCells.size());
  for (const auto &gridCell : gridCells) {
    cloudDownsampled->points.emplace_back(gridCell.second);
  }

  cloudDownsampled->header = cloud->header;
  return cloudDownsampled;
}

template <typename PointT>
void HeightMapper::raycasting(
    const Eigen::Vector3f &sensorOrigin,
    const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cloud) {
  if (cloud->empty()) {
    return;
  }
  raycaster_.correctHeight(map_, *cloud, sensorOrigin);
}

void HeightMapper::moveMapOrigin(const grid_map::Position &position) {
  map_.move(position);
}

/////////////////////////////////////////////////////////////////////////////
///////////////// EXPLICIT INSTANTIATION OF TEMPLATE FUNCTIONS //////////////
/////////////////////////////////////////////////////////////////////////////
// Laser
template void HeightMapper::fastHeightFilter<Laser>(
    const typename pcl::PointCloud<Laser>::Ptr &cloud,
    typename pcl::PointCloud<Laser>::Ptr &filtered_cloud);
template typename pcl::PointCloud<Laser>::Ptr
HeightMapper::heightMapping<Laser>(const pcl::PointCloud<Laser>::Ptr &cloud);

template pcl::PointCloud<Laser>::Ptr
HeightMapper::cloudRasterization<Laser>(const pcl::PointCloud<Laser>::Ptr &cloud,
                                        float gridSize);

template pcl::PointCloud<Laser>::Ptr
HeightMapper::cloudRasterizationAlt<Laser>(const pcl::PointCloud<Laser>::Ptr &cloud,
                                           float gridSize);

template void
HeightMapper::raycasting<Laser>(const Eigen::Vector3f &sensorOrigin,
                                const typename pcl::PointCloud<Laser>::Ptr &cloud);

// Color
template void HeightMapper::fastHeightFilter<Color>(
    const typename pcl::PointCloud<Color>::Ptr &cloud,
    typename pcl::PointCloud<Color>::Ptr &filtered_cloud);
template typename pcl::PointCloud<Color>::Ptr
HeightMapper::heightMapping<Color>(const pcl::PointCloud<Color>::Ptr &cloud);

template pcl::PointCloud<Color>::Ptr
HeightMapper::cloudRasterization<Color>(const pcl::PointCloud<Color>::Ptr &cloud,
                                        float gridSize);

template pcl::PointCloud<Color>::Ptr
HeightMapper::cloudRasterizationAlt<Color>(const pcl::PointCloud<Color>::Ptr &cloud,
                                           float gridSize);

template void
HeightMapper::raycasting<Color>(const Eigen::Vector3f &sensorOrigin,
                                const typename pcl::PointCloud<Color>::Ptr &cloud);

} // namespace lesta
