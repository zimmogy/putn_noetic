/*
 * FeatureExtractor.h
 *
 *  Modified by: Haoran Wang
 *  Revision date: 2026-08-12
 */

#pragma once

#include <height_mapping_core/height_mapping_core.h>
#include "lesta/types/layer_definitions.h"

namespace lesta {

class FeatureExtractor {
public:
  struct Config {
    double pca_radius;
    double grid_res;
  } cfg;

  FeatureExtractor(const Config &cfg);

  void extractFeatures(HeightMap &map);
  void extractFeatures(HeightMap &map,
                       const std::vector<grid_map::Index> &measured_indices);

private:
  void ensureFeatureLayers(HeightMap &map);
};

} // namespace lesta
