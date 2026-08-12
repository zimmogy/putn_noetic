/*
 * TraversabilityEstimator.h
 *
 *  Modified by: Haoran Wang
 *  Revision date: 2026-08-12
 */

#pragma once

#include "lesta/core/TraversabilityNetwork.h"
#include "lesta/types/traversability.h"
#include "lesta/types/layer_definitions.h"

#include <height_mapping_core/height_mapping_core.h>

namespace lesta {

class TraversabilityEstimator {
public:
  struct Config {
    std::string model_path;
    int input_dimension;
    float binary_threshold;
    std::vector<std::string> feature_fields;
  } cfg;

  using Traversability = lesta_types::Traversability;

  TraversabilityEstimator(const Config &cfg);

  void estimateTraversability(HeightMap &map);
  void estimateTraversability(HeightMap &map,
                              const std::vector<grid_map::Index> &measured_indices);
  const TraversabilityNetwork &getModel() const { return network_; }

  void ensureTraversabilityLayers(HeightMap &map);

  // Numerically stable sigmoid function
  inline float sigmoid(float x) const {
    if (x >= 0) {
      return 1.0f / (1.0f + std::exp(-x));
    } else {
      float exp_x = std::exp(x);
      return exp_x / (1.0f + exp_x);
    }
  }

private:
  void estimateTraversabilityImpl(HeightMap &map,
                                  const std::vector<grid_map::Index> &indices);

  TraversabilityNetwork network_;
  std::map<std::string, std::string> featurefield_to_layer_ = {
      {"step", layers::Feature::STEP},
      {"slope", layers::Feature::SLOPE},
      {"roughness", layers::Feature::ROUGHNESS},
      {"curvature", layers::Feature::CURVATURE},
      {"variance", layers::Feature::VARIANCE},
      {"intensity_mean", layers::Feature::INTENSITY_MEAN},
      {"intensity_var", layers::Feature::INTENSITY_VAR},
      {"sparsity", layers::Feature::SPARSITY},
  };
};

} // namespace lesta
