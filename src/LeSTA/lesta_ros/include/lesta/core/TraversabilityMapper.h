/*
 * TraversabilityMapper.h
 *
 *  Modified by: Haoran Wang
 *  Revision date: 2026-08-12
 */

#pragma once

#include "lesta/core/TraversabilityEstimator.h"

namespace lesta {

class TraversabilityMapper : public TraversabilityEstimator {
public:
  struct Config : public TraversabilityEstimator::Config {
    bool use_log_odds_update;
    float learning_rate;
    float prob_min;
    float prob_max;
  } cfg;

  TraversabilityMapper(const Config &cfg);

  void traversabilityMapping(HeightMap &map,
                             const std::vector<grid_map::Index> &measured_indices);

private:
  void ensureTraversabilityMappingLayers(HeightMap &map);
  void traversabilityMappingImpl(HeightMap &map,
                                 const std::vector<grid_map::Index> &indices);

  // Numerically stable log odds function
  inline float log_odds(float prob) const {
    prob = std::max(std::min(prob, 0.9999f), 0.0001f);
    return std::log(prob) - std::log(1.0f - prob);
  }

  inline float clampValue(float value, float min, float max) const {
    return std::min(std::max(value, min), max);
  }
};

} // namespace lesta
