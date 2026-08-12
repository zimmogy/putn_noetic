/*
 * LabelGenerator.h
 *
 *  Modified by: Haoran Wang
 *  Revision date: 2026-08-12
 */

#pragma once

#include <height_mapping_core/height_mapping_core.h>
#include "lesta/types/traversability.h"
#include "lesta/types/layer_definitions.h"

namespace lesta {

class LabelGenerator {
public:
  struct Config {
    double footprint_radius;
    double max_traversable_step;
  } cfg;

  using Traversability = lesta_types::Traversability;
  LabelGenerator(const Config &cfg);

  void addFootprint(HeightMap &map, grid_map::Position &robot_position);
  void addObstacles(HeightMap &map, const std::vector<grid_map::Index> &measured_indices);

void NewFunction(height_mapping::HeightMap & map, const grid_map::Index & index);

private:
  void ensureLabelLayers(HeightMap &map);
};

} // namespace lesta
