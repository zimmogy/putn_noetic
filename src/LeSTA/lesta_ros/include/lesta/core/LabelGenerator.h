/*
 * LabelGenerator.h
 *
 *  Created on: Feb 07, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
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
    double max_traversable_visual_cost; // New parameter for visual cost threshold
    double fatal_step_threshold; // New parameter for fatal step 
  } cfg;

  using Traversability = lesta_types::Traversability;
  LabelGenerator(const Config &cfg);

  void addFootprint(HeightMap &map, grid_map::Position &robot_position, float traversability_score = 1.0f);
  void addObstacles(HeightMap &map, const std::vector<grid_map::Index> &measured_indices);

void NewFunction(height_mapping::HeightMap & map, const grid_map::Index & index);

private:
  void ensureLabelLayers(HeightMap &map);
};

} // namespace lesta
