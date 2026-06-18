/*
 * LabelGenerator.cpp
 *
 *  Created on: Feb 10, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/core/LabelGenerator.h"

namespace lesta {

LabelGenerator::LabelGenerator(const Config &cfg) : cfg(cfg) {}

void LabelGenerator::ensureLabelLayers(HeightMap &map) {

  map.addLayer(layers::Label::FOOTPRINT, 0.0f);
  map.addLayer(layers::Label::TRAVERSABILITY);
}

void LabelGenerator::addFootprint(HeightMap &map, grid_map::Position &robot_position, float traversability_score) {

  ensureLabelLayers(map);

  // Iterate over the footprint radius
  grid_map::CircleIterator iterator(map, robot_position, cfg.footprint_radius);
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    if (map.isEmptyAt(*iterator))
      continue;

    // pass if recoreded as obstacle to prevent noisy label generation
    auto is_non_traversable = std::abs(map.at(layers::Label::TRAVERSABILITY, *iterator) -
                                       (float)Traversability::NON_TRAVERSABLE) < 1e-3;
    if (is_non_traversable)
      continue;

    map.at(layers::Label::FOOTPRINT, *iterator) = 1.0;
    map.at(layers::Label::TRAVERSABILITY, *iterator) = traversability_score;
  }
}

void LabelGenerator::addObstacles(HeightMap &map,
                                  const std::vector<grid_map::Index> &measured_indices) {

  ensureLabelLayers(map);
  // ensure that map include vision layers
  if (!map.exists(layers::Visual::COST)) 
    {map.addLayer(layers::Visual::COST, std::nanf(""));}
  for (const auto &index : measured_indices) {

    if (map.isEmptyAt(layers::Feature::SLOPE, index))
      continue;

    bool has_footprint = std::abs(map.at(layers::Label::FOOTPRINT, index) - 1.0) < 1e-3;
    float step = map.at(layers::Feature::STEP, index);
    float roughness = map.at(layers::Feature::ROUGHNESS, index);  
    // [新增] 安全地获取视觉代价
    float visual_cost = std::numeric_limits<float>::quiet_NaN();
    if (!map.isEmptyAt(layers::Visual::COST, index)) {
        visual_cost = map.at(layers::Visual::COST, index);
    }

    // ======================================
    // Hybrid Pseudo-labeling Strategy （Modified - 双阈值截断）
    // ======================================
    // [version 0.2] Fatal Threshold
    // 1. 绝对致死物理极限 (Fatal Threshold): 一票否决
    // 作用：兜底 SLAM 定位漂移，防止将历史足迹错误投影到真正的悬崖或高墙上
    if (step > cfg.fatal_step_threshold) {
      map.at(layers::Label::TRAVERSABILITY, index) = (float)Traversability::NON_TRAVERSABLE;
    }
    // 2. 真实足迹最高优先级 (Soft Positive/Negative):
    // 作用：解决高草、软泥等“可变形障碍物”带来的几何雷达误判。只要真开过去了，绝对相信本体 IMU
    else if (has_footprint) {
      // nothing todo, keep the historical score calculated by IMU
      continue; 
    }
    // 3. 常规几何障碍 (Hard-negative):
    // 作用：处理没有轨迹覆盖的常规障碍物（如 0.3m 以上的石头或台阶）
    else if (step > cfg.max_traversable_step) {
      map.at(layers::Label::TRAVERSABILITY, index) = (float)Traversability::NON_TRAVERSABLE;
    }
    // 4. 严苛的绝对安全奖励 (Hard-positive): 视觉与几何双重认证
    // 作用：极其平坦且视觉认为安全的区域，打上 1.0 的安全标签
    else if (step < 0.03 && roughness < 0.01 && !std::isnan(visual_cost) && visual_cost < cfg.max_traversable_visual_cost) {
      map.at(layers::Label::TRAVERSABILITY, index) = (float)Traversability::TRAVERSABLE;
    }
    // 5. 其他未知区域 (Unknown):
    // 作用：留白，交由后续的 MLP 网络去泛化
    else {
      map.at(layers::Label::TRAVERSABILITY, index) = (float)Traversability::UNKNOWN;
    }
    /* version 0.1
    // 1. Hard-negative sample from absolute physical obbstacle
    if (step > cfg.max_traversable_step) {
      map.at(layers::Label::TRAVERSABILITY, index) = (float)Traversability::NON_TRAVERSABLE;
    }
    // 2. Soft positive/negative sample: if there is trajectory, keep  historical score calculated by IMU.
    else if (has_footprint) {
      // nothing todo, keep the score
      continue; 
    }
    // 3. Hard-positive sample with adjustable parameter
    else if (step < 0.03 && roughness < 0.01 && !std::isnan(visual_cost) && visual_cost < cfg.max_traversable_visual_cost) {
      map.at(layers::Label::TRAVERSABILITY, index) = (float)Traversability::TRAVERSABLE;
    }
    // 4. 其他未知区域：由于没有被探索过，交给之后的网络自己去泛化
    else {
      map.at(layers::Label::TRAVERSABILITY, index) = (float)Traversability::UNKNOWN;
    }
    */
    // if (map.at(layers::Feature::STEP, index) > cfg.max_traversable_step)
    //   map.at(layers::Label::TRAVERSABILITY, index) =
    //       (float)Traversability::NON_TRAVERSABLE;
    // else if (has_footprint) // Noisy label removal
    //   map.at(layers::Label::TRAVERSABILITY, index) = (float)Traversability::TRAVERSABLE;
    // else
    //   map.at(layers::Label::TRAVERSABILITY, index) = (float)Traversability::UNKNOWN;
  }
}

} // namespace lesta
