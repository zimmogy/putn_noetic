/*
 * label_point.h
 *
 *  Modified by: Haoran Wang
 *  Revision date: 2026-08-12
 */

#pragma once

#include <pcl/point_types.h>

namespace lesta_types {

struct EIGEN_ALIGN16 LabelPoint {
  PCL_ADD_POINT4D; // This adds the members x,y,z
  float elevation;
  float step;
  float slope;
  float roughness;
  float curvature;
  
  float intensity_mean;
  float intensity_var;
  float sparsity;

  float variance;
  float footprint;
  float traversability_label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace lesta_types

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lesta_types::LabelPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, elevation, elevation)(
        float,
        step,
        step)(float, slope, slope)(float, roughness, roughness)(
        float,
        curvature,
        curvature)(float, variance, variance)(float,
                                              footprint,
                                              footprint)(float,
                                                         traversability_label,
                                                         traversability_label)
                                                        (float,
                                                         intensity_mean, intensity_mean)
                                                        (float,
                                                         intensity_var, intensity_var)
                                                        (float,
                                                         sparsity, sparsity))
