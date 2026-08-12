/*
 * TraversabilityModel.h
 *
 *  Modified by: Haoran Wang
 *  Revision date: 2026-08-12
 */

#pragma once

#include <torch/script.h>
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace lesta {

class TraversabilityNetwork {
public:
  struct Config {
    std::string model_path;
    float classifier_threshold;
  } cfg;

  TraversabilityNetwork() = default;
  ~TraversabilityNetwork() = default;

  bool loadCheckpoint(const std::string &model_path);
  int inputDimension() const { return input_dimension_; }
  void setInputDimension(int input_dimension) { input_dimension_ = input_dimension; }
  std::vector<float> inference(const std::vector<Eigen::VectorXf> &features);

private:
  torch::jit::script::Module model_; // pytorch learned model
  bool model_loaded_{false};
  int input_dimension_{-1};
};
} // namespace lesta
