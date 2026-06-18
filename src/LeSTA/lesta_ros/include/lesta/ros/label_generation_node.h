/*
 * label_generation_node.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "lesta/save_training_data.h"
// =============================
// [new] IMU soft-label head file
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <deque>
#include <mutex>
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
// =============================

#include "common/ros/common.h"
#include "lesta/core/core.h"

namespace lesta_ros {

class LabelGenerationNode {
public:
  struct Config {
    std::string lidarscan_topic;
    double pose_update_rate;
    double map_pub_rate;
    double footprint_radius;
    bool remove_backpoints;
    bool debug_mode;
  } cfg_;

  LabelGenerationNode();
  ~LabelGenerationNode() = default;
  void loadConfig(const ros::NodeHandle &nh);

private:
  // init functions
  void initializeTimers();
  void initializePubSubs();
  void initializeServices();

  // callback functions
  void lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void recordFootprints(const ros::TimerEvent &event);
  void publishLabelMap(const ros::TimerEvent &event);
  
  // =============================
  // [new] IMU callback for soft-label generation
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  float calculateSoftLabel();
  // =============================
  // [new] 
  void sensorSyncCallback(const sensor_msgs::PointCloud2ConstPtr& scan_msg, const sensor_msgs::ImageConstPtr& visual_cost_msg);

  pcl::PointCloud<Laser>::Ptr
  preprocessScan(const pcl::PointCloud<Laser>::Ptr &scan_raw,
                 const geometry_msgs::TransformStamped &sensor2base,
                 const geometry_msgs::TransformStamped &base2map);
  std::vector<grid_map::Index>
  terrainMapping(const pcl::PointCloud<Laser>::Ptr &inputcloud,
                 const Eigen::Vector3f &sensorOrigin3D);

  // service callbacks
  bool saveLabelMap(lesta::save_training_data::Request &req, 
                    lesta::save_training_data::Response &res);

  // utility functions
  std::vector<grid_map::Index>
  getMeasuredIndices(const HeightMap &map, const pcl::PointCloud<Laser>::Ptr &input_scan);
  void publishDownsampledScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishFilteredScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishRasterizedScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void toPointCloud2(const HeightMap &map,
                     const std::vector<std::string> &layers,
                     const std::unordered_set<grid_map::Index> &grid_indices,
                     sensor_msgs::PointCloud2 &cloud);
  void toMapRegion(const HeightMap &map, visualization_msgs::Marker &region);

  ros::NodeHandle nh_;

  // Subscribers & Publishers
  // ros::Subscriber sub_lidarscan_;
  // [new] 
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_lidarscan_sync_;
  message_filters::Subscriber<sensor_msgs::Image> sub_visual_cost_sync_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> SyncPolicy;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  ros::Subscriber sub_imu_; // [new] IMU subscriber

  ros::Publisher pub_downsampled_scan_;
  ros::Publisher pub_filtered_scan_;
  ros::Publisher pub_rasterized_scan_;
  ros::Publisher pub_labelmap_;
  ros::Publisher pub_mapping_region_;

  // Timers
  ros::Timer map_publish_timer_;
  ros::Timer pose_update_timer_;

  // Services
  ros::ServiceServer srv_save_map_;

  // Core objects
  std::unique_ptr<lesta::GlobalMapper> mapper_;
  std::unique_ptr<lesta::FeatureExtractor> feature_extractor_;
  std::unique_ptr<lesta::LabelGenerator> label_generator_;
  TransformOps tf_;
  FrameID frame_id_;

  // State variables
  bool lidarscan_received_{false};

  // =============================
  // [new] IMU state variables and superparameters for soft-label 
  std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
  std::mutex imu_mutex_;

  double imu_window_size_ = 1.0; // seconds
  double lambda_decay_ = 0.05; // decay factor for older IMU data in soft label calculation
  double min_soft_label_ = 0.2; // minimum soft label value to prevent zero labels
  // =============================
  // [new]

};
} // namespace lesta_ros