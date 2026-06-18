/*
 * trav_mapping_node.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "common/ros/common.h"
#include "lesta/core/core.h"
// [new] added .h file
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace lesta_ros {

class TravMappingNode {
public:
  struct Config {
    std::string lidarscan_topic;
    double map_pub_rate;
    double scan_filter_range;
    bool remove_backpoints;
    bool debug_mode;
  } cfg_;

  TravMappingNode();
  ~TravMappingNode() = default;
  void loadConfig(const ros::NodeHandle &nh);

private:
  // init functions
  void initializeTimers();
  void initializePubSubs();
  void initializeServices();

  // callback functions: main logic
  void lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void publishMaps(const ros::TimerEvent &event);

  pcl::PointCloud<Laser>::Ptr preprocessScan(
      const pcl::PointCloud<Laser>::Ptr &scan_raw,
      const geometry_msgs::TransformStamped &sensor2map);
  std::vector<grid_map::Index>
  terrainMapping(const pcl::PointCloud<Laser>::Ptr &cloud_input,
                 const Eigen::Vector3f &sensor_origin);

  // Helper functions
  std::vector<grid_map::Index>
  getMeasuredIndices(const HeightMap &map, const pcl::PointCloud<Laser>::Ptr &input_scan);
  void publishDownsampledScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishFilteredScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishRasterizedScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishTravMap(const HeightMap &heightmap,
                      const std::unordered_set<grid_map::Index> &measuredIndices);
  void publishMapRegion(const HeightMap &map);
  void toPointCloud2(const HeightMap &map,
                     const std::vector<std::string> &layers,
                     const std::unordered_set<grid_map::Index> &measuredIndices,
                     sensor_msgs::PointCloud2 &cloud);
  void toMapRegion(const HeightMap &map, visualization_msgs::Marker &marker);

  // Timer callback

  ros::NodeHandle nh_;

  // Subscribers & Publishers
  // ros::Subscriber sub_lidarscan_;
  // [new] Add subscriber for visual cost map
  // =================================
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_lidarscan_sync_;
  message_filters::Subscriber<sensor_msgs::Image> sub_visual_cost_sync_;
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image>
      SyncPolicy;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  // syncedCallback function 
  void syncedCallback(const sensor_msgs::PointCloud2ConstPtr& scan_msg, const sensor_msgs::ImageConstPtr& cost_img_msg);
  // =================================

  ros::Publisher pub_downsampled_scan_;
  ros::Publisher pub_filtered_scan_;
  ros::Publisher pub_rasterized_scan_;
  ros::Publisher pub_travmap_;
  ros::Publisher pub_map_region_;

  // Timers
  ros::Timer map_publish_timer_;

  // Core objects
  std::unique_ptr<lesta::GlobalMapper> mapper_;
  std::unique_ptr<lesta::FeatureExtractor> feature_extractor_;
  std::unique_ptr<lesta::TraversabilityMapper> trav_mapper_;
  TransformOps tf_;
  FrameID frame_id_;

  // State variables
  bool lidarscan_received_{false};
};
} // namespace lesta_ros