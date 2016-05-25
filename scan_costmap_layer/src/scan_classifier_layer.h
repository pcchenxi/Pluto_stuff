#ifndef SCAN_CLASSIFIER_H_
#define SCAN_CLASSIFIER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Vector3.h"

namespace scan_classifier_layer_namespace
{

class ScanClassifierLayer : public costmap_2d::CostmapLayer
{
public:
  ScanClassifierLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  double mark_x_, mark_y_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  
  ////////////////////////////////////////////////////////////////////
  void callback_costmap(const sensor_msgs::PointCloudConstPtr &cloud_in);
  sensor_msgs::PointCloud costmap_cloud_;
  bool updating_, first_;
  int  map_size_x_;
  int  map_size_y_;
  ros::Subscriber sub_;
  ros::Publisher  pub_updated_;
  ros::NodeHandle nh_;
  std::vector< std::pair<double,double> > mark_;

  int b_min_x, b_min_y, b_max_x, b_max_y;
  unsigned char* costmap_cpy_;
};
}
#endif
