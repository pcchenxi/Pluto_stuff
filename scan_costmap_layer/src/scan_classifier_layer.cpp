#include "scan_classifier_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(scan_classifier_layer_namespace::ScanClassifierLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace scan_classifier_layer_namespace
{

ScanClassifierLayer::ScanClassifierLayer()
{
  nh_ = ros::NodeHandle("~/" + name_);
  sub_ = nh_.subscribe("/costmap_cloud", 1, &ScanClassifierLayer::callback_costmap, this);
  pub_updated_ = nh_.advertise<geometry_msgs::Vector3>("/costmap_updated", 1);

  map_size_x_ = 600;
  map_size_y_ = 600;

  costmap_cpy_ = new unsigned char[map_size_x_ * map_size_y_];
}

void ScanClassifierLayer::onInitialize()
{
  //ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = costmap_2d::NO_INFORMATION;
  matchSize();
  float re = layered_costmap_->getCostmap()->getResolution();
  if(re == 0)
    re = 0.05;

  float x_s = -2.0;
  float x_e = 2.0 + re;
  float y_s = -2.0;
  float y_e = 2.0 + re;


  int map_size_x = layered_costmap_->getCostmap()->getSizeInCellsX();
  int map_size_y = layered_costmap_->getCostmap()->getSizeInCellsY();

  std::vector<geometry_msgs::Point> rect;
  geometry_msgs::Point p1, p2, p3, p4;
  p1.x = -map_size_x/2; p1.y = -map_size_y/2;
  p2.x = map_size_x/2;  p2.y = -map_size_y/2;
  p3.x = map_size_x/2;  p3.y = map_size_y/2;
  p4.x = -map_size_x/2; p4.y = map_size_y/2;

 // setConvexPolygonCost(rect, 255);


  for(float i = x_s; i <x_e; i=i+re)
  {
    for(float j = y_s; j<y_e; j=j+re)
    {
        unsigned int mx;
        unsigned int my;
       // std::cout << i << " " << j <<std::endl;
        if(layered_costmap_->getCostmap()->worldToMap(i, j, mx, my))
        {
            setCost(mx, my, 1);
        }
    }
  }


 // updateWithOverwrite(*layered_costmap_->getCostmap(), x_s, x_e, y_s, y_e);
  std::cout << re << std::endl;
  //sub_ = nh.subscribe("/costmap_cloud", 1, &ScanClassifierLayer::callback_costmap, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &ScanClassifierLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void ScanClassifierLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void ScanClassifierLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;


  double wx_min, wy_min, wx_max, wy_max;
  for(int i = 0; i < costmap_cloud_.points.size(); i++)
  {
    mark_x_   = costmap_cloud_.points[i].x;
    mark_y_   = costmap_cloud_.points[i].y;
    int cost  = costmap_cloud_.points[i].z;

    if(cost == 255)
        continue;

    *min_x = std::min(*min_x, mark_x_);
    *min_y = std::min(*min_y, mark_y_);
    *max_x = std::max(*max_x, mark_x_);
    *max_y = std::max(*max_y, mark_y_);
  }

  b_min_x = *min_x;
  b_min_y = *min_y;
  b_max_x = *max_x;
  b_max_y = *max_y;
}

void ScanClassifierLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_ )
    return;

  map_size_x_ = master_grid.getSizeInCellsX();
  map_size_y_ = master_grid.getSizeInCellsY();

  if(first_)
  {
    updateWithTrueOverwrite(master_grid, 0, 0, map_size_x_, map_size_y_);
    first_ = false;
    return;
  }

 // resetMaps();
  unsigned int mx;
  unsigned int my;

  for(int i = 0; i < costmap_cloud_.points.size(); i++)
  {
    mark_x_   = costmap_cloud_.points[i].x;
    mark_y_   = costmap_cloud_.points[i].y;
    int cost  = costmap_cloud_.points[i].z;

    if(master_grid.worldToMap(mark_x_, mark_y_, mx, my))
    {
      if(cost != 255)
      {
	setCost(mx, my, cost);
	setCost(mx+1, my+1, cost);
	setCost(mx, my+1, cost);
	setCost(mx+1, my, cost);
      }
      //master_grid.setCost(mx, my, cost_v);
    }
  }
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
 // updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);

}


void ScanClassifierLayer::callback_costmap(const sensor_msgs::PointCloudConstPtr &cloud_in)
{
  updating_ = true;
  //costmap_cloud_.points.clear();

  costmap_cloud_  = *cloud_in;

  geometry_msgs::Vector3 flag;

//  float x = costmap_cloud_.points[0].x;
//  float y = costmap_cloud_.points[0].y;
//
//  if(x == -1 && y == -1)
    pub_updated_.publish(flag);

 // std::cout << "in call back" << costmap_cloud_.points.size() << std::endl;
}




} // end namespace
