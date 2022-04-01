#ifndef GROUND_SEGMENTATION_H_
#define GROUND_SEGMENTATION_H_

#include <mutex>
#include <ros/ros.h>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include "ground_seg/segment.h"
#include <pcl/common/centroid.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <ctime>
#include <opencv/cv.h>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;//X:任意大小的float类型的vector


const double seg_angle = M_PI/3;
typedef pcl::PointXYZI  PointType;


struct GroundSegmentationParams {
  GroundSegmentationParams() :
      selection(1),
      visualize(false),
      r_min_square(0.3 * 0.3),
      r_max_square(20*20),
      n_bins(30),
      n_segments(180),
      max_dist_to_line(0.15),
      max_slope(1),
      n_threads(4),
      max_error_square(0.01),
      long_threshold(2.0),
      max_long_height(0.1),
      max_start_height(0.2),
      sensor_height(0.2),
      line_search_angle(0.2),
      sensor_model(32),
      num_iter(3),
      num_lpr(20), 
      th_seeds(1.2),
      th_dists(0.3),
      horizon_scan(1800),
      N_scan(32),
      ang_res_x(0.2),
      ang_res_y(1.33),
      ang_bottom(30.67),
      groundscan_Ind(20),
      minrange(1),
      ground_N(20),
      out_slope(0.8){}

  // Visualize estimated ground.
  bool visualize;
  // Minimum range of segmentation.
  double r_min_square;
  // Maximum range of segmentation.
  double r_max_square;
  // Number of radial bins.
  int n_bins;
  // Number of angular segments.
  int n_segments;
  // Maximum distance to a ground line to be classified as ground.
  double max_dist_to_line;
  // Max slope to be considered ground line.
  double max_slope;
  // Max error for line fit.
  double max_error_square;
  // Distance at which points are considered far from each other.
  double long_threshold;
  // Maximum slope for
  double max_long_height;
  // Maximum heigh of starting line to be labelled ground.
  double max_start_height;
  // Height of sensor above ground.
  double sensor_height;
  // How far to search for a line in angular direction [rad].
  double line_search_angle;
  // Number of threads.
  int n_threads;
  // Selection of segment_methods
  int selection;
  /////////////////////////////parameter for scancall back2
  //velodyne sensor_mode(21)
  int sensor_model;
  // iteration times
  int  num_iter;
  // number of the lowest z point
  int num_lpr;
  // threshold distance of seeds    
  double th_seeds;
  // Distance Threshold th_dists
  double   th_dists;
  /////////////////parameter for scan ground////////
  int N_scan;
  int horizon_scan;
  double ang_res_x;
  double ang_res_y;
  double ang_bottom;
  int groundscan_Ind;
  double minrange;
  int ground_N;

  double out_slope;
};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;

class GroundSegmentation {

  const GroundSegmentationParams params_;

  // Access with segments_[segment][bin].
  std::vector<Segment> segments_;

  // Bin index of every point.
  std::vector<std::pair<int, int> > bin_index_;

  // Bin array to record the number of ground point in each bin;
  std::vector<std::vector<int> > bin_ground;

  // 2D coordinates (d, z) of every point in its respective segment.
  std::vector<Bin::MinZPoint> segment_coordinates_;

  // Visualizer.
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

  void assignCluster(std::vector<int>* segmentation);

  void assignClusterThread(const unsigned int& start_index,
                           const unsigned int& end_index,
                           std::vector<int>* segmentation);

  void border_detection(std::vector< std::vector<int> >& bin_ground, std::vector<int>* segmentation);

  void insertPoints(const PointCloud& cloud);

  void insertionThread(const PointCloud& cloud,
                       const size_t start_index,
                       const size_t end_index);

  void getMinZPoints(PointCloud* out_cloud);

  void getLines(std::list<PointLine>* lines);

  void lineFitThread(const unsigned int start_index, const unsigned int end_index,
                     std::list<PointLine> *lines, std::mutex* lines_mutex);

  pcl::PointXYZ minZPointTo3d(const Bin::MinZPoint& min_z_point, const double& angle);

  void getMinZPointCloud(PointCloud* cloud);

  void visualizePointCloud(const PointCloud::ConstPtr& cloud,
                           const std::string& id = "point_cloud");

  void visualizeLines(const std::list<PointLine>& lines);

  void visualize(const std::list<PointLine>& lines, const PointCloud::ConstPtr& cloud, const PointCloud::ConstPtr& ground_cloud, const PointCloud::ConstPtr& obstacle_cloud);

public:

  GroundSegmentation(const GroundSegmentationParams& params = GroundSegmentationParams());

  void segment(const PointCloud& cloud, std::vector<int>* segmentation);
  void transform_extrinsic(const pcl::PointCloud<pcl::PointXYZ> & cloud_valid, pcl::PointCloud<pcl::PointXYZ> & cloud_trans,Eigen::Matrix4f tran_matrix);
};

class groundplane_fit{/////外部点云数据作为输入参数(引用)，然后调用Groundplane_fit的函数, 在scan_callback 定义对象，在ground_segmentation.ccp 实现各函数
  const GroundSegmentationParams params_;
  
  public:
  groundplane_fit(const GroundSegmentationParams& params, pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn,pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn_org);//不用groundplane_fit::groundplane_fit
  void init_groundpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud);
  void estimate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud);
  void obstacle_ground_cloud_found(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr  obstacle_cloud,std::vector<int>* label);
  void filtering();
  pcl::PointCloud<pcl::PointXYZ>::Ptr  test_api();

 //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn_org;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn;
private:
// pcl::PointCloud<pcl::PointXYZ> ground_cloud;
// pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn_org;
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn;
 int test_flag;
MatrixXf normal;
float d;
float dist;
};

class scan_ground{
  const GroundSegmentationParams params_;

  public:
  // scan_ground(const GroundSegmentationParams& params,pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloudIn,pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloudIn_org,pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloudIn_sorted,std::vector<int>* labels,std::vector<double>* range_s);
  // void filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud);
  // void init_scanID();
  // void ground_point_detect(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud);
  // void point_segmentation();

  scan_ground(const GroundSegmentationParams& params, pcl::PointCloud<PointType>::Ptr laser_cloudIn, pcl::PointCloud<PointType>::Ptr laser_cloudIn_org, pcl::PointCloud<PointType>::Ptr laser_cloudIn_sorted, std::vector<int>* labels, std::vector<double>* range_s);
  void filtering(pcl::PointCloud<PointType>::Ptr obstacle_cloud);
  void init_scanID();
  void ground_point_detect(pcl::PointCloud<PointType>::Ptr ground_cloud,pcl::PointCloud<PointType>::Ptr obstacle_cloud);
  void point_segmentation(pcl::PointCloud<PointType>::Ptr segmented_cloud);

  private:
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn_org;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_cloud;
  

  pcl::PointCloud<PointType>::Ptr cloudIn_org;
  pcl::PointCloud<PointType>::Ptr cloudIn;
  pcl::PointCloud<PointType>::Ptr sorted_cloud;

  std::vector<int>* label;
  std::vector<double>* ranges;
  //////label definition： the label of projected point cloud(N_scan * horizon_scan)
  //-2: 无效点
  //-1: 地面点
  // 0: 初始值, 不存在点
  // 1： 不是地面点的有效点
  //////global_label:
  // std::vector<std::vector<int> > * global_label;//
  // std::vector<std::vector<int> > * ground_label;//
};



#endif // GROUND_SEGMENTATION_H_
