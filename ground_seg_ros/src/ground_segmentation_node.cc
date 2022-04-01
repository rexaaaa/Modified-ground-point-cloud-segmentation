#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include "ground_seg/ground_segmentation.h"
#include "std_msgs/String.h"
#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <fstream>
#include <string>


bool debug = true;
/////// for lidar_data
int pcd_num = 10;
int pcd_spf = 0;

std::vector<int> count_h(78750,0);
std::vector<int> count_v(78750,0);
std::vector<double> average_h(78750,0.0);
std::vector<double> average_v(78750,0.0);
////// 


pcl::PointCloud<pcl::PointXYZ> cloud_debug;
// #include <algorithm>
/*我们定义的分割节点的类*/
class SegmentationNode {

  ros::Publisher ground_pub_;      //地面点的发布者
  ros::Publisher obstacle_pub_;    //障碍物点的发布者
  ros::Publisher border_pub_;    //障碍物点的发布者
  ros::Publisher debug_pub_;    //障碍物点的发布者

  ros::Publisher ground_pub_PT;      //地面点的发布者
  ros::Publisher obstacle_pub_PT;    //障碍物点的发布者
  GroundSegmentationParams params_;//地面分割的参数

public:
  SegmentationNode(ros::NodeHandle& nh,
                   const std::string& ground_topic,
                   const std::string& obstacle_topic,
                   const std::string& border_topic,
                   const std::string& debug_topic,
                   const GroundSegmentationParams& params,
                   const bool& latch = false) : params_(params) {
    ground_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(ground_topic, 1, latch);
    obstacle_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(obstacle_topic, 1, latch);
    border_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(border_topic, 1, latch);
    debug_pub_   = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(debug_topic, 1, latch);

    ground_pub_PT = nh.advertise<pcl::PointCloud<PointType>>(ground_topic, 1, latch);
    obstacle_pub_PT = nh.advertise<pcl::PointCloud<PointType>>(obstacle_topic, 1, latch);

  }

  /*回调函数*/
  void scanCallback(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud,border_cloud;
    /*将我们设定的参数传递进去，这里就直接跳转到linefit_ground_segmentation之中*/
    GroundSegmentation segmenter(params_);
    /*定义一个是否属于地面点的标签，来进行区分*/
    std::vector<int> labels;
    /*这个是地面分割类中的主要函数，分割函数9*/
    std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(cloud, cloud, indices);
    
///////////////////////////////////////////////
    std::vector<float > LIDAR_EXTRINSIC={1.57, 0.0, 1.62, -0.5, 2.3, 0.0};
    pcl::PointCloud<pcl::PointXYZ> cloud_trans;
    Eigen::Matrix4f tran_matrix;
    tran_matrix = Eigen::Matrix4f::Identity();
    for(int i =0; i<3;i++)
      tran_matrix(i,3) = LIDAR_EXTRINSIC[i];
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisd(LIDAR_EXTRINSIC[3]/180.0*M_PI,
                                                  Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisd(LIDAR_EXTRINSIC[4]/180.0*M_PI,
                                                   Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisd(LIDAR_EXTRINSIC[5]/180.0*M_PI,
                                                 Eigen::Vector3d::UnitZ()));

    Eigen::Matrix3f rot_matrix;
    rot_matrix=yawAngle*pitchAngle*rollAngle;

    tran_matrix.block<3,3>(0,0) = rot_matrix;
    segmenter.transform_extrinsic(cloud,cloud_trans,tran_matrix);

///////////////////////////////////////////////
    segmenter.segment(cloud_trans, &labels);
    /*定义了两种点云，一个是地面点，一个是障碍物点*/
    
    ground_cloud.header = cloud_trans.header;
    obstacle_cloud.header = cloud_trans.header;
    border_cloud.header = cloud_trans.header;

    for (size_t i = 0; i < cloud_trans.size(); ++i) {
      border_cloud.push_back(cloud_trans[i]);
    }
    /*将整个的无序点云按照标签分配到不同的点云之中*/
    /*通过这里的标签对于是否属于地面点进行了划分*/
    for (size_t i = 0; i < cloud_trans.size(); ++i) {
      if (labels[i] == 1) ground_cloud.push_back(cloud_trans[i]);
      else obstacle_cloud.push_back(cloud_trans[i]);
    }
    // ROS_INFO("gd:%d",ground_cloud.size());
    /*将按照标签分类好的数据分配到不同的点云之中*/
    ground_pub_.publish(ground_cloud);
    obstacle_pub_.publish(obstacle_cloud);
    border_pub_.publish(border_cloud);
    // ROS_INFO("publish already");

  }
////////debug////////////////////////////////////////////
void scanCallback_debug(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    /*将我们设定的参数传递进去，这里就直接跳转到linefit_ground_segmentation之中*/
    GroundSegmentation segmenter(params_);
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud,border_cloud;
    /*定义一个是否属于地面点的标签，来进行区分*/
    std::vector<int> labels;
    /*这个是地面分割类中的主要函数，分割函数9*/
    if (debug == true){
      segmenter.segment(cloud, &labels);
      ground_cloud.header = cloud.header;
      obstacle_cloud.header = cloud.header;
      border_cloud.header = cloud.header;

      for (size_t i = 0; i < cloud.size(); ++i) {
        if (labels[i] == 1) ground_cloud.push_back(cloud[i]);
        else if ( labels[i] == 2) border_cloud.push_back(cloud[i]);
        else obstacle_cloud.push_back(cloud[i]);
      }

    }
    else {
      segmenter.segment(cloud_debug, &labels);
      ground_cloud.header = cloud_debug.header;
      obstacle_cloud.header = cloud_debug.header;
      border_cloud.header = cloud_debug.header;

      for (size_t i = 0; i < cloud_debug.size(); ++i) {
        if (labels[i] == 1) ground_cloud.push_back(cloud_debug[i]);
        else if ( labels[i] == 2) border_cloud.push_back(cloud_debug[i]);
        else obstacle_cloud.push_back(cloud_debug[i]);
      }
    }
    /*定义了两种点云，一个是地面点，一个是障碍物点*/
    /*将整个的无序点云按照标签分配到不同的点云之中*/
    /*通过这里的标签对于是否属于地面点进行了划分*/
    // ROS_INFO("gd:%d",ground_cloud.size());
    /*将按照标签分类好的数据分配到不同的点云之中*/
    ground_pub_.publish(ground_cloud);
    obstacle_pub_.publish(obstacle_cloud);
    border_pub_.publish(border_cloud);
    if (debug == true)
    {
      cloud_debug = cloud;
      debug = false;
    }
  }
//  void scanCallback2(const sensor_msgs::PointCloud2::ConstPtr& in_cloud_msg) {
   
//     /*定义了两种点云，一个是地面点，一个是障碍物点*/
//     pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
//     pcl::fromROSMsg(*in_cloud_msg, ground_cloud);
//     ground_cloud.header = cloud.header;
//     obstacle_cloud.header = cloud.header;
 
//     /*将按照标签分类好的数据分配到不同的点云之中*/
//     //ground_pub_.publish(ground_cloud);
//     ROS_INFO("wtf!");
//   }
/////////////////////////////////////拟合平面点的scan call back
/////////////////////////////////////
 void scanCallback2(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
   ROS_INFO("recieve");
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    std::vector<int> labels;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn_org(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *laserCloudIn);
    pcl::fromROSMsg(*cloud, *laserCloudIn_org);
    
    ground_cloud->header = laserCloudIn->header;
    obstacle_cloud->header = laserCloudIn->header;
    groundplane_fit ground_chy(params_, laserCloudIn, laserCloudIn_org);
    ground_chy.filtering();
    // ////first extract the ground point from original point cloud data
    ground_chy.init_groundpoint(ground_cloud);
    // //// in the iteration and estimate plane to found the ground point
    for(int i=0; i< params_.num_iter;++i){
      ground_chy.estimate_plane(ground_cloud);
      ground_cloud->clear();
      obstacle_cloud->clear();
      ground_chy.obstacle_ground_cloud_found(ground_cloud,obstacle_cloud,&labels);
    } 
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> fp_ms = end - start;
  std::cout << "Done! Took " << fp_ms.count() << "ms\n";
     ground_pub_.publish(*ground_cloud);
     obstacle_pub_.publish(*obstacle_cloud);
  }

 void scanCallback3(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    ROS_INFO("recieve");
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    std::vector<int> labels;
    std::vector<double> range;
 
    // pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn_org(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudIn_org(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr ground_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr obstacle_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr sorted_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr seg_cloud(new pcl::PointCloud<PointType>());

    pcl::fromROSMsg(*cloud, *laserCloudIn);
    pcl::fromROSMsg(*cloud, *laserCloudIn_org);
    
    ground_cloud->header   = laserCloudIn->header;
    obstacle_cloud->header = laserCloudIn->header;
    sorted_cloud->header   = laserCloudIn->header;
    seg_cloud->header      = laserCloudIn->header;

    scan_ground ground_chy(params_, laserCloudIn, laserCloudIn_org, sorted_cloud,&labels,&range);
    ground_chy.filtering(obstacle_cloud);
    ROS_INFO("test1");
    // ////first extract the ground point from original point cloud data
    ground_chy.init_scanID();
    ROS_INFO("test2");
    ground_chy.ground_point_detect(ground_cloud,obstacle_cloud);
    ROS_INFO("test3");
    ground_chy.point_segmentation(seg_cloud);
    ROS_INFO("test4");
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "Done! Took " << fp_ms.count() << "ms\n";
    // ground_pub_.publish(*ground_cloud);
        // ground_pub_PT.publish(*ground_cloud);
       ground_pub_PT.publish(*seg_cloud);
    // ground_pub_PT.publish(*sorted_cloud);//sorted_cloud: 如果不存在点的index,则不会显示
      obstacle_pub_.publish(*ground_cloud);
  }


  void scanCallback_txt(const pcl::PointCloud<PointType>& cloud) {
    std::ofstream fout;
    double horizon_angle;
    double vectical_angle;
    double simulation_time=0.01;
    std::vector<int> count_h(78750,0);
    std::vector<int> count_v(78750,0);
    // ROS_INFO("point_size:%d", cloud.points.size());
    std::vector<std::string> file_str(10, "/home/yihang/carla_lidar_data/");
    file_str[0]+="scan_1.txt";
    file_str[1]+="scan_2.txt";
    file_str[2]+="scan_3.txt";
    file_str[3]+="scan_4.txt";
    file_str[4]+="scan_5.txt";
    file_str[5]+="scan_6.txt";
    file_str[6]+="scan_7.txt";
    file_str[7]+="scan_8.txt";
    file_str[8]+="scan_9.txt";
    file_str[9]+="scan_10.txt";
    fout.setf(ios::fixed,ios::floatfield);
    fout.precision(6);
      if (pcd_num > 0 ){
 
       fout.open(file_str[10-pcd_num].c_str());
       for(int i = 0; i < cloud.points.size (); ++i){
          if ( isnan(cloud.points[i].x) || isnan(cloud.points[i].y) || isnan(cloud.points[i].z)){
            fout << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << " " << std::endl;  
          }
          else{
            horizon_angle  = atan2(cloud.points[i].y,cloud.points[i].x)*180/M_PI;
            vectical_angle = atan2(cloud.points[i].z,sqrt(cloud.points[i].x*cloud.points[i].x+cloud.points[i].y*cloud.points[i].y))*180/M_PI;
            simulation_time=0.01;
            fout << simulation_time << "," << horizon_angle << "," << vectical_angle << " " << std::endl;  
          }
        }
        fout.close();
        
        pcd_num--;

        if (pcd_num ==0){
        ROS_INFO("write finish");
      }

      }
      


      file_str[9] = "/home/yihang/carla_lidar_data/scan_spf.txt";
      if(pcd_spf==450){
        fout.open(file_str[9].c_str());
       for(int i = 0; i < cloud.points.size (); ++i){
          if ( isnan(cloud.points[i].x) || isnan(cloud.points[i].y) || isnan(cloud.points[i].z)){
            fout << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << " " << std::endl;  
          }
          else{
            horizon_angle  = atan2(cloud.points[i].y,cloud.points[i].x)*180/M_PI;
            vectical_angle = atan2(cloud.points[i].z,sqrt(cloud.points[i].x*cloud.points[i].x+cloud.points[i].y*cloud.points[i].y))*180/M_PI;
            simulation_time=0.01;
            fout << simulation_time << "," << horizon_angle << "," << vectical_angle << " " << std::endl; 
          } 
        }
        fout.close();
      }

      if (pcd_spf == 450){
        ROS_INFO("write spf finish ");
      }

      
      if(pcd_spf<=1000){
        // fout.open(file_str[8].c_str());
       for(int i = 0; i < cloud.points.size (); ++i){
          if ( isnan(cloud.points[i].x) || isnan(cloud.points[i].y) || isnan(cloud.points[i].z)){
            // fout << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << " " << std::endl;  
          }
          else{
            horizon_angle  = atan2(cloud.points[i].y,cloud.points[i].x)*180/M_PI;
            vectical_angle = atan2(cloud.points[i].z,sqrt(cloud.points[i].x*cloud.points[i].x+cloud.points[i].y*cloud.points[i].y))*180/M_PI;
            average_h[i]= average_h[i]*(count_h[i])/(count_h[i]+1)+horizon_angle/(count_h[i]+1);
            average_v[i]= average_v[i]*(count_h[i])/(count_h[i]+1)+vectical_angle/(count_h[i]+1);
            count_h[i]++;
            count_v[i]++;
            //fout << simulation_time << "," << horizon_angle << "," << vectical_angle << " " << std::endl; 
          } 
        }
        
      }
      else if (pcd_spf == 1001){

        file_str[8] = "/home/yihang/carla_lidar_data/scan_average.txt";
        fout.open(file_str[8].c_str());
        for(int i=0; i<78750; ++i){
          fout << simulation_time << "," << average_h[i] << "," << average_v[i] << " " << std::endl;
        }
        fout.close();
        ROS_INFO("write average finish ");
      }


      pcd_spf++;


      
    // save pcd
    // if (pcd_num==10){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_2.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
    // else if (pcd_num==9){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_2.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
    // else if (pcd_num==8){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_3.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
    // else if (pcd_num==7){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_4.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
    // else if (pcd_num==6){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_5.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
    // else if (pcd_num==5){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_6.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
    // else if (pcd_num==4){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_7.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
    // else if (pcd_num==3){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_8.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
    // else if (pcd_num==2){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_9.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
    // else if (pcd_num==1){
    //   pcl::io::savePCDFileASCII ("/home/yihang/carla_lidar_data/scan_10.pcd", cloud);
    //   ROS_INFO("save worked");
    //   pcd_num--;
    // }
  }

};

/*主函数，给出了ros的接口，跳转到linefir_ground_segmentation之中*/
int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_segmentation");
  /*调用glog中的函数来初始化日志，为后面做准备*/
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh("~");

  // Do parameter stuff.
  /*参数设定，可以读入我们在yaml中设定的参数*/
  GroundSegmentationParams params; //param("string", variant, default variant)
  nh.param("visualize", params.visualize, params.visualize);
  nh.param("n_bins", params.n_bins, params.n_bins);
  nh.param("n_segments", params.n_segments, params.n_segments);
  nh.param("max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
  nh.param("max_slope", params.max_slope, params.max_slope);
  nh.param("long_threshold", params.long_threshold, params.long_threshold);
  nh.param("max_long_height", params.max_long_height, params.max_long_height);
  nh.param("max_start_height", params.max_start_height, params.max_start_height);
  nh.param("sensor_height", params.sensor_height, params.sensor_height);
  nh.param("line_search_angle", params.line_search_angle, params.line_search_angle);
  nh.param("n_threads", params.n_threads, params.n_threads);
  nh.param("selection",params.selection, params.selection);
  nh.param("sensor_model",params.sensor_model,params.sensor_model);
  nh.param("num_iter",params.num_iter,params.num_iter);
  nh.param("num_lpr" ,params.num_lpr ,params.num_lpr);
  nh.param("th_seeds",params.th_seeds,params.th_seeds);
  nh.param("th_dist",params.th_dists,params.th_dists);
  nh.param("N_scan",params.N_scan,params.N_scan);
  nh.param("horizon_scan",params.horizon_scan,params.horizon_scan);
  nh.param("ang_res_x",params.ang_res_x,params.ang_res_x);
  nh.param("ang_res_y",params.ang_res_y,params.ang_res_y);
  nh.param("ang_bottom",params.ang_bottom,params.ang_bottom);
  nh.param("groundscan_Ind",params.groundscan_Ind,params.groundscan_Ind);
  nh.param("minrange",params.minrange,params.minrange);
  nh.param("ground_N",params.ground_N,params.ground_N);

  nh.param("out_slope", params.out_slope, params.out_slope);
  // nh.param("r_min_square",params.r_min_square,params.r_min_square);
  // nh.param("r_max_square",params.r_max_square,params.r_max_square);

  // Params that need to be squared.
  /*得到平方后的数据信息，分别是r_min,r_max和max_fit_error*/
  double r_min, r_max, max_fit_error;
  if (nh.getParam("r_min", r_min)) {
    params.r_min_square = r_min*r_min;
  }
  if (nh.getParam("r_max", r_max)) {
    params.r_max_square = r_max*r_max;
  }
  if (nh.getParam("max_fit_error", max_fit_error)) {
    params.max_error_square = max_fit_error * max_fit_error;
  }

  std::string ground_topic, obstacle_topic, input_topic,border_topic, debug_topic;
  bool latch;
  /*input topic是需要我们自己设定的，根据自己雷达的数据而定*/
  nh.param<std::string>("input_topic", input_topic, "input_cloud");
  nh.param<std::string>("ground_output_topic", ground_topic, "ground_cloud");
  nh.param<std::string>("obstacle_output_topic", obstacle_topic, "obstacle_cloud");
  nh.param<std::string>("border_output_topic", border_topic, "cloud_trans");
  nh.param<std::string>("debug_input_topic", debug_topic, "cloud_debug");
  nh.param("latch", latch, false);

  // Start node.
  /*我们初始化类，从类中调到别的地方*/
  SegmentationNode node(nh, ground_topic, obstacle_topic, border_topic, debug_topic, params, latch);
  ros::Subscriber cloud_sub;
  
  /*从这里的回调函数，开始进入主要的函数中*/
  switch(params.selection){
  case 0: {cloud_sub = nh.subscribe(input_topic, 50, &SegmentationNode::scanCallback_debug,   &node) ;break;}
  case 1: {cloud_sub = nh.subscribe(input_topic, 1000, &SegmentationNode::scanCallback,   &node) ;break;}
  case 2: {cloud_sub = nh.subscribe(input_topic, 50, &SegmentationNode::scanCallback2,  &node) ;break;}
  case 3: {cloud_sub = nh.subscribe(input_topic, 50, &SegmentationNode::scanCallback3,  &node) ;break;}
  case 4: {cloud_sub = nh.subscribe(input_topic, 50, &SegmentationNode::scanCallback,   &node) ;break;}
  case 5: {cloud_sub = nh.subscribe(input_topic, 50, &SegmentationNode::scanCallback_txt,   &node) ;break;}
  }
  ros::spin();
  //ros::spinOnce();
}