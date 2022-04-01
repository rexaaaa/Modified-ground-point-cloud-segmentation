#include "ground_seg/ground_segmentation.h"

#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>
//////////////////////////////////////////////////////////////////////////////
/*可视化点云*/
void GroundSegmentation::visualizePointCloud(const PointCloud::ConstPtr& cloud,
                                             const std::string& id) {
  viewer_->addPointCloud(cloud, id, 0);  //   /*将点云和id添加到可视化显示器之中*/
}
/*显示地面线*/
void GroundSegmentation::visualizeLines(const std::list<PointLine>& lines) {
  size_t counter = 0;
  /*遍历显示地面线*/
  for (auto it = lines.begin(); it != lines.end(); ++it) {
    viewer_->addLine<pcl::PointXYZ>(it->first, it->second, std::to_string(counter++));
  }
}

/*
  可视化:
  在这里，我们将pcl_viewer中需要设定的内容都设定好
*/
void GroundSegmentation::visualize(const std::list<PointLine>& lines,
                                   const PointCloud::ConstPtr& min_cloud,
                                   const PointCloud::ConstPtr& ground_cloud,
                                   const PointCloud::ConstPtr& obstacle_cloud) {
  viewer_->setBackgroundColor (0, 0, 0);
  viewer_->addCoordinateSystem (1.0);
  viewer_->initCameraParameters ();
  viewer_->setCameraPosition(-2.0, 0, 2.0, 1.0, 0, 0);
  visualizePointCloud(min_cloud, "min_cloud");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                             0.0f, 1.0f, 0.0f,
                                             "min_cloud");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             2.0f,
                                             "min_cloud");
  visualizePointCloud(ground_cloud, "ground_cloud");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                             1.0f, 0.0f, 0.0f,
                                             "ground_cloud");
  visualizePointCloud(obstacle_cloud, "obstacle_cloud");
  visualizeLines(lines);
  while (!viewer_->wasStopped ()){
      viewer_->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////

/*地面分割的构造函数及各种selection的重载*///////////////////////////////
GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params) :
    params_(params),//params_ 是GroundSegmentation 类里面包含的GroundSegmentationParams对象
    /////////segments_: vector 存储Segment 对象,std::vector 初始化segments_(个数，元素初值)
    segments_(params.n_segments, Segment(params.n_bins,//Segments class 定义在segment.h, segment.ccp 那个是补充库，里面的segments是构造函数
                                         params.max_slope,
                                         params.max_error_square,
                                         params.long_threshold,
                                         params.max_long_height,
                                         params.max_start_height,
                                         params.sensor_height)) {
  if (params.visualize) viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");

  bin_ground.resize(params.n_segments,std::vector<int> (params.n_bins,0));

      }


/*地面分割的分割函数*/
void GroundSegmentation::segment(const PointCloud& cloud, std::vector<int>* segmentation) {
  // cloud: input PointCloud
  // segmentation: int 的 vector 的指针 
  /*初始化一些比较基础的东西*/
  std::cout << "Segmenting cloud with " << cloud.size() << " points...\n";
  //std::chrono::high_resolution_clock 系统启动之后的计时时钟，不是绝对时间，只用于求相对时间
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  segmentation->clear();
  segmentation->resize(cloud.size(), 0);
  bin_index_.resize(cloud.size());//  std::vector<std::pair<int, int> > bin_index_;
  segment_coordinates_.resize(cloud.size());//std::vector<Bin::MinZPoint> segment_coordinates_;
  // ROS_INFO("bin_ground_size:%d",bin_ground.size());
  /*插入点云数据*/
  // ROS_INFO("insert begin");
  insertPoints(cloud);//多线程处理数据
  // ROS_INFO("insert ok");
  /*定义地面点的线的参数*/
  std::list<PointLine> lines;
  /*根据我们的设定来决定是否进行可视化的展示*/
  if (params_.visualize) {
    getLines(&lines);
  }
  else {
    getLines(NULL);//getLines,对每个segment里面的点进行拟合，得到所有的线存在lines_里，lines_的存储没有按segment存
  }                //lines_ 存着首尾两个拟合点的(d,z),根据第一个和最后一个的被拟合点的d算的z
  // ROS_INFO("get line ok");                       
  /*对于传入的分割进行细分*/
  /*从这里可以看到对于点云属于障碍物还是属于地面点进行了标签的划分*/
  assignCluster(segmentation);
  // ROS_INFO("segment ok");     
  /*如果是进行了可视化的操作，则进行一下的操作*/
  if (params_.visualize) {
    // Visualize.
    PointCloud::Ptr obstacle_cloud(new PointCloud());
    // Get cloud of ground points.
    PointCloud::Ptr ground_cloud(new PointCloud());
    // Get cloud of border points.
    // PointCloud::Ptr border_cloud(new PointCloud());
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (segmentation->at(i) == 1) {ground_cloud->push_back(cloud[i]);}
      else {
      obstacle_cloud->push_back(cloud[i]);
      }

    }
    PointCloud::Ptr min_cloud(new PointCloud());
    getMinZPointCloud(min_cloud.get());
    visualize(lines, min_cloud, ground_cloud, obstacle_cloud);
  }
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> fp_ms = end - start;
  std::cout << "line_fit Done! Took " << fp_ms.count() << "ms\n";


  // begin border detection
  // border_detection(bin_ground, segmentation);
}

/* 边沿bin检测 */
void GroundSegmentation::border_detection(std::vector<std::vector<int> >& bin_ground, std::vector<int>* segmentation){
double ratio =0;
int border_bin = -1;

for(int i=0; i<bin_ground.size();++i){
  border_bin = -1;
  for (int k=0; k< bin_ground[0].size();++k){
    // if(bin_ground[i][k-1] != 0){
    //   // ROS_INFO("size:%d",bin_ground[i][k-1]);
    // }
    
    if ( k==0 || bin_ground[i][k-1] ==0 ){
      continue;  
    }
    ratio = fabs(bin_ground[i][k] - bin_ground[i][k-1])/bin_ground[i][k-1];
    // ROS_INFO("ratio:%f",ratio);
    if (ratio > 0.7){
      border_bin = k;
      break;
    }
  }
  if(border_bin != -1){
    // ROS_INFO("found");
    for (int j=0; j< segmentation->size(); ++j){
      if(bin_index_[j].first == i && bin_index_[j].second == border_bin){
        // if(segmentation->at(j) != 1){
        //   segmentation->at(j) = 2;
        // }
        segmentation->at(j) = 2;
      }
      if(bin_index_[j].first == i && bin_index_[j].second == (border_bin+1) ){
        // if(segmentation->at(j) != 1){
        //   segmentation->at(j) = 2;
        // }
        segmentation->at(j) = 2;
      }
    }
  }
  else{
    // ROS_INFO("size:%d,%d",bin_ground.size(),bin_ground[0].size());
     //ROS_INFO("size:%d",segmentation->size());
  }
}

}

void GroundSegmentation::transform_extrinsic(const pcl::PointCloud<pcl::PointXYZ> & cloud_valid, pcl::PointCloud<pcl::PointXYZ> & cloud_trans,Eigen::Matrix4f tran_matrix){
  for(auto p : cloud_valid){
      if(p.z > 0.5)
        continue;
      pcl::PointXYZ thisPoint;

      thisPoint.x = tran_matrix(0, 0) * p.x
          + tran_matrix(0, 1) * p.y
          + tran_matrix(0, 2) * p.z
          + tran_matrix(0, 3);
      thisPoint.y = tran_matrix(1, 0) * p.x
          + tran_matrix(1, 1) * p.y
          + tran_matrix(1, 2) * p.z
          + tran_matrix(1, 3);
      thisPoint.z = tran_matrix(2, 0) * p.x
          + tran_matrix(2, 1) * p.y
          + tran_matrix(2, 2) * p.z
          + tran_matrix(2, 3);

      cloud_trans.points.push_back(thisPoint);
    }
    cloud_trans.header = cloud_valid.header;
    cloud_trans.height = cloud_valid.height;
    cloud_trans.width = cloud_valid.width;


}

/*获取到线*/
void GroundSegmentation::getLines(std::list<PointLine> *lines) {
  std::mutex line_mutex;
  std::vector<std::thread> thread_vec(params_.n_threads);
  unsigned int i;
  for (i = 0; i < params_.n_threads; ++i) {
    const unsigned int start_index = params_.n_segments / params_.n_threads * i;
    const unsigned int end_index = params_.n_segments / params_.n_threads * (i+1);
    thread_vec[i] = std::thread(&GroundSegmentation::lineFitThread, this,
                                start_index, end_index, lines, &line_mutex);
  }
  for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();
  }

}

/*这里是获取线的操作*/
void GroundSegmentation::lineFitThread(const unsigned int start_index,
                                       const unsigned int end_index,
                                       std::list<PointLine> *lines, std::mutex* lines_mutex) {
  const bool visualize = lines;
  const double seg_step = 2*M_PI/params_.n_segments;

  // if(params_.selection==4)
  // {
  //   seg_step = 2*M_PI/3/params_.n_segments;//kitti
  // }
  // else
  // {
  //   seg_step = 2*M_PI/params_.n_segments;//kitti
  // }


  double angle = -M_PI + seg_step/2 + seg_step * start_index;
  for (unsigned int i = start_index; i < end_index; ++i) {
    segments_[i].fitSegmentLines();
    // Convert lines to 3d if we want to.
    /*这里也是可视化的一些操作*/
    if (visualize) {
      std::list<Segment::Line> segment_lines;
      segments_[i].getLines(&segment_lines);
      for (auto line_iter = segment_lines.begin(); line_iter != segment_lines.end(); ++line_iter) {
        const pcl::PointXYZ start = minZPointTo3d(line_iter->first, angle);
        const pcl::PointXYZ end = minZPointTo3d(line_iter->second, angle);
        lines_mutex->lock();
        lines->emplace_back(start, end);
        lines_mutex->unlock();
      }

      angle += seg_step;
    }
  }
}

void GroundSegmentation::getMinZPointCloud(PointCloud* cloud) {
  const double seg_step = 2*M_PI /params_.n_segments;
  // if(params_.selection==4)
  // {
  //   seg_step = 2*M_PI/3/params_.n_segments;//kitti
  // }
  // else
  // {
  //   seg_step = 2*M_PI/params_.n_segments;//kitti
  // }
  double angle = -M_PI + seg_step/2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter) {
      const pcl::PointXYZ min = minZPointTo3d(bin_iter->getMinZPoint(), angle);
      cloud->push_back(min);
    }

    angle += seg_step;
  }
}

/*根据传入的二维点，也可以通过算出的angle将二维点转化为三维点，主要是x-y平面内的变换*/
pcl::PointXYZ GroundSegmentation::minZPointTo3d(const Bin::MinZPoint &min_z_point,
                                                const double &angle) {
  pcl::PointXYZ point;
  point.x = cos(angle) * min_z_point.d;
  point.y = sin(angle) * min_z_point.d;
  point.z = min_z_point.z;
  return point;
}

/*分配集群，将传入的分割进行簇的划分*/
void GroundSegmentation::assignCluster(std::vector<int>* segmentation) {
  std::vector<std::thread> thread_vec(params_.n_threads);
  const size_t cloud_size = segmentation->size();
  for (unsigned int i = 0; i < params_.n_threads; ++i) {
    const unsigned int start_index = cloud_size / params_.n_threads * i;
    const unsigned int end_index = cloud_size / params_.n_threads * (i+1);
    thread_vec[i] = std::thread(&GroundSegmentation::assignClusterThread, this,
                                start_index, end_index, segmentation);
  }
  for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();
  }
}

/*执行分配集群的线程操作*/
void GroundSegmentation::assignClusterThread(const unsigned int &start_index,
                                             const unsigned int &end_index,
                                             std::vector<int> *segmentation) {
  /*通过传入的segments的个数，得到划分segment的步长，也就是δ值*/
  const double segment_step= 2*M_PI/params_.n_segments ;
  // if(params_.selection==4)
  // {
  //   segment_step = 2*M_PI/3/params_.n_segments;//kitti
  // }
  // else
  // {
  //   segment_step = 2*M_PI/params_.n_segments;//kitti
  // }
  /*进行遍历操作*/
  /*对于每一个点进行处理，根据距离判断是否属于远离地面的点*/
  for (unsigned int i = start_index; i < end_index; ++i) {
    /*首先，得到每一个三维点的二维映射*/
    Bin::MinZPoint point_2d = segment_coordinates_[i];
    /*得到第i个bin的第一个值，作为分割的索引*/ //第i个点的segment index
    const int segment_index = bin_index_[i].first;
    /*判定分割的index是需要大于0的*/
    if (segment_index >= 0) {
      /*计算处两个点到线的垂直距离*/
      /*将点投影到直线上*/
      // ROS_INFO("i:%d",i);
      std::pair<double,bool> result = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);
      double dist = result.first;
      bool border_flag = result.second;
      // Search neighboring segments.
      /*搜索相邻的分割*/
      int steps = 1;
      /*根据划分好的segment来不断的进行数据的处理*/
      /*在设定的步长的情况下在搜索框内能有几个步长*/
      while (dist == -1 && steps * segment_step < params_.line_search_angle) {
        // Fix indices that are out of bounds.
        /*修复超出范围的索引*/
        int index_1 = segment_index + steps;//进行正向步长的搜索
        while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;
        int index_2 = segment_index - steps;//进行反向步长的索引
        while (index_2 < 0) index_2 += params_.n_segments;
        // Get distance to neighboring lines.
        /*算出根据正反双向最近搜索的点到线段投影的距离*/
        std::pair<double,bool> result1 = segments_[index_1].verticalDistanceToLine(point_2d.d, point_2d.z);
        std::pair<double,bool> result2 = segments_[index_2].verticalDistanceToLine(point_2d.d, point_2d.z);

        const double dist_1   = result1.first;
        const double dist_2   = result2.first;
        const bool   border_1 = result1.second;
        const bool   border_2 = result2.second;
        // Select larger distance if both segments return a valid distance.
        /*如果两个分割都返回有效距离，则选择更大的距离*/
        if (dist_1 > dist) {
          dist = dist_1;
          border_flag =border_1;
        }
        if (dist_2 > dist) {
          dist = dist_2;
          border_flag =border_2;
        }
        /*不断增加步长，一直持续下去，直到跳出循环，这样可以保证比较公平的遍历到里面的点*/
        ++steps;
      }
      /*这里是进行标签的设定*/
      // ROS_INFO("dist:%f",dist);
      if (dist < params_.max_dist_to_line && dist != -1) {
        /*这里是对于是否属于地面点的判定*/
        segmentation->at(i) = 1;
        bin_ground[bin_index_[i].first][bin_index_[i].second]++;
      }
      else if ( border_flag == true){
        // segmentation->at(i) = 2;
        // bin_ground[bin_index_[i].first][bin_index_[i].second]++;

      }
    }
  }
}

/*获取最小z点的点云数据*/
void GroundSegmentation::getMinZPoints(PointCloud* out_cloud) {
  /*得到分割的步长，以及bins的步长*/
  const double seg_step = 2*M_PI/params_.n_segments;//kitti

  // if(params_.selection==4)
  // {
  //   seg_step = 2*M_PI/3/params_.n_segments;//kitti
  // }
  // else
  // {
  //   seg_step = 2*M_PI/params_.n_segments;//kitti
  // }

  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
      / params_.n_bins;
  /*得到最小的r*/
  const double r_min = sqrt(params_.r_min_square);
  double angle = -M_PI + seg_step/2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    double dist = r_min + bin_step/2;
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter) {
      pcl::PointXYZ point;
      if (bin_iter->hasPoint()) {
        /*对于bin_iter进行最小z点的提取*/
        Bin::MinZPoint min_z_point(bin_iter->getMinZPoint());
        point.x = cos(angle) * min_z_point.d;
        point.y = sin(angle) * min_z_point.d;
        point.z = min_z_point.z;
        /*将point放入到out_cloud之中*/
        out_cloud->push_back(point);
      }
      /*按照步长增加dist*/
      dist += bin_step;
    }
    /*按照划分的步长进行角度的增加*/
    angle += seg_step;
  }
}


/*插入点云*/
void GroundSegmentation::insertPoints(const PointCloud& cloud) {
  std::vector<std::thread> threads(params_.n_threads);//线程个数n_threads 在launch 的segmentation_params.yaml
  const size_t points_per_thread = cloud.size() / params_.n_threads;
  // Launch threads.
  /*根据我们设定的数目来将整个的点云分为几个部分开始处理，利用多线程来处理*/
  for (unsigned int i = 0; i < params_.n_threads - 1; ++i) {
    const size_t start_index = i * points_per_thread;
    const size_t end_index = (i+1) * points_per_thread - 1;
    threads[i] = std::thread(&GroundSegmentation::insertionThread, this,
                             cloud, start_index, end_index);//需要执行的函数的指针insertionThread
  }                                                                                          //cloud, start_index,end_index 是insertionThread 需要的形参
  // Launch last thread which might have more points than others.
  /*启动最后一个可能含有更多点云的线程*/
  const size_t start_index = (params_.n_threads - 1) * points_per_thread;
  const size_t end_index = cloud.size() - 1;
  threads[params_.n_threads - 1] =
      std::thread(&GroundSegmentation::insertionThread, this, cloud, start_index, end_index);//this是GroundSegmentation的对象
  // Wait for threads to finish.
  for (auto it = threads.begin(); it != threads.end(); ++it) {
    it->join();//等待进程结束
  }
}

/*线程启动中会执行的函数*/  //计算点云里每个点的bin_index 和segment_index
void GroundSegmentation::insertionThread(const PointCloud& cloud,
                                         const size_t start_index,
                                         const size_t end_index) {
  /*同样的先算步长，然后再进行其他的操作*/
  const double segment_step= 2*M_PI/params_.n_segments;
  int count=0;
  // if(params_.selection==4)
  // {
  //   segment_step = 2*M_PI/3/params_.n_segments;//kitti
  // }
  // else
  // {
  //   segment_step = 2*M_PI/params_.n_segments;//kitti
  // }
  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square)) / params_.n_bins;//每个bins的范围，params_.r_max_square是总的r_max, params_.r_min_square是总的r_min
  const double r_min = sqrt(params_.r_min_square);
  /*对于起始索引和终止索引进行遍历*/
  for (unsigned int i = start_index; i < end_index; ++i) {
    pcl::PointXYZ point(cloud[i]);
    /*这里是算模长*/
    const double range_square = point.x * point.x + point.y * point.y;
    const double range = sqrt(range_square);
    /*判断模场是否属于最小值和最大值之间*/
    // if(range_square>params_.r_max_square)
    // {
    //   ROS_INFO("range_over");
    // }
    // if( isnan(point.x) || isnan(point.y) || isnan(point.z)){
    //   ROS_INFO("NAN exist------------>");
    //   if(range_square> params_.r_max_square){
    //     ROS_INFO("range_max_square:%f",range_square);
    //   }
    //   if (range_square < params_.r_min_square){
    //     ROS_INFO("range_min_square:%f",range_square);
    //   }
    // }
    if (range_square < params_.r_max_square && range_square > params_.r_min_square ) {
       /*得到角度*/
      if( isnan(range_square) ){ROS_INFO("wrong");}
      const double angle = std::atan2(point.y, point.x);//[-180,180]
      /*根据模场和角度来算出bin的索引以及划分的索引*/
      const unsigned int bin_index = (range - r_min) / bin_step;
      const unsigned int segment_index = (angle + M_PI) / segment_step;//angle+M_PI [0,360]
      const unsigned int segment_index_clamped = segment_index == params_.n_segments ? 0 : segment_index;
      /*对于设定的属于bin和划分中的集合进行数据的添加*/
      segments_[segment_index_clamped][bin_index].addPoint(range, point.z);//segments_[segment_index_clamped] 就是第segment_index_clamped个segments
                                                                                                                                                                 //addPoint(range, point.z)只添加z较小的点
      /*将后面的数据作为一个元组全部传递到bin_index之中*/
      bin_index_[i] = std::make_pair(segment_index_clamped, bin_index);//bin_index[i]：第i个点的segment_index 和bin_index
      // count++;
    }
    else {
      bin_index_[i] = std::make_pair<int, int>(-1, -1);//(-1,-1)无效点
    }
    /*获取到划分坐标为最小z点的坐标和range*/
    segment_coordinates_[i] = Bin::MinZPoint(range, point.z);//segment_coordinates_ 存的是Bin::MinZPoint类型的点(d,z)，
                                                                                                                            //(d,z)是z较小的(d,z)
  }
  // ROS_INFO("count:%d",count);
}

groundplane_fit::groundplane_fit(const GroundSegmentationParams& params, pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn,pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn_org):
params_(params),
cloudIn(laserCloudIn),
cloudIn_org(laserCloudIn_org),
dist(0),
d(0),
test_flag(0)
{
}

bool point_z_cmp(pcl::PointXYZ a,pcl::PointXYZ b){
return (a.z<b.z);
}

bool point_type_z_cmp(PointType a,PointType b){
return (a.z<b.z);
}


void groundplane_fit::filtering(){
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloudIn, *cloudIn, indices);
  sort(cloudIn->points.begin(),cloudIn->points.end(),point_z_cmp);//sort: 将laserCLoudIn中地址为begin到end的数据，按照function point_cmp排序
  pcl::PointCloud<pcl::PointXYZ>::iterator it = cloudIn->points.begin();//iterator 是一种地址
  for(int i=0;i<cloudIn->points.size();i++){
        if(cloudIn->points[i].z < -1.5*params_.sensor_height){
            it++;
        }else{
            break;
        }
    }
  cloudIn->points.erase(cloudIn->points.begin(),it);//删除laserCloudIn中地址为begin()到it之前的点
}

void groundplane_fit::init_groundpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud){
double sum=0;
double mean=0;
int          cn=0;
for(int i=0;i<params_.num_lpr&&i<cloudIn->points.size();++i){
  sum+=cloudIn->points[i].z;
  cn++;
}
mean=cn!=0?sum/cn:0;
ground_cloud->clear();
//////find ground point//////////
for(int i=0;i<cloudIn->points.size();++i){
if(cloudIn->points[i].z<mean+params_.th_seeds){
  ground_cloud->points.push_back(cloudIn->points[i]);
}
}
}


  void groundplane_fit::estimate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud){
  Eigen::Matrix3f cov;//3*3 协方差矩阵
  Eigen::Vector4f pc_mean;//4*1矩阵 最后一个元素是1
  pcl::computeMeanAndCovarianceMatrix(*ground_cloud, cov, pc_mean);
  // Singular Value Decomposition: SVD Eigen库
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
// use the least singular vector as normal//特征值从小到大排列，对应的特征向量
    normal = (svd.matrixU().col(2));
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
        // according to normal.T*[x,y,z] = -d
    d = -(normal.transpose()*seeds_mean)(0,0);
    // set distance threhold to `th_dist - d`
    dist = params_.th_dists - d;
  }


//   void groundplane_fit::obstacle_ground_cloud_found(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud,std::vector<int>* label){
//     MatrixXf points(cloudIn_org->points.size(),3);//n*3 矩阵
//     label->resize(cloudIn_org->points.size(), 0);
//     int j =0;
//     // pcl::PointCloud<pcl::PointXYZ>::iterator p;// pcl::PointCloud<pcl::PointXYZ>::iterator   //iterator不能用做Ptr
//     //     for(p=cloudIn_org->points.begin();p!=cloudIn_org->points.end();++p){
//     //       points.row(j)<<p->pointx,p->y,p->z;
//     //     }
//         for(auto p:cloudIn_org->points){
//             points.row(j++)<<p.x,p.y,p.z;//(x,y,z)
//         }
//     //////////////////////////////////////a*x+b*y+c*z结果///////////////////////
//      VectorXf result = points*normal;    
//      for(int r=0;r<result.rows();++r){
//             if(result[r]<dist){//a*x+b*y+c*z+d< the_dist, 点到面的距离a*x+b*y+c*z+d
//                 ground_cloud->points.push_back(cloudIn_org->points[r]);
//                 label->at(r) = 1;
//             }else{
//                 label->at(r) = -1;
//                 obstacle_cloud->points.push_back(cloudIn_org->points[r]);
//             }
//         }

//     // ROS_INFO("Wrong");
//     if(ground_cloud->points.size()==0){
//         ROS_INFO("Wrong");
// } 
//   }


//去掉无效点的cloudIn比较好
 void groundplane_fit::obstacle_ground_cloud_found(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud,std::vector<int>* label){
    MatrixXf points(cloudIn->points.size(),3);//n*3 矩阵
    label->resize(cloudIn->points.size(), 0);
    int j =0;
    // pcl::PointCloud<pcl::PointXYZ>::iterator p;// pcl::PointCloud<pcl::PointXYZ>::iterator   //iterator不能用做Ptr
    //     for(p=cloudIn_org->points.begin();p!=cloudIn_org->points.end();++p){
    //       points.row(j)<<p->pointx,p->y,p->z;
    //     }
        for(auto p:cloudIn->points){
            points.row(j++)<<p.x,p.y,p.z;//(x,y,z)
        }
    //////////////////////////////////////a*x+b*y+c*z结果///////////////////////
     VectorXf result = points*normal;    
     for(int r=0;r<result.rows();++r){
            if(result[r]<dist){//a*x+b*y+c*z+d< the_dist, 点到面的距离a*x+b*y+c*z+d
                ground_cloud->points.push_back(cloudIn->points[r]);
                label->at(r) = 1;
            }else{
                label->at(r) = -1;
                obstacle_cloud->points.push_back(cloudIn->points[r]);
            }
        }

    // ROS_INFO("Wrong");
    if(ground_cloud->points.size()<=100){
        test_flag=1;
} 
    if(test_flag==1){
      ROS_INFO("Wrong");
    }
  }

 pcl::PointCloud<pcl::PointXYZ>::Ptr  groundplane_fit::test_api(){
    return cloudIn;
  }

////////////////////////////////scanID ground point//////////////////////////////
scan_ground::scan_ground(const GroundSegmentationParams& params, pcl::PointCloud<PointType>::Ptr laser_cloudIn, pcl::PointCloud<PointType>::Ptr laser_cloudIn_org, pcl::PointCloud<PointType>::Ptr laser_cloudIn_sorted, std::vector<int>* labels,std::vector<double>* range_s)
:params_(params),cloudIn(laser_cloudIn), cloudIn_org(laser_cloudIn_org), sorted_cloud(laser_cloudIn_sorted), label(labels), ranges(range_s)
{
  
}

void scan_ground::filtering(pcl::PointCloud<PointType>::Ptr obstacle_cloud){
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloudIn, *cloudIn, indices);
  sort(cloudIn->points.begin(),cloudIn->points.end(),point_type_z_cmp);//sort: 将laserCLoudIn中地址为begin到end的数据，按照function point_cmp排序
  pcl::PointCloud<PointType>::iterator it = cloudIn->points.begin();//iterator 是一种地址
  for(int i=0;i<cloudIn->points.size();i++){//去除反射点
        if(cloudIn->points[i].z < -1.5*params_.sensor_height){
            // obstacle_cloud->points.push_back(cloudIn->points[i]);
            it++;
        }else{
            break;
        }
    }
  ROS_INFO("size:%d",cloudIn->points.size());
  cloudIn->points.erase(cloudIn->points.begin(),it);//删除laserCloudIn中地址为begin()到it之前的点
  
}

void scan_ground::init_scanID(){
  ROS_INFO("size:%d",cloudIn->points.size());
  sorted_cloud.reset(new pcl::PointCloud<PointType>());
  sorted_cloud->points.resize(params_.N_scan*params_.horizon_scan);
  label->resize( params_.N_scan*params_.horizon_scan, 0);
  ranges->resize(params_.N_scan*params_.horizon_scan, -1);
  // *global_label=std::vector< std::vector<int> >(params_.N_scan,std::vector<int>(params_.horizon_scan,0));
  // *ground_label=std::vector< std::vector<int> >(params_.N_scan,std::vector<int>(params_.horizon_scan,0));
  int cloud_size= cloudIn->points.size();
  PointType pointi;
  double range=0;
  int row_Idn;
  int col_Idn;
  int index;
  double vec_angle;
  double hor_angle;
  for(int i=0;i<cloud_size;++i)
  {
    pointi.x=cloudIn->points[i].x;
    pointi.y=cloudIn->points[i].y;
    pointi.z=cloudIn->points[i].z;

    vec_angle=atan2(pointi.z,sqrt(pointi.x*pointi.x+pointi.y*pointi.y))*180/M_PI;
    row_Idn = (vec_angle + params_.ang_bottom) / params_.ang_res_y;//row_Idn [0,31]
    // ROS_INFO("row_Idn:%d",row_Idn);
    if(row_Idn<0 || row_Idn>=params_.N_scan) 
      {
        // obstacle_cloud->points.push_back(cloudIn->points[i]);
        continue;
      }
    hor_angle=atan2(pointi.x,pointi.y)* 180 / M_PI;//顺时针旋转 //double(360/params_.horizon_scan)
    col_Idn = round((hor_angle+180)/params_.ang_res_x);//hor_angle=[-180,180]-->[0,1799]
    // ROS_INFO("col_Idn:%d",col_Idn);
    if(col_Idn>=params_.horizon_scan)
    {
      col_Idn-=params_.horizon_scan;
    }
    index= row_Idn+params_.N_scan*col_Idn;//32*1800
    range = sqrt( pointi.x * pointi.x+ pointi.y * pointi.y+ pointi.z * pointi.z );
    if(range <params_.minrange) 
      {
        label->at(index)=-2;
        continue;
      }
    
    // ROS_INFO("index-th:%d",index);
    //test result: 还是存在repeat 情况
    // if(label->at(index)==1)
    // {
    //   // label->at(index)=2;
    //   ROS_INFO("repeat");
    // }
    label->at(index)=1;//label:1 存在有效点(可以是地面点，或者不是),label=-2 无效点, label=0(range=-1)没有点
    ranges->at(index)=range;
    sorted_cloud->points[index]=pointi;
    // sorted_cloud->points.push_back(pointi);
  }

}

void scan_ground::ground_point_detect(pcl::PointCloud<PointType>::Ptr ground_cloud,pcl::PointCloud<PointType>::Ptr obstacle_cloud){
  int lower_Idn=0;
  int upper_Idn=0;
  double diffX;
  double diffY;
  double diffZ;
  double angle;
  int  ground_point_found;
  for(int i=0;i<params_.horizon_scan;++i)
  {
    ground_point_found=0;
    for(int j=0;j<params_.ground_N;++j)
    {
      lower_Idn= j+i*params_.N_scan;
      upper_Idn= (j+1)+i*params_.N_scan;
      // ROS_INFO("%d",lower_Idn);
      // ROS_INFO("%d",upper_Idn);
      if(lower_Idn==(params_.horizon_scan*params_.ground_N-1))
      {
        break;
      }
      if(label->at(lower_Idn)==0 || label->at(upper_Idn)==0 || label->at(lower_Idn)==-2 || label->at(upper_Idn)==-2)
      {
        continue;
      }
      diffX = sorted_cloud->points[upper_Idn].x - sorted_cloud->points[lower_Idn].x;
      diffY = sorted_cloud->points[upper_Idn].y - sorted_cloud->points[lower_Idn].y;
      diffZ = sorted_cloud->points[upper_Idn].z - sorted_cloud->points[lower_Idn].z;
      angle = atan2(diffZ,sqrt(diffX*diffX+diffY*diffY)) *180/M_PI;
      if(abs(angle)<10 && abs(sorted_cloud->points[lower_Idn].z)>0.65*params_.sensor_height && abs(sorted_cloud->points[upper_Idn].z)>0.65*params_.sensor_height)//&& diffZ< 0.2
        {
          label->at(lower_Idn)=-1;
          label->at(upper_Idn)=-1;
          // sorted_cloud->points[lower_Idn].intensity=;
          ground_cloud->points.push_back(sorted_cloud->points[lower_Idn]);
          ground_cloud->points.push_back(sorted_cloud->points[upper_Idn]);
          ground_point_found+=2;
        }
      // else 
      // {
      //   obstacle_cloud->points.push_back(sorted_cloud->points[lower_Idn]);
      //   obstacle_cloud->points.push_back(sorted_cloud->points[upper_Idn]);
      // }
    }
    if(ground_point_found>=(params_.ground_N/3))
    {
      for(int k=0;k<params_.ground_N;++k)
      {
        if(label->at((k+i*params_.N_scan))!=-1)
        {
          label->at((k+i*params_.N_scan))=-1;
        }   
      }
    }
  }
  // ROS_INFO("size:%d",ground_cloud->points.size());
  int t_ind;
  for(int i=0;i<params_.horizon_scan;++i)//test label=1
  {
    for(int j=0;j<params_.N_scan;++j)
    {
      t_ind=j+i*params_.N_scan;
      if(label->at(t_ind)==1)
      {
        obstacle_cloud->points.push_back(sorted_cloud->points[t_ind]);
      }


    }
  }
}

// void scan_ground::point_segmentation(pcl::PointCloud<PointType>::Ptr segmented_cloud){//32线四邻域
//   int index;
//   int count=2;//   count=2开始作为语义分割的label
//   int q;    //   queue_size of the element with the same label, but not searched
//   int q_l;  //   queue_length if all the element with same label, but searched
//   double beta;
//   double alpha;
//   double d1;
//   double d2;
//   std::vector<int> index_neighbour(4,-1);
//   std::vector<int> index_vadality(4,1);
//   std::vector<int> same_label(params_.horizon_scan*params_.N_scan,-1);//same_label 一个队列，q_l是队列长度,q是队列当前检索的位置
//                                                                       //检测到新的neighbour加入same_label末端
  
//   /*
//   index_neighbour[0]=int index_nu;
//   index_neighbour[1]=int index_nd;
//   index_neighbour[2]=int index_nl;
//   index_neighbour[3]=int index_nr;

//   index_vadality[0]: the vadality of index_neighbour[0]
//   */
//   for(int i=0;i<params_.horizon_scan;++i)
//   {
//     for(int j=0;j<params_.N_scan;++j)
//     {
//       index=j+i*params_.N_scan;
//       if(label->at(index)==1)//label=1 说明非地面点的有效点
//       {
//         q=0;
//         q_l=1;
//         same_label.resize(params_.horizon_scan*params_.N_scan,-1);
//         same_label[0]=index;
//         label->at(index)=count;
//         while(q<=(q_l-1))
//         {
//           // label->at(index)=count;
//           index=same_label[q];
//           index_neighbour[0]= index-1;
//           index_neighbour[1]= index+1;
//           index_neighbour[2]= index-params_.N_scan;
//           index_neighbour[3]= index+params_.N_scan;
//           // reset vadality of the neighbours
//           index_vadality[0]=1;
//           index_vadality[1]=1;
//           index_vadality[2]=1;
//           index_vadality[3]=1;
//           // ROS_INFO("q  :%d",q);
//           // ROS_INFO("q_l:%d",q_l);
//           // ROS_INFO("index:%d",index);
//           /////check vadality of index/////
//           if((index%params_.N_scan)==0)                //最上排元素
//           {
//             index_vadality[0]=0;
//           }
//           if((index%params_.N_scan)==(params_.N_scan-1))//最下排元素
//           {
//             index_vadality[1]=0;
//           }
//           if(index<params_.N_scan)               //最左排元素
//           {
//             index_neighbour[2]=index+(params_.horizon_scan-1)*(params_.N_scan);
//           }
//           if(index>=( (params_.horizon_scan-1)*params_.N_scan) )//最右排元素
//           {
//             index_neighbour[3]=index-(params_.horizon_scan-1)*(params_.N_scan);
//           }
//           // ROS_INFO("0:%d",index_vadality[0]);
//           // ROS_INFO("1:%d",index_vadality[1]);
//           // ROS_INFO("2:%d",index_vadality[2]);
//           // ROS_INFO("3:%d",index_vadality[3]);
//           /////depth segmentation/////
//           for(int i=0;i<4;++i)
//           {
//             // ROS_INFO("0:%d",index_neighbour[i]);
//             // ROS_INFO("1:%d",index_vadality[i]);
//             if(index_vadality[i]!=0 && label->at(index_neighbour[i])==1)//邻近点有效且是有效点
//             {
//               if(i==0 || i==1)
//               {
//                 alpha=params_.ang_res_y*M_PI/180.0;
//               }
//               else
//               {
//                 alpha=params_.ang_res_x*M_PI/180.0;
//               }

//               // ROS_INFO("x:%f",params_.ang_res_x);
//               // ROS_INFO("y:%f",params_.ang_res_y);

//               // if(ranges->at(index_neighbour[i])==-1 )// || ranges->at(index)
//               // {
//               //   ROS_INFO("neigh range error");
//               //   ROS_INFO("index:%d",index_neighbour[i]);
//               // }

//               // if(ranges->at(index)==-1 )// || ranges->at(index)
//               // {
//               //   ROS_INFO("index range error");
//               //   ROS_INFO("index:%d",index);
//               // }
              
//               d1=std::max(ranges->at(index_neighbour[i]),ranges->at(index));
//               d2=std::min(ranges->at(index_neighbour[i]),ranges->at(index));
//               beta=atan2( d2*sin(alpha),(d1-d2*cos(alpha)) );
//               // ROS_INFO("bata:%f",beta);
//               if(beta>seg_angle)
//               {
//                 same_label[q_l]=index_neighbour[i];
//                 label->at(index_neighbour[i])=count;
//                 // ROS_INFO("index_n  :%d",index_neighbour[i]);
//                 // ROS_INFO("same_la  :%d",same_label[q+1]);
//                 q_l++;
//               }
//             }
//           }
//           q++;
//         }
//         // ROS_INFO("q_l:%d",q_l);
//         if(q_l>=30)
//         {
//           for(int i=0;i<q_l;++i)
//           { 
//             label->at(same_label[i])=count;
//             // ROS_INFO("count:%d",count);
//             // ROS_INFO("q_l:%d",q_l);
//           }
//           // ROS_INFO("count:%d",count);
//           count++;
//         }
//         else
//         {
//           for(int i=0;i<q_l;++i)
//           { 
//             label->at(same_label[i])=-2;//无效点
//           }
//         }
//       }
//       // else if(label->at(index)==0)
//       // {
//       //   ROS_INFO("test");
//       //   label->at(index)=-2;// 将label=0的index归类为无效点
//       // }
//     }
//   }  
//   for(int i=0;i<params_.horizon_scan;++i)
//   {
//     for(int j=0;j<params_.N_scan;++j)
//     {
//       if(label->at((j+i*params_.N_scan))>=2)//&& ranges->at((j+i*params_.N_scan))!=-1
//       {
//       segmented_cloud->points.push_back(sorted_cloud->points[j+i*params_.N_scan]);
//       segmented_cloud->points.back().intensity=label->at(j+i*params_.N_scan);
//       }
//       // segmented_cloud->points.push_back(sorted_cloud->points[j+i*params_.N_scan]);
//       // segmented_cloud->points.back().intensity=label->at(j+i*params_.N_scan);
//       // sorted_cloud->points[j+i*params_.N_scan].intensity=label->at(j+i*params_.N_scan);
//       // sorted_cloud->points[j+i*params_.N_scan].intensity=50;
//     }
//   }
//     // ROS_INFO("NO:%d",segmented_cloud->points.size());
// }

void scan_ground::point_segmentation(pcl::PointCloud<PointType>::Ptr segmented_cloud){//八邻域
  int index;
  int count=2;//   count=2开始作为语义分割的label
  int q;    //   queue_size of the element with the same label, but not searched
  int q_l;  //   queue_length if all the element with same label, but searched
  double beta;
  double alpha;
  double d1;
  double d2;
  std::vector<int> index_neighbour(8,-1);
  std::vector<int> index_vadality(8,1);
  std::vector<int> same_label(params_.horizon_scan*params_.N_scan,-1);//same_label 一个队列，q_l是队列长度,q是队列当前检索的位置
  int line_count_detector=0;//detect the searching time of (up /down);
                                                                      //检测到新的neighbour加入same_label末端
  
  /*
  index_neighbour[0]=int index_nu;
  index_neighbour[1]=int index_nd;
  index_neighbour[2]=int index_nl;
  index_neighbour[3]=int index_nr;

  index_vadality[0]: the vadality of index_neighbour[0]
  */
  for(int i=0;i<params_.horizon_scan;++i)
  {
    for(int j=0;j<params_.N_scan;++j)
    {
      index=j+i*params_.N_scan;
      if(label->at(index)==1)//label=1 说明非地面点的有效点
      {
        line_count_detector=0;
        q=0;
        q_l=1;
        same_label.resize(params_.horizon_scan*params_.N_scan,-1);
        same_label[0]=index;
        label->at(index)=count;
        while(q<=(q_l-1))
        {
          // label->at(index)=count;
          index=same_label[q];
          index_neighbour[0]= index-1;//up
          index_neighbour[1]= index+1;//down
          index_neighbour[2]= index-params_.N_scan;//left
          index_neighbour[3]= index+params_.N_scan;//right
          index_neighbour[4]= index-1-params_.N_scan;//up left
          index_neighbour[5]= index-1+params_.N_scan;//up right
          index_neighbour[6]= index+1-params_.N_scan;//down left
          index_neighbour[7]= index+1+params_.N_scan;//down right
          // reset vadality of the neighbours
          index_vadality[0]=1;
          index_vadality[1]=1;
          index_vadality[2]=1;
          index_vadality[3]=1;
          index_vadality[4]=1;
          index_vadality[5]=1;
          index_vadality[6]=1;
          index_vadality[7]=1;
          // ROS_INFO("q  :%d",q);
          // ROS_INFO("q_l:%d",q_l);
          // ROS_INFO("index:%d",index);
          /////check vadality of index/////
          if((index%params_.N_scan)==0)                //最上排元素
          {
            index_vadality[0]=0;
            index_vadality[4]=0;
            index_vadality[5]=0;
          }
          if((index%params_.N_scan)==(params_.N_scan-1))//最下排元素
          {
            index_vadality[1]=0;
            index_vadality[6]=0;
            index_vadality[7]=0;
          }
          if(index<params_.N_scan)               //最左排元素
          {
            index_neighbour[2]=index+(params_.horizon_scan-1)*(params_.N_scan);
            index_neighbour[4]=index+(params_.horizon_scan-1)*(params_.N_scan);
            index_neighbour[6]=index+(params_.horizon_scan-1)*(params_.N_scan);
          }
          if(index>=( (params_.horizon_scan-1)*params_.N_scan) )//最右排元素
          {
            index_neighbour[3]=index-(params_.horizon_scan-1)*(params_.N_scan);
            index_neighbour[5]=index-(params_.horizon_scan-1)*(params_.N_scan);
            index_neighbour[7]=index-(params_.horizon_scan-1)*(params_.N_scan);
          }
          // ROS_INFO("0:%d",index_vadality[0]);
          // ROS_INFO("1:%d",index_vadality[1]);
          // ROS_INFO("2:%d",index_vadality[2]);
          // ROS_INFO("3:%d",index_vadality[3]);
          /////depth segmentation/////
          for(int i=0;i<8;++i)
          {
            // ROS_INFO("0:%d",index_neighbour[i]);
            // ROS_INFO("1:%d",index_vadality[i]);
            if(index_vadality[i]!=0 && label->at(index_neighbour[i])==1)//邻近点有效且是有效点
            {
              if(i==0 || i==1)
              {
                alpha=params_.ang_res_y*M_PI/180.0;
              }
              else
              {
                alpha=params_.ang_res_x*M_PI/180.0;
              }

              // ROS_INFO("x:%f",params_.ang_res_x);
              // ROS_INFO("y:%f",params_.ang_res_y);

              // if(ranges->at(index_neighbour[i])==-1 )// || ranges->at(index)
              // {
              //   ROS_INFO("neigh range error");
              //   ROS_INFO("index:%d",index_neighbour[i]);
              // }

              // if(ranges->at(index)==-1 )// || ranges->at(index)
              // {
              //   ROS_INFO("index range error");
              //   ROS_INFO("index:%d",index);
              // }
              
              d1=std::max(ranges->at(index_neighbour[i]),ranges->at(index));
              d2=std::min(ranges->at(index_neighbour[i]),ranges->at(index));
              beta=atan2( d2*sin(alpha),(d1-d2*cos(alpha)) );
              // ROS_INFO("bata:%f",beta);
              if(beta>seg_angle)
              {
                same_label[q_l]=index_neighbour[i];
                label->at(index_neighbour[i])=count;
                if(i==0 ||i==4||i==5)
                {
                  line_count_detector++;
                }
                if(i==1 || i==6 || i==7)
                {
                  line_count_detector--;
                }
                // ROS_INFO("index_n  :%d",index_neighbour[i]);
                // ROS_INFO("same_la  :%d",same_label[q+1]);
                q_l++;
              }
            }
          }
          q++;
        }
        // ROS_INFO("q_l:%d",q_l);
        if(q_l>=30)
        {
          for(int i=0;i<q_l;++i)
          { 
            label->at(same_label[i])=count;
            // ROS_INFO("count:%d",count);
            // ROS_INFO("q_l:%d",q_l);
          }
          // ROS_INFO("count:%d",count);
          count++;
        }
        else if (q_l>=5)
        {
          if(line_count_detector>=3)
          {
            for(int i=0;i<q_l;++i)
          { 
            label->at(same_label[i])=count;
            // ROS_INFO("count:%d",count);
            // ROS_INFO("q_l:%d",q_l);
          }
          // ROS_INFO("count:%d",count);
          count++;
          }
        }
        else
        {
          for(int i=0;i<q_l;++i)
          { 
            label->at(same_label[i])=-2;//无效点
          }
        }
      }
      // else if(label->at(index)==0)
      // {
      //   ROS_INFO("test");
      //   label->at(index)=-2;// 将label=0的index归类为无效点
      // }
    }
  }  
  //// extract the segmented point cloud with specific intensity
  for(int i=0;i<params_.horizon_scan;++i)
  {
    for(int j=0;j<params_.N_scan;++j)
    {
      if(label->at((j+i*params_.N_scan))>=2)//&& ranges->at((j+i*params_.N_scan))!=-1
      {
      segmented_cloud->points.push_back(sorted_cloud->points[j+i*params_.N_scan]);
      segmented_cloud->points.back().intensity=label->at(j+i*params_.N_scan);
      }
      // segmented_cloud->points.push_back(sorted_cloud->points[j+i*params_.N_scan]);
      // segmented_cloud->points.back().intensity=label->at(j+i*params_.N_scan);
      // sorted_cloud->points[j+i*params_.N_scan].intensity=label->at(j+i*params_.N_scan);
      // sorted_cloud->points[j+i*params_.N_scan].intensity=50;
    }
  }
    // ROS_INFO("NO:%d",segmented_cloud->points.size());
}