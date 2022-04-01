#include "ground_seg/segment.h"
#include <ros/ros.h>
Segment::Segment(const unsigned int& n_bins,///Segment的构造函数
                 const double& max_slope,
                 const double& max_error,
                 const double& long_threshold,
                 const double& max_long_height,
                 const double& max_start_height,
                 const double& sensor_height) :
                 bins_(n_bins),
                 max_slope_(max_slope),
                 max_error_(max_error),
                 long_threshold_(long_threshold),
                 max_long_height_(max_long_height),
                 max_start_height_(max_start_height),
                 sensor_height_(sensor_height){}

/*分割线拟合*/
void Segment::fitSegmentLines() {
  // Find first point.
  auto line_start = bins_.begin();//std::vector<Bin> bins_, 长度是n_bins
  /*在整个的bins中找第一个点*/
  while (!line_start->hasPoint()) {//bins_里面的每个元素存的都是每个bin中最小的元素，如果没有最小的元素，则has point=false
    ++line_start;
    // ROS_INFO("test");
    // Stop if we reached last point.
    if (line_start == bins_.end()) {return;}
  }
  ROS_INFO("line_start---> d:%f, z:%f ", line_start->getMinZPoint().d, line_start->getMinZPoint().z);
  // Fill lines.
  bool is_long_line = false;
  double cur_ground_height = -sensor_height_;

  // bool max_break = false;
  /*将线的第一个点的信息传递到*/
  std::list<Bin::MinZPoint> current_line_points(1, line_start->getMinZPoint());


  // if (std::fabs(current_line_points.back().z - cur_ground_height) > 0.1){
  // current_line_points.pop_back();
  // }


  LocalLine cur_line = std::make_pair(0,0);// curline= (k,b) means y= kx+b
  /*从第一个点开始对于bins上的每一个点都进行遍历操作*/
  for (auto line_iter = line_start+1; line_iter != bins_.end(); ++line_iter) {
    /*如果我们设定的线上有点，则进行后面的操作*/
    if (line_iter->hasPoint()) {
      Bin::MinZPoint cur_point = line_iter->getMinZPoint();
      /*如果两个的线的长度大于我们设定的阈值，则我们认为这两个点之间是长线*/
      if (cur_point.d - current_line_points.back().d > long_threshold_) is_long_line = true;
      /*针对于后面几次遍历而言的，至少有三个点的情况下*/
      if (current_line_points.size() >= 2) {

        // Get expected z value to possibly reject far away points.
        /*获取远离点的z值*/
        double expected_z = std::numeric_limits<double>::max();
        /*如果是长线段且当前线段的点数大于2，则我们获取到期待的z，这个在第一次迭代的时候不会被执行*/
        if (is_long_line && current_line_points.size() > 2) {
          expected_z = cur_line.first * cur_point.d + cur_line.second;
        }
        /*将当前点插入到current_line之中*/
        // if(std::fabs( cur_point.z - current_line_points.back().z) <0.01)
        current_line_points.push_back(cur_point);
        
        /*对于当前线点传入到本地线拟合中，得到最后的结果*/
        cur_line = fitLocalLine(current_line_points);
        /*将我们经过本地线拟合之后的*/
        const double error = getMaxError(current_line_points, cur_line);//返回最大拟合误差
        // Check if not a good line.
        /*将算出来的误差和最大误差进行比较，判断是否是一个合格的线*/
        if (error > max_error_ ||
            std::fabs(cur_line.first) > max_slope_ ||
            is_long_line && std::fabs(expected_z - cur_point.z) > max_long_height_) {
              //
          // Add line until previous point as ground.
          /*添加线直到浅一点是地面点*/
          current_line_points.pop_back();//拟合不好则把当前点去掉
          // Don't let lines with 2 base points through.
          /*不要让有两个基点的线穿过*/
          // if (std::fabs(cur_line.first) > max_slope_)
          // {
          //   max_break = true;
          //   ROS_INFO("out_slope break");
          // }

          if (current_line_points.size() >= 3) {//去掉当前点，如果起码还有三个点
            /*对于当前线点进行本地拟合，得到一条新的线*/
            const LocalLine new_line = fitLocalLine(current_line_points);//可以记录前一个计算值，不用再计算一次
            /*将进行处理后的点放入到线中*/
            lines_.push_back(localLineToLine(new_line, current_line_points));
            /*计算出当前地面点的高度*/
            cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;
          }
          // Start new line.
          is_long_line = false;
          /*erase在删除的过程中还是减少vector的size*///拟合效果不好，则把当前点之前的点都删除，留下当前点，作为拟合直线的最新的第一个点
          current_line_points.erase(current_line_points.begin(), --current_line_points.end());//解决问题：如果存在坡度的话，就可以把前面平面部分的点删掉
          --line_iter;
          
          break;
            //  ROS_INFO("out_slope not break");//  break test
        
          
        }
        // Good line, continue.
        else { }
      }
    /*在有1到2个点的情况下的处理*/
      else {
        // Not enough points.
        /*判断是否满足添加条件，添加这些点*///判断两点是否在同一平面中
        if (//std::fabs(cur_point.d - current_line_points.back().d) < long_threshold_ &&
            std::fabs(cur_point.d)                                < 10 &&
            std::fabs(cur_point.z - current_line_points.back().z) < +max_start_height_ &&
            std::fabs(current_line_points.back().z - cur_ground_height) < +max_start_height_) {//起始点的问题，如果第一步起不来，后面肯定没有点了
          // Add point if valid. 
            // if(current_line_points.size()==1 && current_line_points.back() )                                                  //max_start_height 不能太大，不然路基的上沿会被当成地面点
            current_line_points.push_back(cur_point);
          
          // else{
          //   current_line_points.push_back(cur_point);  
          // }
          // if (max_break)
          // {
          //   ROS_INFO("out_slope not break");//  break test
          // }

        }
        /*开始一条新的线*/
        // else {
        //   // Start new line.
        //   current_line_points.clear();
  
        //   current_line_points.push_back(cur_point);
          
        //   // if (max_break)
        //   // {
        //   //   ROS_INFO("out_slope not break");//  break test
        //   // }

        // }
      }
    }
  }
  // Add last line.
  /*添加最后一条线*/
  // ROS_INFO("size:%d",current_line_points.size());
  if (current_line_points.size() > 2) {
    const LocalLine new_line = fitLocalLine(current_line_points);
    lines_.push_back(localLineToLine(new_line, current_line_points));
  }
}

/*本地线到线，得到两个点，构建出一条直线*/
Segment::Line Segment::localLineToLine(const LocalLine& local_line,
                                       const std::list<Bin::MinZPoint>& line_points) {
  Line line;
  const double first_d = line_points.front().d;//line_points 第一个点和最后一个点在拟合直线中的值(d,z) ,z 用d算
  const double second_d = line_points.back().d;
  /*跟去前面的值来进行locl_line的处理*/
  const double first_z = local_line.first * first_d + local_line.second;
  const double second_z = local_line.first * second_d + local_line.second;
  line.first.z = first_z;
  line.first.d = first_d;
  line.second.z = second_z;
  line.second.d = second_d;
  return line;
}

/*到线的垂直距离*/ // origin
// double Segment::verticalDistanceToLine(const double &d, const double &z) {
//   static const double kMargin = 0.1;
//   double distance = -1;
//   for (auto it = lines_.begin(); it != lines_.end(); ++it) {
//     /*这里设定了论文中要求的距离*/
//     /*针对于d点，按照设定的余量在前后范围内找到两个点*/ //保证了点在拟合直线的范围之内
//     if (it->first.d - kMargin < d && it->second.d + kMargin > d) {
//       /*这里是先算出斜率，将传入的两个点传入到直线中，算出一个最近的额z值差，也就是垂直的距离*/
//       /*算出找到的两个点之间的斜率*/
//       const double delta_z = it->second.z - it->first.z;
//       const double delta_d = it->second.d - it->first.d;
//       const double expected_z = (d - it->first.d)/delta_d *delta_z + it->first.z;//(delta_z/delta_d)是一个斜率
//       //算出最终的距离
//       distance = std::fabs(z - expected_z);
//     }
//   }
//   return distance;
// }
/////////////////////////////////////////
/*到线的垂直距离*/
std::pair<double,bool> Segment::verticalDistanceToLine(const double &d, const double &z) {
  static const double kMargin = 0.1; //0.1 default
  double distance = -1;
  double min_d = DBL_MAX;
  double record_flag =0;
  bool   border_flag =false;
  double d_border= lines_.back().second.d;
  std::pair<double,bool> k;
  for (auto it = lines_.begin(); it != lines_.end(); ++it) {
    /*这里设定了论文中要求的距离*/
    /*针对于d点，按照设定的余量在前后范围内找到两个点*/ //保证了点在拟合直线的范围之内
    if(record_flag ==1)//2
    {
      k.first = min_d;
      if(std::fabs(d-d_border)<0.05){
      border_flag = true;
      }
      k.second = border_flag;
      return k;
    }
    
    if (it->first.d - kMargin < d && it->second.d + kMargin > d) {
      /*这里是先算出斜率，将传入的两个点传入到直线中，算出一个最近的额z值差，也就是垂直的距离*/
      /*算出找到的两个点之间的斜率*/
      const double delta_z = it->second.z - it->first.z;
      const double delta_d = it->second.d - it->first.d;
      const double expected_z = (d - it->first.d)/delta_d *delta_z + it->first.z;//(delta_z/delta_d)是一个斜率
      //算出最终的距离
      record_flag++;
      distance = std::fabs(z - expected_z);
      if (distance <std::fabs(min_d))
      {
        min_d =distance;
      }
    }

  }

  // double d_border=(lines_end()-1)->second.d;
  if(d>d_border && d<d_border+0.01){
    border_flag = true;
  }
  if (distance == -1)
  {
    min_d=-1;
  }
  k.first  = min_d;
  k.second = border_flag;
  return k;
}

double Segment::getMeanError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
  double error_sum = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;
    error_sum += residual * residual;
  }
  return error_sum/points.size();
}

double Segment::getMaxError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
  double max_error = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;
    const double error = residual * residual;
    if (error > max_error) max_error = error;
  }
  return max_error;
}

/*本地线拟合*/
/*
z=kd+b

[d1,1;d2,1] * [k,b]^T =[z1,z2]^T

A*x=b 线性最小二乘法

*/
Segment::LocalLine Segment::fitLocalLine(const std::list<Bin::MinZPoint> &points) {
  const unsigned int n_points = points.size();
  Eigen::MatrixXd X(n_points, 2);//n*2 矩阵
  Eigen::VectorXd Y(n_points);//n*1 矩阵
  unsigned int counter = 0;
  for (auto iter = points.begin(); iter != points.end(); ++iter) {
    X(counter, 0) = iter->d;
    X(counter, 1) = 1;
    Y(counter) = iter->z;
    ++counter;
  }
  Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);//Ax=b 求伪逆(QR分解)
  LocalLine line_result;
  line_result.first = result(0);
  line_result.second = result(1);
  return line_result;
}

bool Segment::getLines(std::list<Line> *lines) {
  if (lines_.empty()) {
    return false;
  }
  else {
    *lines = lines_;
    return true;
  }
}
