// #ifndef _INCLUDE_MIN_BOX_H_
// #define _INCLUDE_MIN_BOX_H_

// #include <algorithm>
// #include <vector>
// #include <ctime>
// #include "opencv2/core/core.hpp"
// #include "opencv2/opencv.hpp"
// #include <opencv2/highgui.hpp>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/common.h>
// #include <pcl/common/centroid.h>
// #include <pcl/point_cloud.h>

// #include <visualization_msgs/MarkerArray.h>

// #include "tracking_msgs/DetectedObject.h"
// #include "tracking_msgs/DetectedObjectArray.h"

// #define PRISMBOX 0
// #define CUBOIDBOX 1

// using namespace std;


// struct Vertex{
//   float x;
//   float y;
//   Vertex(){}
//   Vertex(float a, float b)
//   {
//     x = a;
//     y = b;
//   }
//   bool operator <(const Vertex &p) const {
//     return x < p.x || (x == p.x && y < p.y);
//   }

//   Vertex operator +(const Vertex &p) const{
//     Vertex vertex;
//     vertex.x = x + p.x;
//     vertex.y = y + p.y;
//     return vertex;
//   }

//   Vertex operator -(const Vertex &p) const{
//     Vertex vertex;
//     vertex.x = x - p.x;
//     vertex.y = y - p.y;
//     return vertex;
//   }
  
//   friend Vertex operator *(const float& k, const Vertex &p){
//     Vertex vertex;
//     vertex.x = k * p.x;
//     vertex.y = k * p.y;
//     return vertex;
//   }
// };

// class MyPrism{
// public:
//   std::vector<Vertex> vertices;
//   float top;
//   float bottom;
//   float center_z;
//   float height;
// };


// class Prism{
// public:
//   std::vector<MyPrism> prisms_;
//   ros::Time time;
//   visualization_msgs::MarkerArray toRviz();
// };

// // class BoundingBox{
// // public:
// //   Prism prisms_;
// //   Prism cuboids_;
// //   BoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZI> > clusters);
// // };
// class BoundingBox{
// public:
//   Prism prisms_;
//   Prism cuboids_;
//   BoundingBox(tracking_msgs::DetectedObjectArray& input);
// };


// class ConvexHull{
// public:
//   std::vector<Vertex> vertices_;
//   ConvexHull(vector<Vertex> P);
//   vector<Vertex> toRec();
//   vector<Vertex> toRec1();
//   vector<Vertex> toRec2();
//   vector<Vertex> Caliper();
//   // Vertex getMedianPoint();

// private:
//   vector<Vertex> origin_points_;
//   float cross(const Vertex &O, const Vertex &A, const Vertex &B);
//   float getDistance(const Vertex &A, const Vertex &B);
//   Vertex foot(const Vertex &A, const Vertex &B, const Vertex &O);
//   float minDisToRec(std::vector<Vertex> rect, Vertex point);
//   vector<Vertex> fitRec(float k);
//   float getArea(std::vector<Vertex> rect);
// public:
//   cv::Mat direction(vector<Vertex> points_input);
// };



// #endif
#ifndef _INCLUDE_MIN_BOX_H_
#define _INCLUDE_MIN_BOX_H_

#include <algorithm>
#include <vector>
#include <ctime>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>

#include <visualization_msgs/MarkerArray.h>

#include "tracking_msgs/DetectedObject.h"
#include "tracking_msgs/DetectedObjectArray.h"

#define PRISMBOX 0
#define CUBOIDBOX 1

using namespace std;

//namespace myshape {

struct Vertex{
  double x;
  double y;
  Vertex(){}
  Vertex(double a, double b)
  {
    x = a;
    y = b;
  }
  bool operator <(const Vertex &p) const {
    return x < p.x || (x == p.x && y < p.y);
  }

  Vertex operator +(const Vertex &p) const{
    Vertex vertex;
    vertex.x = x + p.x;
    vertex.y = y + p.y;
    return vertex;
  }

  Vertex operator -(const Vertex &p) const{
    Vertex vertex;
    vertex.x = x - p.x;
    vertex.y = y - p.y;
    return vertex;
  }
  
  friend Vertex operator *(const double& k, const Vertex &p){
    Vertex vertex;
    vertex.x = k * p.x;
    vertex.y = k * p.y;
    return vertex;
  }
};

class MyPrism{
public:
  std::vector<Vertex> vertices;
  double top;
  double bottom;
};



class Prism{
public:
  std::vector<MyPrism> prisms_;
  ros::Time time;
  visualization_msgs::MarkerArray toRviz();
};

class BoundingBox{
public:
  Prism prisms_;
  Prism cuboids_;
  Prism oldcuboids_;
  BoundingBox(tracking_msgs::DetectedObjectArray& input, ros::Time time);
  // BoundingBox(pcl::PointCloud<pcl::PointXYZI> cluster);

  BoundingBox(){}
      ~BoundingBox(){}
};


class ConvexHull{
public:
  std::vector<Vertex> origin_points_;
  std::vector<Vertex> vertices_;
  ConvexHull(vector<Vertex> P);
  vector<Vertex> toRec();
  vector<Vertex> toRec1();
  vector<Vertex> toRec2();
  vector<Vertex> Caliper();
  // Vertex getMedianPoint();

private:
  //vector<Vertex> origin_points_;
  double cross(const Vertex &O, const Vertex &A, const Vertex &B);
  double getDistance(const Vertex &A, const Vertex &B);
  Vertex foot(const Vertex &A, const Vertex &B, const Vertex &O);
  double minDisToRec(std::vector<Vertex> rect, Vertex point);
  vector<Vertex> fitRec(double k);
  double getArea(std::vector<Vertex> rect);
public:
  cv::Mat direction(vector<Vertex> points_input);
};

//}



#endif
