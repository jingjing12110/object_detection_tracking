// #include "box_type.h"
// #include "min_box.h"
// using namespace std;


// // Returns a list of points on the convex hull in counter-clockwise order.
// // Note: the last point in the returned list is the same as the first one.
// ConvexHull::ConvexHull(vector<Vertex> P)
// {
//   origin_points_ = P;
//   size_t n = P.size(), k = 0;
//   if (n <= 3) 
//   {
//     vertices_.assign(P.begin(), P.end());
//     return;
//   }

//   vertices_.resize(2*n);
//   // vector<Vertex> H(2*n);

//   // Sort points lexicographically
//   sort(P.begin(), P.end());

//   // Build lower hull
//   for (size_t i = 0; i < n; ++i) 
//   {
//     while (k >= 2 && cross(vertices_[k-2], vertices_[k-1], P[i]) <= 0) k--;
//     vertices_[k++] = P[i];
//   }

//   // Build upper hull
//   for (size_t i = n-1, t = k+1; i > 0; --i) {
//     while (k >= t && cross(vertices_[k-2], vertices_[k-1], P[i-1]) <= 0) k--;
//     vertices_[k++] = P[i-1];
//   }

//   vertices_.resize(k-1);
// }

// // must be big enough to hold 2*max(|coordinate|)^2

// // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// // Returns a positive value, if OAB makes a counter-clockwise turn,
// // negative for clockwise turn, and zero if the points are collinear.
// float ConvexHull::cross(const Vertex &O, const Vertex &A, const Vertex &B)
// {
//   return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
// }

// float ConvexHull::getDistance(const Vertex &A, const Vertex &B)
// {
//   return sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y));
// }

// // O在AB上的垂足
// Vertex ConvexHull::foot(const Vertex &A, const Vertex &B, const Vertex &O)
// {
//   float k = (O.x * (B.x - A.x) + O.y * (B.y - A.y))/
//             (pow((B.x - A.x), 2) + pow((B.y - A.y), 2));
//   return k * (B - A);
// }

// //用PCA计算主方向，即朝向
// cv::Mat ConvexHull::direction(vector<Vertex> points_input)
// {
//   cv::Mat pcaSet(points_input.size(), 2, CV_32FC1, cv::Scalar::all(0));

//   float* data = nullptr;
//   for(size_t i = 0; i < points_input.size(); i++)
//   {
//     data = pcaSet.ptr<float>(i);
//     *data++ = points_input[i].x;
//     *data++ = points_input[i].y;
//   }
//   cv::PCA pca(pcaSet, cv::Mat(), CV_PCA_DATA_AS_ROW, 2);
//   //参数依次为：原始数据；原始数据均值，输入空会自己计算；每行/列代表一个样本；保留多少特征值，默认全保留

//   cv::Mat direction_vector = pca.eigenvectors;
//   return direction_vector;
// }
// // 给定朝向，拟合矩形
// vector<Vertex> ConvexHull::fitRec(float k)
// {
//   std::vector<Vertex> rect;
//   float k_1 = -1 / k;

//   float b_1, b_2, b_3, b_4;
//   b_1 = vertices_[0].y - k * vertices_[0].x;
//   b_2 = vertices_[0].y - k * vertices_[0].x;
//   b_3 = vertices_[0].y - k_1 * vertices_[0].x;
//   b_4 = vertices_[0].y - k_1 * vertices_[0].x;

//   for(size_t i = 0; i < vertices_.size(); i++)
//   {
//     //点在直线的左边
//     if(k * vertices_[i].x + b_1 <= vertices_[i].y)
//     {
//       b_1 = vertices_[i].y - k * vertices_[i].x;
//     }

//     if(k * vertices_[i].x + b_2 >= vertices_[i].y)
//     {
//       b_2 = vertices_[i].y - k * vertices_[i].x;
//     }

//     if(k_1 * vertices_[i].x + b_3 <= vertices_[i].y)
//     {
//       b_3 = vertices_[i].y - k_1 * vertices_[i].x;
//     }

//     if(k_1 * vertices_[i].x + b_4 >= vertices_[i].y)
//     {
//       b_4 = vertices_[i].y - k_1 * vertices_[i].x;
//     }
//   }
  
//   Vertex vertex;
//   vertex.x = (b_3 - b_1) / (k - k_1);
//   vertex.y = vertex.x * k + b_1;
//   rect.push_back(vertex);

//   vertex.x = (b_2 - b_3) / (k_1 - k);
//   vertex.y = vertex.x * k + b_2;
//   rect.push_back(vertex);

//   vertex.x = (b_4 - b_2) / (k - k_1);
//   vertex.y = vertex.x * k + b_2;
//   rect.push_back(vertex);

//   vertex.x = (b_1 - b_4) / (k_1 - k);
//   vertex.y = vertex.x * k + b_1;
//   rect.push_back(vertex);

//   return rect;
// }

// // 根据PCA得到的方向绘制矩形
// vector<Vertex> ConvexHull::toRec()
// {
//   std::vector<Vertex> rect;
//   cv::Mat direct= direction(vertices_);
//   float k = direct.at<float>(0,1)/direct.at<float>(0,0);
//   rect = fitRec(k);
//   return rect;
// }

// float ConvexHull::getArea(std::vector<Vertex> rect)
// {
//   return abs(cross(rect[1], rect[0], rect[2]));
// }

// // 得到面积最小fitting box
// vector<Vertex> ConvexHull::Caliper()
// {
//   std::vector<Vertex> rect;
//   float area = std::numeric_limits<float>::infinity();
//   int len = vertices_.size();
//   for(int i = 0; i < len; i++)
//   {
//     //计算斜率
//     float k = (vertices_[(i+1) % len].y - vertices_[(i) % len].y)/
//               (vertices_[(i+1) % len].x - vertices_[(i) % len].x);

//     std::vector<Vertex> rect_temp = fitRec(k);
//     float area_temp = getArea(rect_temp);
//     if(area > area_temp)
//     {
//       area = area_temp;
//       rect = rect_temp;
//     }
//   }
//   return rect;
// }


// // Vertex ConvexHull::getMedianPoint()
// // {
// //   int n = origin_points_.size();
// //   float w[n];
// //   Vertex median;

// //   for(int i = 0; i < 2; i++)
// //   {
// //     if(i == 0)
// //     {
// //       for(int j = 0; j < n; j++)
// //       {
// //         w[j] = 1;
// //       }
// //     }
// //     else
// //     {
// //       for(int j = 0; j < n; j++)
// //       {
// //         w[j] =abs(median.x - origin_points_[j].x) + abs(median.y - origin_points_[j].y);
// //       }
// //     }

// //     std::vector<float> pts_x;
// //     std::vector<float> pts_y;
// //     for(int j = 0; j < n; j++)
// //     {
// //       float pt_x_new = w[j] * origin_points_[j].x;
// //       float pt_y_new = w[j] * origin_points_[j].y;
// //       pts_x.push_back(pt_x_new);
// //       pts_y.push_back(pt_y_new);
// //     }
// //     std::vector<float> pts_x_copy = pts_x;
// //     std::vector<float> pts_y_copy = pts_y;
// //     sort(pts_x.begin(), pts_x.end());
// //     sort(pts_y.begin(), pts_y.end());

// //     std::vector<float>::iterator it_x = find(pts_x_copy.begin(), pts_x_copy.end(), pts_x[n/2]);
// //     std::vector<float>::iterator it_y = find(pts_y_copy.begin(), pts_y_copy.end(), pts_y[n/2]);

// //     // median.x = pts_x[n/2] / w[it_x - pts_x_copy.begin()];
// //     // median.y = pts_y[n/2] / w[it_y - pts_y_copy.begin()];
// //     median.x = pts_x[n/2] / w[it_x - pts_x_copy.begin()];
// //     median.y = pts_y[n/2] / w[it_y - pts_y_copy.begin()];
// //   }
// //   return median;
// // }

// float ConvexHull::minDisToRec(std::vector<Vertex> rect, Vertex point)
// {
//   float min_distance = std::numeric_limits<float>::infinity();
//   for(int i = 0; i < 4; i++)
//   {
//     float dis1 = abs(cross(rect[i%4], rect[(i+1)%4], point)) / getDistance(rect[i%4], rect[(i+1)%4]);
//     min_distance = (min_distance>dis1) ? dis1:min_distance;
//   }
//   return min_distance;
// }
// // 公式推导详见
// // An Orientation Corrected Bounding Box Fit Based on the Convex Hull under Real Time Constraints
// // Auther: Benjamin Naujoks and Hans-Joachim Wuensche
// vector<Vertex> ConvexHull::toRec1()
// {
//   static float lambda = 0.01;
//   std::vector<Vertex> rect = Caliper();
//   Vertex higher_point, lower_point, third_point, median_point;
  
//   //计算最高点和最低点
//   float upper_min = std::numeric_limits<float>::infinity();
//   float lower_min = std::numeric_limits<float>::infinity();;

//   for(size_t i = 0; i < vertices_.size(); i++)
//   {
//     float upper_dis = abs(cross(vertices_[i], rect[0], rect[1]))/getDistance(rect[0], rect[1]);
//     float lower_dis = abs(cross(vertices_[i], rect[2], rect[3]))/getDistance(rect[2], rect[3]);

//     if(upper_dis < upper_min)
//     {
//       upper_min = upper_dis;
//       higher_point = vertices_[i];
//     }

//     if(lower_dis < lower_min)
//     {
//       lower_min = lower_dis;
//       lower_point = vertices_[i];
//     }
//   }

//   //点到直线的距离减去垂足到两端点的最短距离和缩放系数的乘积
//   float distance = 0;
//   for(size_t i = 0; i < vertices_.size(); i++)
//   {
//     float min_cut = min(getDistance(foot(lower_point, higher_point, vertices_[i]) , lower_point),
//                         getDistance(foot(lower_point, higher_point, vertices_[i]) , higher_point));

//     float foot_distance = abs(cross(lower_point, higher_point, vertices_[i]))/
//                       getDistance(lower_point, higher_point);
                      
//     float dis_temp = foot_distance -  lambda * min_cut;

//     if(dis_temp > distance)
//     {
//       third_point = vertices_[i];
//       distance = dis_temp;
//     }
//   }

//   std::vector<Vertex> rect1 = fitRec((higher_point.y-lower_point.y)/(higher_point.x-lower_point.x));
//   std::vector<Vertex> rect2 = fitRec((higher_point.y-third_point.y)/(higher_point.x-third_point.x));
//   std::vector<Vertex> rect3 = fitRec((lower_point.y-third_point.y)/(lower_point.x-third_point.x));
//   float dis_sum1 = 0;
//   float dis_sum2 = 0;
//   float dis_sum3 = 0;
//   for(size_t i = 0; i < origin_points_.size(); i++)
//   {
//     dis_sum1 += minDisToRec(rect1, origin_points_[i]);
//     dis_sum2 += minDisToRec(rect2, origin_points_[i]);
//     dis_sum3 += minDisToRec(rect3, origin_points_[i]);
//   }

//   if(dis_sum1 < dis_sum2 && dis_sum1 < dis_sum3)
//   {
//     return rect1;
//   }
//   else if(dis_sum2 < dis_sum1 && dis_sum2 < dis_sum3)
//   {
//     return rect2;
//   }
//   else
//   {
//     return rect3;
//   }
// }

// vector<Vertex> ConvexHull::toRec2()
// {
//   std::vector<double> rectangles_weight;
//   std::vector<std::vector<Vertex>> rectangles_points;
//   //rectangles.resize(vertices_.size());
//   //rectangles_points.resize(vertices_.size());
//   for(size_t i = 0; i < vertices_.size(); i++)
//   {
//     if(i != vertices_.size()-1)
//     {
//       double k1 = (vertices_[i+1].y - vertices_[i].y)/(vertices_[i+1].x - vertices_[i].x);
//       double b11 = vertices_[i].y - k1 * vertices_[i].x;
//       double maxdis1 = 0;
//       int dispoint = i;
//       for(size_t j = 0; j < vertices_.size(); j++)
//       {
//         double dis = abs((k1 * vertices_[j].x - vertices_[j].y + b11)/sqrt(k1 * k1 + 1));
//         if(dis>maxdis1)
//         {
//           maxdis1 = dis;
//           dispoint = j;
//         }
//       }
//       double b12 = vertices_[dispoint].y - k1 * vertices_[dispoint].x;
//       double k2 = -1/k1;
//       double maxdis2 = 0;
//       double mindis2 = 0;
//       int maxdispoint = i;
//       int mindispoint = i;
//       double b1_ = vertices_[i].y - k2 * vertices_[i].x;
//       for(size_t j = 0; j < vertices_.size(); j++)
//       {
//         double dis = (k2 * vertices_[j].x - vertices_[j].y + b1_)/sqrt(k2 * k2 + 1);
//         if(dis>maxdis2)
//         {
//           maxdis2 = dis;
//           maxdispoint = j;
//         }else if(dis<mindis2)
//         {
//           mindis2 = dis;
//           mindispoint = j;
//         }
//       }
//       double b21 = vertices_[maxdispoint].y - k2 * vertices_[maxdispoint].x;
//       double b22 = vertices_[mindispoint].y - k2 * vertices_[mindispoint].x;

//       //栅格化
      
//       double point_weight = 0;
//       for(size_t j = 0; j < origin_points_.size(); j++)
//       {
//         double minweight1 = min(abs((k1 * origin_points_[j].x - origin_points_[j].y + b11)/sqrt(k1 * k1 + 1)),abs((k1 * origin_points_[j].x - origin_points_[j].y + b12)/sqrt(k1 * k1 + 1)));
//         double minweight2 = min(abs((k2 * origin_points_[j].x - origin_points_[j].y + b21)/sqrt(k2 * k2 + 1)),abs((k2 * origin_points_[j].x - origin_points_[j].y + b22)/sqrt(k2 * k2 + 1)));
//         point_weight = point_weight + min(minweight1,minweight2);
//       }
//       point_weight = point_weight/origin_points_.size();
//       //rectangles_weight.push_back(abs(((b11 - b12)/sqrt(k1 * k1 + 1)) * ((b21 - b22)/sqrt(k2 * k2 + 1))));
//       rectangles_weight.push_back(point_weight);

//       std::vector<Vertex> vertexs;
//       Vertex point1;
//       point1.x = (b21-b11)/(k1-k2);
//       point1.y = (k1*b21-k2*b11)/(k1-k2);
//       vertexs.push_back(point1);
//       Vertex point2;
//       point2.x = (b21-b12)/(k1-k2);
//       point2.y = (k1*b21-k2*b12)/(k1-k2);
//       vertexs.push_back(point2);
//       Vertex point3;
//       point3.x = (b22-b12)/(k1-k2);
//       point3.y = (k1*b22-k2*b12)/(k1-k2);
//       vertexs.push_back(point3);
//       Vertex point4;
//       point4.x = (b22-b11)/(k1-k2);
//       point4.y = (k1*b22-k2*b11)/(k1-k2);
//       vertexs.push_back(point4);

//       rectangles_points.push_back(vertexs);

//     }else if (i == vertices_.size()-1)
//     {
//       double k1 = (vertices_[0].y - vertices_[i].y)/(vertices_[0].x - vertices_[i].x);
//       double b11 = vertices_[i].y - k1 * vertices_[i].x;
//       double maxdis1 = 0;
//       int dispoint = i;
//       for(size_t j = 0; j < vertices_.size(); j++)
//       {
//         double dis = abs((k1 * vertices_[j].x - vertices_[j].y + b11)/sqrt(k1 * k1 + 1));
//         if(dis>maxdis1)
//         {
//           maxdis1 = dis;
//           dispoint = j;
//         }
//       }
//       double b12 = vertices_[dispoint].y - k1 * vertices_[dispoint].x;
//       double k2 = -1/k1;
//       double maxdis2 = 0;
//       double mindis2 = 0;
//       int maxdispoint = i;
//       int mindispoint = i;
//       double b1_ = vertices_[i].y - k2 * vertices_[i].x;
//       for(size_t j = 0; j < vertices_.size(); j++)
//       {
//         double dis = (k2 * vertices_[j].x - vertices_[j].y + b1_)/sqrt(k2 * k2 + 1);
//         if(dis>maxdis2)
//         {
//           maxdis2 = dis;
//           maxdispoint = j;
//         }else if(dis<mindis2)
//         {
//           mindis2 = dis;
//           mindispoint = j;
//         }
//       }
//       double b21 = vertices_[maxdispoint].y - k2 * vertices_[maxdispoint].x;
//       double b22 = vertices_[mindispoint].y - k2 * vertices_[mindispoint].x;

//       double point_weight = 0;
//       for(size_t j = 0; j < origin_points_.size(); j++)
//       {
//         double minweight1 = min(abs((k1 * origin_points_[j].x - origin_points_[j].y + b11)/sqrt(k1 * k1 + 1)),abs((k1 * origin_points_[j].x - origin_points_[j].y + b12)/sqrt(k1 * k1 + 1)));
//         double minweight2 = min(abs((k2 * origin_points_[j].x - origin_points_[j].y + b21)/sqrt(k2 * k2 + 1)),abs((k2 * origin_points_[j].x - origin_points_[j].y + b22)/sqrt(k2 * k2 + 1)));
//         point_weight = point_weight + min(minweight1,minweight2);
//       }
//       point_weight = point_weight/origin_points_.size();
//       //rectangles_weight.push_back(abs(((b11 - b12)/sqrt(k1 * k1 + 1)) * ((b21 - b22)/sqrt(k2 * k2 + 1))));
//       rectangles_weight.push_back(point_weight);

//       std::vector<Vertex> vertexs;
//       Vertex point1;
//       point1.x = (b21-b11)/(k1-k2);
//       point1.y = (k1*b21-k2*b11)/(k1-k2);
//       vertexs.push_back(point1);
//       Vertex point2;
//       point2.x = (b21-b12)/(k1-k2);
//       point2.y = (k1*b21-k2*b12)/(k1-k2);
//       vertexs.push_back(point2);
//       Vertex point3;
//       point3.x = (b22-b12)/(k1-k2);
//       point3.y = (k1*b22-k2*b12)/(k1-k2);
//       vertexs.push_back(point3);
//       Vertex point4;
//       point4.x = (b22-b11)/(k1-k2);
//       point4.y = (k1*b22-k2*b11)/(k1-k2);
//       vertexs.push_back(point4);

//       if(isnan(point1.x))
//       {
//         std::cout<<"nan!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<point1.x<<std::endl;
//       }

//       rectangles_points.push_back(vertexs);
//     }
//   }

//   double minrectangles = 10000000;
//   int num = -1;
//   for(size_t i = 0; i < rectangles_weight.size(); i++)
//   {
//     //std::cout<<"rectangles[i]:"<<rectangles_weight[i]<<std::endl;
//     if(rectangles_weight[i]<minrectangles)
//     {
//       minrectangles = rectangles_weight[i];
//       num = i;
//     }
//   }
//   std::vector<Vertex> rect1 = rectangles_points[num];
//   return rect1; 
// }

// vector<Vertex> generatePoints(const int num)
// {
//   vector<Vertex> points;
//   srand((unsigned)time(NULL));
//   for(int i = 0; i < num; i++)
//   {
//     Vertex point;
//     point.x = 200+20*(rand()/double(RAND_MAX)+0.2);
//     point.y = 200 + 200*(rand()/double(RAND_MAX)+0.2);
//     points.push_back(point);
//   }
//   for(int i = 0; i < num; i++)
//   {
//     Vertex point;
//     point.x = 200+400*(rand()/double(RAND_MAX)+0.2);
//     point.y = 350+50*(rand()/double(RAND_MAX)+0.2);
//     points.push_back(point);
//   }
//   return points;
// }

// // int main()
// // {
// //   vector<Vertex> points_generated = generatePoints(50);
// //   ConvexHull* convex_hull (new ConvexHull(points_generated));
// //   cv::Rect frame(0, 0, 800, 800);
// // 	// cv::Mat intensity_img = cv::Mat::zeros(frame.size(), CV_8UC3);
// // 	cv::Mat img = cv::Mat::zeros(frame.size(), CV_8UC3);
// //   for(int i = 0; i < points_generated.size(); i++)
// //   {
// //     img.at<cv::Vec3b>(cv::Point(points_generated[i].x, points_generated[i].y)) = cv::Vec3b(0, 0, 255);
// //   }
// //   cv::imshow("origin", img);

// //   vector<Vertex> points_hull = convex_hull->vertices_;
// //   points_hull = convex_hull->toRec1();
// //   for(int i = 0; i < points_hull.size()-1; i++)
// //   {
// //    cv::line(img, cv::Point(points_hull[i].x, points_hull[i].y),
// //                   cv::Point(points_hull[i+1].x, points_hull[i+1].y),
// //                   cv::Scalar(255,0,0),3);
// //   }
// //   cv::line(img, cv::Point((points_hull.end()-1)->x, (points_hull.end()-1)->y),
// //                   cv::Point(points_hull.begin()->x, points_hull.begin()->y),
// //                   cv::Scalar(255,0,0),3);

// //   Vertex point = convex_hull->getMedianPoint();
// //   img.at<cv::Vec3b>(cv::Point(point.x, point.y)) = cv::Vec3b(255, 0, 0);
// //   cv::imshow("line", img);
// //   cv::waitKey(0);
// // }

#include "box_type.h"
#include "min_box.h"
using namespace std;
// using namespace myshape;

// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
ConvexHull::ConvexHull(vector<Vertex> P)
{
  origin_points_ = P;
  size_t n = P.size(), k = 0;
  if (n <= 3) 
  {
    vertices_.assign(P.begin(), P.end());
    return;
  }

  vertices_.resize(2*n);
  // vector<Vertex> H(2*n);

  // Sort points lexicographically
  sort(P.begin(), P.end());

  // Build lower hull
  for (size_t i = 0; i < n; ++i) 
  {
    while (k >= 2 && cross(vertices_[k-2], vertices_[k-1], P[i]) <= 0) k--;
    vertices_[k++] = P[i];
  }

  // Build upper hull
  for (size_t i = n-1, t = k+1; i > 0; --i) {
    while (k >= t && cross(vertices_[k-2], vertices_[k-1], P[i-1]) <= 0) k--;
    vertices_[k++] = P[i-1];
  }

  vertices_.resize(k-1);
}

// must be big enough to hold 2*max(|coordinate|)^2

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
double ConvexHull::cross(const Vertex &O, const Vertex &A, const Vertex &B)
{
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

double ConvexHull::getDistance(const Vertex &A, const Vertex &B)
{
  return sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y));
}

// O在AB上的垂足
Vertex ConvexHull::foot(const Vertex &A, const Vertex &B, const Vertex &O)
{
  double k = (O.x * (B.x - A.x) + O.y * (B.y - A.y))/
            (pow((B.x - A.x), 2) + pow((B.y - A.y), 2));
  return k * (B - A);
}

//用PCA计算主方向，即朝向
cv::Mat ConvexHull::direction(vector<Vertex> points_input)
{
  cv::Mat pcaSet(points_input.size(), 2, CV_32FC1, cv::Scalar::all(0));

  double* data = nullptr;
  for(int i = 0; i < points_input.size(); i++)
  {
    data = pcaSet.ptr<double>(i);
    *data++ = points_input[i].x;
    *data++ = points_input[i].y;
  }
  cv::PCA pca(pcaSet, cv::Mat(), CV_PCA_DATA_AS_ROW, 2);
  //参数依次为：原始数据；原始数据均值，输入空会自己计算；每行/列代表一个样本；保留多少特征值，默认全保留

  cv::Mat direction_vector = pca.eigenvectors;
  return direction_vector;
}
// 给定朝向，拟合矩形
vector<Vertex> ConvexHull::fitRec(double k)
{
  std::vector<Vertex> rect;
  double k_1 = -1 / k;

  double b_1, b_2, b_3, b_4;
  b_1 = vertices_[0].y - k * vertices_[0].x;
  b_2 = vertices_[0].y - k * vertices_[0].x;
  b_3 = vertices_[0].y - k_1 * vertices_[0].x;
  b_4 = vertices_[0].y - k_1 * vertices_[0].x;

  for(int i = 0; i < vertices_.size(); i++)
  {
    //点在直线的左边
    if(k * vertices_[i].x + b_1 <= vertices_[i].y)
    {
      b_1 = vertices_[i].y - k * vertices_[i].x;
    }

    if(k * vertices_[i].x + b_2 >= vertices_[i].y)
    {
      b_2 = vertices_[i].y - k * vertices_[i].x;
    }

    if(k_1 * vertices_[i].x + b_3 <= vertices_[i].y)
    {
      b_3 = vertices_[i].y - k_1 * vertices_[i].x;
    }

    if(k_1 * vertices_[i].x + b_4 >= vertices_[i].y)
    {
      b_4 = vertices_[i].y - k_1 * vertices_[i].x;
    }
  }
  
  Vertex vertex;
  vertex.x = (b_3 - b_1) / (k - k_1);
  vertex.y = vertex.x * k + b_1;
  rect.push_back(vertex);

  vertex.x = (b_2 - b_3) / (k_1 - k);
  vertex.y = vertex.x * k + b_2;
  rect.push_back(vertex);

  vertex.x = (b_4 - b_2) / (k - k_1);
  vertex.y = vertex.x * k + b_2;
  rect.push_back(vertex);

  vertex.x = (b_1 - b_4) / (k_1 - k);
  vertex.y = vertex.x * k + b_1;
  rect.push_back(vertex);

  return rect;
}

// 根据PCA得到的方向绘制矩形
vector<Vertex> ConvexHull::toRec()
{
  std::vector<Vertex> rect;
  cv::Mat direct= direction(vertices_);
  double k = direct.at<double>(0,1)/direct.at<double>(0,0);
  rect = fitRec(k);
  return rect;
}

double ConvexHull::getArea(std::vector<Vertex> rect)
{
  return abs(cross(rect[1], rect[0], rect[2]));
}

// 得到面积最小fitting box
vector<Vertex> ConvexHull::Caliper()
{
  std::vector<Vertex> rect;
  double area = std::numeric_limits<double>::infinity();
  int len = vertices_.size();
  for(int i = 0; i < len; i++)
  {
    //计算斜率
    double k = (vertices_[(i+1) % len].y - vertices_[(i) % len].y)/
              (vertices_[(i+1) % len].x - vertices_[(i) % len].x);

    std::vector<Vertex> rect_temp = fitRec(k);
    double area_temp = getArea(rect_temp);
    if(area > area_temp)
    {
      area = area_temp;
      rect = rect_temp;
    }
  }
  return rect;
}


// Vertex ConvexHull::getMedianPoint()
// {
//   int n = origin_points_.size();
//   double w[n];
//   Vertex median;

//   for(int i = 0; i < 2; i++)
//   {
//     if(i == 0)
//     {
//       for(int j = 0; j < n; j++)
//       {
//         w[j] = 1;
//       }
//     }
//     else
//     {
//       for(int j = 0; j < n; j++)
//       {
//         w[j] =abs(median.x - origin_points_[j].x) + abs(median.y - origin_points_[j].y);
//       }
//     }

//     std::vector<double> pts_x;
//     std::vector<double> pts_y;
//     for(int j = 0; j < n; j++)
//     {
//       double pt_x_new = w[j] * origin_points_[j].x;
//       double pt_y_new = w[j] * origin_points_[j].y;
//       pts_x.push_back(pt_x_new);
//       pts_y.push_back(pt_y_new);
//     }
//     std::vector<double> pts_x_copy = pts_x;
//     std::vector<double> pts_y_copy = pts_y;
//     sort(pts_x.begin(), pts_x.end());
//     sort(pts_y.begin(), pts_y.end());

//     std::vector<double>::iterator it_x = find(pts_x_copy.begin(), pts_x_copy.end(), pts_x[n/2]);
//     std::vector<double>::iterator it_y = find(pts_y_copy.begin(), pts_y_copy.end(), pts_y[n/2]);

//     // median.x = pts_x[n/2] / w[it_x - pts_x_copy.begin()];
//     // median.y = pts_y[n/2] / w[it_y - pts_y_copy.begin()];
//     median.x = pts_x[n/2] / w[it_x - pts_x_copy.begin()];
//     median.y = pts_y[n/2] / w[it_y - pts_y_copy.begin()];
//   }
//   return median;
// }

double ConvexHull::minDisToRec(std::vector<Vertex> rect, Vertex point)
{
  double min_distance = std::numeric_limits<double>::infinity();
  for(int i = 0; i < 4; i++)
  {
    double dis1 = abs(cross(rect[i%4], rect[(i+1)%4], point)) / getDistance(rect[i%4], rect[(i+1)%4]);
    min_distance = (min_distance>dis1) ? dis1:min_distance;
  }
  return min_distance;
}
// 公式推导详见
// An Orientation Corrected Bounding Box Fit Based on the Convex Hull under Real Time Constraints
// Auther: Benjamin Naujoks and Hans-Joachim Wuensche
vector<Vertex> ConvexHull::toRec1()
{
  static double lambda = 0.01;
  std::vector<Vertex> rect = Caliper();
  Vertex higher_point, lower_point, third_point, median_point;
  
  //计算最高点和最低点
  double upper_min = std::numeric_limits<double>::infinity();
  double lower_min = std::numeric_limits<double>::infinity();;

  for(int i = 0; i < vertices_.size(); i++)
  {
    double upper_dis = abs(cross(vertices_[i], rect[0], rect[1]))/getDistance(rect[0], rect[1]);
    double lower_dis = abs(cross(vertices_[i], rect[2], rect[3]))/getDistance(rect[2], rect[3]);

    if(upper_dis < upper_min)
    {
      upper_min = upper_dis;
      higher_point = vertices_[i];
    }

    if(lower_dis < lower_min)
    {
      lower_min = lower_dis;
      lower_point = vertices_[i];
    }
  }

  //点到直线的距离减去垂足到两端点的最短距离和缩放系数的乘积
  double distance = 0;
  for(int i = 0; i < vertices_.size(); i++)
  {
    double min_cut = min(getDistance(foot(lower_point, higher_point, vertices_[i]) , lower_point),
                        getDistance(foot(lower_point, higher_point, vertices_[i]) , higher_point));

    double foot_distance = abs(cross(lower_point, higher_point, vertices_[i]))/
                      getDistance(lower_point, higher_point);
                      
    double dis_temp = foot_distance -  lambda * min_cut;

    if(dis_temp > distance)
    {
      third_point = vertices_[i];
      distance = dis_temp;
    }
  }

  std::vector<Vertex> rect1 = fitRec((higher_point.y-lower_point.y)/(higher_point.x-lower_point.x));
  std::vector<Vertex> rect2 = fitRec((higher_point.y-third_point.y)/(higher_point.x-third_point.x));
  std::vector<Vertex> rect3 = fitRec((lower_point.y-third_point.y)/(lower_point.x-third_point.x));
  double dis_sum1 = 0;
  double dis_sum2 = 0;
  double dis_sum3 = 0;
  for(int i = 0; i < origin_points_.size(); i++)
  {
    dis_sum1 += minDisToRec(rect1, origin_points_[i]);
    dis_sum2 += minDisToRec(rect2, origin_points_[i]);
    dis_sum3 += minDisToRec(rect3, origin_points_[i]);
  }

  if(dis_sum1 < dis_sum2 && dis_sum1 < dis_sum3)
  {
    return rect1;
  }
  else if(dis_sum2 < dis_sum1 && dis_sum2 < dis_sum3)
  {
    return rect2;
  }
  else
  {
    return rect3;
  }
}

vector<Vertex> ConvexHull::toRec2()
{
  std::vector<double> rectangles_weight;
  std::vector<std::vector<Vertex>> rectangles_points;
  //rectangles.resize(vertices_.size());
  //rectangles_points.resize(vertices_.size());
  for(int i = 0; i < vertices_.size(); i++)
  {
    if(i != vertices_.size()-1)
    {
      double k1;
      if(vertices_[i+1].x != vertices_[i].x&&vertices_[i+1].y != vertices_[i].y)
      {
        k1 = (vertices_[i+1].y - vertices_[i].y)/(vertices_[i+1].x - vertices_[i].x);
      }else if(vertices_[i+1].x == vertices_[i].x&&vertices_[i+1].y == vertices_[i].y)
      {
        k1 = (vertices_[i+1].y+0.0001 - vertices_[i].y)/(vertices_[i+1].x+0.0001 - vertices_[i].x);
      }else if(vertices_[i+1].x == vertices_[i].x)
      {
        k1 = (vertices_[i+1].y - vertices_[i].y)/(vertices_[i+1].x+0.0001 - vertices_[i].x);
      }else if(vertices_[i+1].y == vertices_[i].y)
      {
        k1 = (vertices_[i+1].y +0.0001 - vertices_[i].y)/(vertices_[i+1].x - vertices_[i].x);
      }
      double b11 = vertices_[i].y - k1 * vertices_[i].x;
      double maxdis1 = 0;
      int dispoint = i;
      for(int j = 0; j < vertices_.size(); j++)
      {
        double dis = abs((k1 * vertices_[j].x - vertices_[j].y + b11)/sqrt(k1 * k1 + 1));
        if(dis>maxdis1)
        {
          maxdis1 = dis;
          dispoint = j;
        }
      }
      double b12 = vertices_[dispoint].y - k1 * vertices_[dispoint].x;
      double k2 = -1/k1;
      double maxdis2 = 0;
      double mindis2 = 0;
      int maxdispoint = i;
      int mindispoint = i;
      double b1_ = vertices_[i].y - k2 * vertices_[i].x;
      for(int j = 0; j < vertices_.size(); j++)
      {
        double dis = (k2 * vertices_[j].x - vertices_[j].y + b1_)/sqrt(k2 * k2 + 1);
        if(dis>maxdis2)
        {
          maxdis2 = dis;
          maxdispoint = j;
        }else if(dis<mindis2)
        {
          mindis2 = dis;
          mindispoint = j;
        }
      }
      double b21 = vertices_[maxdispoint].y - k2 * vertices_[maxdispoint].x;
      double b22 = vertices_[mindispoint].y - k2 * vertices_[mindispoint].x;
      //栅格化
      
      double point_weight = 0;
      for(int j = 0; j < origin_points_.size(); j++)
      {
        double minweight1 = min(abs((k1 * origin_points_[j].x - origin_points_[j].y + b11)/sqrt(k1 * k1 + 1)),abs((k1 * origin_points_[j].x - origin_points_[j].y + b12)/sqrt(k1 * k1 + 1)));
        double minweight2 = min(abs((k2 * origin_points_[j].x - origin_points_[j].y + b21)/sqrt(k2 * k2 + 1)),abs((k2 * origin_points_[j].x - origin_points_[j].y + b22)/sqrt(k2 * k2 + 1)));
        point_weight = point_weight + min(minweight1,minweight2);
      }
      point_weight = point_weight/origin_points_.size();
      //rectangles_weight.push_back(abs(((b11 - b12)/sqrt(k1 * k1 + 1)) * ((b21 - b22)/sqrt(k2 * k2 + 1))));
      rectangles_weight.push_back(point_weight);

      std::vector<Vertex> vertexs;
      Vertex point1;
      point1.x = (b21-b11)/(k1-k2);
      point1.y = (k1*b21-k2*b11)/(k1-k2);
      vertexs.push_back(point1);
      Vertex point2;
      point2.x = (b21-b12)/(k1-k2);
      point2.y = (k1*b21-k2*b12)/(k1-k2);
      vertexs.push_back(point2);
      Vertex point3;
      point3.x = (b22-b12)/(k1-k2);
      point3.y = (k1*b22-k2*b12)/(k1-k2);
      vertexs.push_back(point3);
      Vertex point4;
      point4.x = (b22-b11)/(k1-k2);
      point4.y = (k1*b22-k2*b11)/(k1-k2);
      vertexs.push_back(point4);

      rectangles_points.push_back(vertexs);
    }else if (i == vertices_.size()-1)
    {
      double k1;
      if(vertices_[i+1].x != vertices_[i].x&&vertices_[i+1].y != vertices_[i].y)
      {
        k1 = (vertices_[i+1].y - vertices_[i].y)/(vertices_[i+1].x - vertices_[i].x);
      }else if(vertices_[i+1].x == vertices_[i].x&&vertices_[i+1].y == vertices_[i].y)
      {
        k1 = (vertices_[i+1].y+0.0001 - vertices_[i].y)/(vertices_[i+1].x+0.0001 - vertices_[i].x);
      }else if(vertices_[i+1].x == vertices_[i].x)
      {
        k1 = (vertices_[i+1].y - vertices_[i].y)/(vertices_[i+1].x+0.0001 - vertices_[i].x);
      }else if(vertices_[i+1].y == vertices_[i].y)
      {
        k1 = (vertices_[i+1].y +0.0001 - vertices_[i].y)/(vertices_[i+1].x - vertices_[i].x);
      }
      //double k1 = (vertices_[0].y - vertices_[i].y)/(vertices_[0].x - vertices_[i].x);
      double b11 = vertices_[i].y - k1 * vertices_[i].x;
      double maxdis1 = 0;
      int dispoint = i;
      for(int j = 0; j < vertices_.size(); j++)
      {
        double dis = abs((k1 * vertices_[j].x - vertices_[j].y + b11)/sqrt(k1 * k1 + 1));
        if(dis>maxdis1)
        {
          maxdis1 = dis;
          dispoint = j;
        }
      }
      double b12 = vertices_[dispoint].y - k1 * vertices_[dispoint].x;
      double k2 = -1/k1;
      double maxdis2 = 0;
      double mindis2 = 0;
      int maxdispoint = i;
      int mindispoint = i;
      double b1_ = vertices_[i].y - k2 * vertices_[i].x;
      for(int j = 0; j < vertices_.size(); j++)
      {
        double dis = (k2 * vertices_[j].x - vertices_[j].y + b1_)/sqrt(k2 * k2 + 1);
        if(dis>maxdis2)
        {
          maxdis2 = dis;
          maxdispoint = j;
        }else if(dis<mindis2)
        {
          mindis2 = dis;
          mindispoint = j;
        }
      }
      double b21 = vertices_[maxdispoint].y - k2 * vertices_[maxdispoint].x;
      double b22 = vertices_[mindispoint].y - k2 * vertices_[mindispoint].x;

      double point_weight = 0;
      for(int j = 0; j < origin_points_.size(); j++)
      {
        double minweight1 = min(abs((k1 * origin_points_[j].x - origin_points_[j].y + b11)/sqrt(k1 * k1 + 1)),abs((k1 * origin_points_[j].x - origin_points_[j].y + b12)/sqrt(k1 * k1 + 1)));
        double minweight2 = min(abs((k2 * origin_points_[j].x - origin_points_[j].y + b21)/sqrt(k2 * k2 + 1)),abs((k2 * origin_points_[j].x - origin_points_[j].y + b22)/sqrt(k2 * k2 + 1)));
        point_weight = point_weight + min(minweight1,minweight2);
      }
      point_weight = point_weight/origin_points_.size();
      //rectangles_weight.push_back(abs(((b11 - b12)/sqrt(k1 * k1 + 1)) * ((b21 - b22)/sqrt(k2 * k2 + 1))));
      rectangles_weight.push_back(point_weight);

      std::vector<Vertex> vertexs;
      Vertex point1;
      point1.x = (b21-b11)/(k1-k2);
      point1.y = (k1*b21-k2*b11)/(k1-k2);
      vertexs.push_back(point1);
      Vertex point2;
      point2.x = (b21-b12)/(k1-k2);
      point2.y = (k1*b21-k2*b12)/(k1-k2);
      vertexs.push_back(point2);
      Vertex point3;
      point3.x = (b22-b12)/(k1-k2);
      point3.y = (k1*b22-k2*b12)/(k1-k2);
      vertexs.push_back(point3);
      Vertex point4;
      point4.x = (b22-b11)/(k1-k2);
      point4.y = (k1*b22-k2*b11)/(k1-k2);
      vertexs.push_back(point4);

      rectangles_points.push_back(vertexs);
    }
  }

  double minrectangles = 10000;
  int num = -1;
  for(int i = 0; i < rectangles_weight.size(); i++)
  {
    if(rectangles_weight[i]<minrectangles)
    {
      minrectangles = rectangles_weight[i];
      num = i;
    }
  }
  std::vector<Vertex> rect1 = rectangles_points[num];
  return rect1; 
}

vector<Vertex> generatePoints(const int num)
{
  vector<Vertex> points;
  srand((unsigned)time(NULL));
  for(int i = 0; i < num; i++)
  {
    Vertex point;
    point.x = 200+20*(rand()/double(RAND_MAX)+0.2);
    point.y = 200 + 200*(rand()/double(RAND_MAX)+0.2);
    points.push_back(point);
  }
  for(int i = 0; i < num; i++)
  {
    Vertex point;
    point.x = 200+400*(rand()/double(RAND_MAX)+0.2);
    point.y = 350+50*(rand()/double(RAND_MAX)+0.2);
    points.push_back(point);
  }
  return points;
}

// int main()
// {
//   vector<Vertex> points_generated = generatePoints(50);
//   ConvexHull* convex_hull (new ConvexHull(points_generated));
//   cv::Rect frame(0, 0, 800, 800);
// 	// cv::Mat intensity_img = cv::Mat::zeros(frame.size(), CV_8UC3);
// 	cv::Mat img = cv::Mat::zeros(frame.size(), CV_8UC3);
//   for(int i = 0; i < points_generated.size(); i++)
//   {
//     img.at<cv::Vec3b>(cv::Point(points_generated[i].x, points_generated[i].y)) = cv::Vec3b(0, 0, 255);
//   }
//   cv::imshow("origin", img);

//   vector<Vertex> points_hull = convex_hull->vertices_;
//   points_hull = convex_hull->toRec1();
//   for(int i = 0; i < points_hull.size()-1; i++)
//   {
//    cv::line(img, cv::Point(points_hull[i].x, points_hull[i].y),
//                   cv::Point(points_hull[i+1].x, points_hull[i+1].y),
//                   cv::Scalar(255,0,0),3);
//   }
//   cv::line(img, cv::Point((points_hull.end()-1)->x, (points_hull.end()-1)->y),
//                   cv::Point(points_hull.begin()->x, points_hull.begin()->y),
//                   cv::Scalar(255,0,0),3);

//   Vertex point = convex_hull->getMedianPoint();
//   img.at<cv::Vec3b>(cv::Point(point.x, point.y)) = cv::Vec3b(255, 0, 0);
//   cv::imshow("line", img);
//   cv::waitKey(0);
// }





