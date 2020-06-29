#include "min_box.h"
#include <ctime>

using namespace std;

BoundingBox::BoundingBox(tracking_msgs::DetectedObjectArray& input,
                         ros::Time time) {
  for (size_t i = 0; i < input.objects.size(); i++) {
    if (input.objects[i].pointcloud.data.size() > 0) {
      std::vector<Vertex> point_input;

      pcl::PointCloud<pcl::PointXYZI>::Ptr scan(
          new pcl::PointCloud<pcl::PointXYZI>());
      // std::cout << "input.objects[i].pointcloud.size: "
      //           << input.objects[i].pointcloud.data.size() << std::endl;

      pcl::fromROSMsg(input.objects[i].pointcloud, *scan);
      // std::cout << "scan->size(): " << scan->size() << std::endl;
      for (int j = 0; j < scan->size(); j++) {
        point_input.push_back(Vertex(scan->points[j].x, scan->points[j].y));
      }
      ConvexHull* convex_hull(new ConvexHull(point_input));

      pcl::PointXYZI min_pt;
      pcl::PointXYZI max_pt;
      pcl::getMinMax3D(*scan, min_pt, max_pt);

      MyPrism prism;
      prism.vertices = convex_hull->vertices_;
      prism.bottom = min_pt.z;
      prism.top = max_pt.z;
      prisms_.prisms_.push_back(prism);
      prisms_.time = time;

      prism.vertices = convex_hull->toRec2();
      cuboids_.prisms_.push_back(prism);
      cuboids_.time = time;
    }
  }  // end for i
}

// BoundingBox::BoundingBox(
//     std::vector<pcl::PointCloud<pcl::PointXYZI> > clusters) {
//   for (size_t i = 0; i < clusters.size(); i++) {
//     std::vector<Vertex> point_input;
//     // if(clusters[i].size() < 10) continue;
//     Vertex origin(clusters[i][0].x, clusters[i][0].y);
//     for (size_t j = 0; j < clusters[i].size(); j++) {
//       point_input.push_back(
//           Vertex(clusters[i][j].x - origin.x, clusters[i][j].y - origin.y));
//     }
//     ConvexHull* convex_hull(new ConvexHull(point_input));

//     pcl::PointXYZ min_pt;
//     pcl::PointXYZ max_pt;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr temp1(
//         new pcl::PointCloud<pcl::PointXYZI>());
//     *temp1 = clusters[i];
//     pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(
//         new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::copyPointCloud(*temp1, *temp2);
//     pcl::getMinMax3D(*temp2, min_pt, max_pt);

//     MyPrism prism;
//     for (size_t j = 0; j < convex_hull->vertices_.size(); j++) {
//       prism.vertices.push_back(convex_hull->vertices_[j] + origin);
//     }
//     // prism.vertices = convex_hull->vertices_;
//     prism.bottom = min_pt.z;
//     prism.top = max_pt.z;
//     prism.center_z = (max_pt.z + min_pt.z) / 2;
//     prism.height = abs(max_pt.z - min_pt.z);
//     prisms_.prisms_.push_back(prism);
//     // std::cout << "--center_z: " << prism.center_z << " height: " <<
//     // prism.height << std::endl;

//     prism.vertices = convex_hull->toRec2();
//     for (size_t j = 0; j < prism.vertices.size(); j++) {
//       prism.vertices[j] = prism.vertices[j] + origin;
//     }

//     cuboids_.prisms_.push_back(prism);
//   }
// }

visualization_msgs::MarkerArray Prism::toRviz() {
  int box_num = prisms_.size();
  visualization_msgs::MarkerArray multi_bbox;
  multi_bbox.markers.resize(box_num + 1);

  multi_bbox.markers[0].header.frame_id = "velo_middle";
  multi_bbox.markers[0].header.stamp = Prism::time;
  multi_bbox.markers[0].action = visualization_msgs::Marker::DELETEALL;

  for (int i = 0; i < box_num; i++) {
    multi_bbox.markers[i + 1].header.frame_id = "velo_middle";
    multi_bbox.markers[i + 1].header.stamp = Prism::time;
    multi_bbox.markers[i + 1].action = visualization_msgs::Marker::ADD;
    multi_bbox.markers[i + 1].pose.orientation.w = 1.0;
    multi_bbox.markers[i + 1].id = i;
    multi_bbox.markers[i + 1].type = visualization_msgs::Marker::LINE_STRIP;

    multi_bbox.markers[i + 1].scale.x = 0.1;
    multi_bbox.markers[i + 1].color.r = 1.0f;
    multi_bbox.markers[i + 1].color.g = 0.0f;
    multi_bbox.markers[i + 1].color.b = 1.0f;
    multi_bbox.markers[i + 1].color.a = 1.0;

    int vertices_size = prisms_[i].vertices.size();
    multi_bbox.markers[i + 1].points.resize(vertices_size * 5);
    for (int j = 0; j < vertices_size; j++) {
      multi_bbox.markers[i + 1].points[5 * j].x =
          prisms_[i].vertices[j % vertices_size].x;
      multi_bbox.markers[i + 1].points[5 * j].y =
          prisms_[i].vertices[j % vertices_size].y;
      multi_bbox.markers[i + 1].points[5 * j].z = prisms_[i].top;
      multi_bbox.markers[i + 1].points[5 * j + 1].x =
          prisms_[i].vertices[j % vertices_size].x;
      multi_bbox.markers[i + 1].points[5 * j + 1].y =
          prisms_[i].vertices[j % vertices_size].y;
      multi_bbox.markers[i + 1].points[5 * j + 1].z = prisms_[i].bottom;
      multi_bbox.markers[i + 1].points[5 * j + 2].x =
          prisms_[i].vertices[(j + 1) % vertices_size].x;
      multi_bbox.markers[i + 1].points[5 * j + 2].y =
          prisms_[i].vertices[(j + 1) % vertices_size].y;
      multi_bbox.markers[i + 1].points[5 * j + 2].z = prisms_[i].bottom;
      multi_bbox.markers[i + 1].points[5 * j + 3].x =
          prisms_[i].vertices[(j + 1) % vertices_size].x;
      multi_bbox.markers[i + 1].points[5 * j + 3].y =
          prisms_[i].vertices[(j + 1) % vertices_size].y;
      multi_bbox.markers[i + 1].points[5 * j + 3].z = prisms_[i].top;
      multi_bbox.markers[i + 1].points[5 * j + 4].x =
          prisms_[i].vertices[j % vertices_size].x;
      multi_bbox.markers[i + 1].points[5 * j + 4].y =
          prisms_[i].vertices[j % vertices_size].y;
      multi_bbox.markers[i + 1].points[5 * j + 4].z = prisms_[i].top;
    }
  }

  // std::cout << "bounding_box.prisms_.size: " << box_num << std::endl;

  return multi_bbox;
}
