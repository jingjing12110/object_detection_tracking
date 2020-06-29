#include <vector>
#include <fstream>
#include <iostream>
#include "point_cloud_segmenter.h"

void parseInput(std::vector<Vec3>& in, PointCloudSegmenter segmenter);
void generateOutput(std::vector<Vec3>& pts, std::string file_name);

int main() {

  int n_iters = 3;
  int n_lpr = 20;
  int n_segs = 3;
  double seed_thresh = 0.4; //meters
  double dist_thresh = 0.2; //meters

  double th_run = 0.5;
  double th_merge = 1.0;
  int x_max = 30;
  int y_max = 15;
  int n_scanlines = 50;

  std::vector<Vec3> input_cloud;
  std::vector<Vec3> predicted_ground;
  std::vector<Vec3> predicted_not_ground;
  std::vector<Vec3> predicted_clusters;

  PointCloudSegmenter segmenter(x_max, y_max, n_iters, n_lpr, n_segs, seed_thresh, dist_thresh, n_scanlines, th_run, th_merge);

  parseInput(input_cloud, segmenter);
  std::cout << "Input size: " << input_cloud.size() << std::endl;

  segmenter.GroundPlaneFitting(input_cloud);

  predicted_ground = segmenter.GetGroundPoints();
  predicted_not_ground = segmenter.GetNonGroundPoints();

  predicted_clusters = segmenter.ScanLineRun(segmenter.p_all);
  generateOutput(predicted_clusters, "predicted_clusters.txt");

  return 0;

}

//Convert input KITTI velodyne data files to vector<Vec3>
void parseInput(std::vector<Vec3>& in, PointCloudSegmenter seg) {
  std::ifstream infile("../velodyne_points_1/data/0000000001.txt");
  double x, y, z, v;
  int k = 0;

  while (infile >> x >> y >> z >> v)
  {
    Vec3 pt = Vec3(x, y, z);
    if (pt.r > 0) {
      if (pt.x < seg.max_x && pt.x > -seg.max_x && pt.y < seg.max_y && pt.y > -seg.max_y && pt.z > -2) {
        if (pt.theta < 3 && pt.theta > -25) {
          in.push_back(pt);
        }
      }
    }

  }
}

//Write Contents of input vector to file
void generateOutput(std::vector<Vec3>& pts, std::string file_name) {
  std::ofstream file;
  file.open (file_name);
  std::vector<Vec3>::iterator it;
  for (it = pts.begin(); it != pts.end(); it++) {
    file << it->x << " " << it->y << " " << it->z << " " << it->label << "\n";
  }
  file.close();
}
