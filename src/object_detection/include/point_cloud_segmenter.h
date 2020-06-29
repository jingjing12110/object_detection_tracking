#include <vector>
#include "nanoflann.hpp"
#include "vec3.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


struct ScanlinePointCloud {
  struct ScanlinePoint {
    Vec3 point;
    size_t scanline_index;

    ScanlinePoint(Vec3 pt, size_t index) : point(pt), scanline_index(index) {}
  };

  std::vector<Vec3>& scanline_points;
  std::vector<ScanlinePoint> filtered_points;

  ScanlinePointCloud(std::vector<Vec3>& pts) : scanline_points(pts) {
    // Filter out ground labels
    for (size_t i = 0; i < pts.size(); i++) {
      Vec3 point = pts[i];
      if (point.label != -3) {
        ScanlinePoint wrapper(point, i);
        filtered_points.push_back(wrapper);
      }
    }
  }

  // Returns the label associated with the results index from a kdtree lookup.
  // The issue with using the results Vec3 directly is that it contains no label
  // information. We need to access the full scanline points vector to determine
  // Vec3's label.
  float get_label(size_t results_index) {
    size_t scanline_index = filtered_points[results_index].scanline_index;
    return scanline_points[scanline_index].label;
  }

  // Returns the squared distance between a point p1 in float[] form and the
  // point at idx_p2
  inline float kdtree_distance(const float* p1, const size_t idx_p2,
                               size_t size) const {
    return filtered_points[idx_p2].point.distance_squared(p1);
  }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const {
    return filtered_points.size();
  }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the
  //  "if/else's" are actually solved at compile time.
  inline float kdtree_get_pt(const size_t idx, int dim) const {
    if (dim == 0)
      return filtered_points[idx].point.x;
    else if (dim == 1)
      return filtered_points[idx].point.y;
    else
      return filtered_points[idx].point.z;
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in
  //   "bb" so it can be avoided to redo it again. Look at bb.size() to find out
  //   the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const {
    return false;
  }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, ScanlinePointCloud>, ScanlinePointCloud,
    3>
    ScanlineKDTree;

struct Scanline {
  std::vector<int> s_queue;
  std::vector<int> e_queue;
  std::vector<int> labels;
  std::vector<Vec3> points;

  ScanlinePointCloud* tree_point_cloud;
  ScanlineKDTree* tree;

  Scanline() : tree_point_cloud(nullptr), tree(nullptr) {}

  ~Scanline() {
    delete tree;
    delete tree_point_cloud;
  }
};

class PointCloudSegmenter {
 private:
  std::vector<Vec3> p_gnd;   // Pts belonging to ground surface
  std::vector<Vec3> p_ngnd;  // Pts not belonging to ground surface

 public:
  std::vector<Vec3> p_all;
  double th_seeds;  // Threshold for points considered to be initial seeds
  double th_dist;   // Threshold distance from plane

  int n_iter;  // Num of iterations (segments in x dir)
  int n_lpr;   // Num of pts used to determine LPR
  int n_segs;  // Num of segments to split data into

  int n_scanlines;
  int new_label;
  double th_run;
  double th_merge;

  int max_x;
  int max_y;

  PointCloudSegmenter() {
    n_iter = 0;
    n_lpr = 0;
    n_segs = 0;
    th_seeds = 0;
    th_dist = 0;
    max_x = 40;
    max_y = 20;
    n_scanlines = 120;
    th_run = 0.5;
    th_merge = 1.0;
  }

  PointCloudSegmenter(int m_x, int m_y, int iterations, int num_lpr,
                      int num_segs, double seed_thresh, double dist_thresh,
                      int n_scan, double run_thresh,
                      double merge_thresh) {  // Constructor
    n_iter = iterations;
    n_lpr = num_lpr;
    n_segs = num_segs;
    th_seeds = seed_thresh;
    th_dist = dist_thresh;
    max_x = m_x;
    max_y = m_y;
    n_scanlines = n_scan;
    th_run = run_thresh;
    th_merge = merge_thresh;
  }

  ~PointCloudSegmenter() {}  // Deconstructor

  //********************** GPF ***************************

  void GroundPlaneFitting(std::vector<Vec3>& cloud);  // Main Loop

  void ExtractInitialSeeds(
      std::vector<Vec3>& cloud_seg,
      std::vector<Vec3>& seeds);  // Returns inital seeds to be used in first
                                  // plane model estimation

  Vec3 CalculatePlaneNormal(std::vector<Vec3>& cur_p_gnd);
  // Returns the normal of the estimated ground plane model

  std::vector<Vec3> GetGroundPoints(void) {
    // Returns points that have been classified as ground points
    return p_gnd;
  }

  std::vector<Vec3> GetNonGroundPoints(void) {
    // Returns points that have been classified as non ground points
    return p_ngnd;
  }

  //********************** SLR ***************************

  std::vector<int> next;
  std::vector<int> tail;
  std::vector<int> rtable;

  std::vector<Vec3> ScanLineRun(std::vector<Vec3>& cloud);  // Main loop

  void FindRuns(Scanline& cur_scanline);

  void UpdateLabels(Scanline& scan_current, Scanline& scan_above);

  void SetRunLabel(Scanline& scan_current, int start_index, int end_index,
                   int label);

  int FindNearestNeighbor(Scanline& scan_current, Scanline& scan_above,
                          int start_index, int end_index, int point_index);

  void MergeLabels(std::vector<int>& labels_to_merge, int min_label);

  void MergeOperation(int u, int v);

  void ResolveMerge(int x, int y);

  std::vector<Vec3> ExtractClusters(std::vector<Scanline>& scanlines);
};


void parseInput(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                std::vector<Vec3>& in, PointCloudSegmenter segmenter) {
#pragma omp for
  for (size_t i = 0; i < cloud->size(); i++) {
    Vec3 pt = Vec3(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    if (pt.r > 0) {
      if (pt.x < segmenter.max_x && pt.x > -segmenter.max_x &&
          pt.y < segmenter.max_y && pt.y > -segmenter.max_y && pt.z > -2) {
        if (pt.theta < 3 && pt.theta > -25) {
          in.push_back(pt);
        }
      }
    }
  }  // end for i
}


void parseOut(std::vector<Vec3>& in, pcl::PointCloud<pcl::PointXYZI>::Ptr out) {
#pragma omp for
  for (size_t i = 0; i < in.size(); i++) {
    pcl::PointXYZI pt;
    pt.x = in[i].x;
    pt.y = in[i].y;
    pt.z = in[i].z;
    pt.intensity = in[i].label;
    out->push_back(pt);
  }
}
