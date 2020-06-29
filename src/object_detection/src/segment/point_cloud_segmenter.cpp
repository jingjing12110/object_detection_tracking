/***************************************************************************************
  Author: Mitchell Sayer
  Program Name: point_cloud_segmenter.cpp
  Brief:
  Date: MAR 1 2018
***************************************************************************************/

#include "point_cloud_segmenter.h"
#include <math.h>
#include <algorithm>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include "nanoflann.hpp"

//************************************************************************************************

Vec3 cloud_centroid;

bool CompareZ(Vec3 const& a, Vec3 const& b) { return a.z < b.z; }

bool CompareX(Vec3 const& a, Vec3 const& b) { return a.x < b.x; }

bool CompareTheta(Vec3 const& a, Vec3 const& b) {
  return a.theta_horizontal < b.theta_horizontal;
}

// Allows Vec3 to be printed directly to std::cout or std::cerr
std::ostream& operator<<(std::ostream& os, const Vec3& vec) {
  return os << vec.x << ";" << vec.y << ";" << vec.z << ": " << vec.label;
}

/*
  Main implementation of GPF
    - Split data into n_segs along the x-axis (dir of travel)
    - Get initial seeds and use them to create first plane estimation
    - Repeat for n_iters using the estimated gnd points from each iter to
        calculate the new estimated
*/
void PointCloudSegmenter::GroundPlaneFitting(std::vector<Vec3>& cloud) {
  int start_s = clock();  // Take start timestamp

  int seg_size = (cloud.size() / n_segs);
  std::vector<Vec3> cloud_segs[n_segs];

  std::vector<Vec3>::iterator it;
  int i = 0, cur_seg = 0;

  for (it = cloud.begin(); it != cloud.end(); it++, i++) {
    if (i == seg_size) {
      if ((cur_seg + 1) != n_segs) {
        cur_seg++;
        i = 0;
      }
    }

    cloud_segs[cur_seg].push_back(*it);
  }

  std::vector<Vec3> cur_p_gnd;  // Pts belonging to ground surface in current
                                // segment iteration
  std::vector<Vec3> cur_p_ngnd;  // Pts not belonging to ground surface in
                                 // current segment iteration
  std::vector<Vec3> cur_p_list;
  std::vector<Vec3>& cur_cloud_seg = cloud_segs[0];

  // Loop through each segment and apply GPF
  for (i = 0; i < n_segs; i++) {
    cur_cloud_seg = cloud_segs[i];

    // Sort points in current segment by x value for segmentation
    std::sort(cur_cloud_seg.begin(), cur_cloud_seg.end(), CompareX);

    // Get initial ground points
    ExtractInitialSeeds(cur_cloud_seg, cur_p_gnd);

    for (int i = 0; i < n_iter; i++) {
      Vec3 normal = CalculatePlaneNormal(cur_p_gnd);

      double d =
          -((normal.x * cloud_centroid.x) + (normal.y * cloud_centroid.y) +
            (normal.z * cloud_centroid.z));
      double normal_mag =
          sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);

      if (normal_mag != 0) {
        // Clear gnd points in between iterations
        cur_p_gnd.clear();
        cur_p_ngnd.clear();
        cur_p_list.clear();

        // Calculate distance from point to orthogonal projection
        // and compare to th_dist
        double dist;
        std::vector<Vec3>::iterator seg_it;
        for (seg_it = cur_cloud_seg.begin(); seg_it != cur_cloud_seg.end();
             seg_it++) {
          dist = std::abs(normal.x * seg_it->x + normal.y * seg_it->y +
                          normal.z * seg_it->z + d) /
                 normal_mag;

          if (dist < th_dist) {
            seg_it->label = -3;
            cur_p_gnd.push_back(*seg_it);
            cur_p_list.push_back(*seg_it);
          } else {
            cur_p_ngnd.push_back(*seg_it);
            cur_p_list.push_back(*seg_it);
          }
        }
      }
    }

    // Add results from current segment to main sets
    this->p_gnd.insert(p_gnd.end(), cur_p_gnd.begin(), cur_p_gnd.end());
    this->p_ngnd.insert(p_ngnd.end(), cur_p_ngnd.begin(), cur_p_ngnd.end());
    this->p_all.insert(p_all.end(), cur_p_list.begin(), cur_p_list.end());

    cur_p_gnd.clear();
    cur_p_ngnd.clear();
    cur_p_list.clear();
  }

  int stop_s = clock();  // Take end timestamp
  std::cout << "GPF runtime: "
            << (stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 << " ms"
            << std::endl;
}

/*
  Calculate Lowest Point Representative and calculate inital seed points using
  th_seeds
*/
void PointCloudSegmenter::ExtractInitialSeeds(std::vector<Vec3>& cloud_seg,
                                              std::vector<Vec3>& seeds) {
  if (!cloud_seg.empty()) {
    // Sort points in current segment by height
    std::sort(cloud_seg.begin(), cloud_seg.end(), CompareZ);

    Vec3 lpr;

    // Calc Lowest Point Representative (LPR)
    std::vector<Vec3>::iterator it;
    int i = 0;

    for (it = cloud_seg.begin(); it != cloud_seg.end(); it++) {
      if (it->z > -4) {
        if (i == n_lpr) break;

        lpr.x += it->x;
        lpr.y += it->y;
        lpr.z += it->z;

        i++;
      }
    }

    lpr.x /= n_lpr;
    lpr.y /= n_lpr;
    lpr.z /= n_lpr;

    // Add point as initial seed if its dist to lpr is < th_seeds
    for (it = cloud_seg.begin(); it != cloud_seg.end(); it++) {
      if (it->z - lpr.z < this->th_seeds) {
        seeds.push_back(*it);
      }
    }
  }
}

/*
  Calculating Estimated Plane Normal:
    1. Calculate the centroid of the points
    2. Calculate the covariance matrix of the points relative to the centroid
    3. Find the main axis (the component of the plane normal which will have the
  largest absolute value) and do simple linear regression along that axis
*/
Vec3 PointCloudSegmenter::CalculatePlaneNormal(std::vector<Vec3>& cur_p_gnd) {
  int n = cur_p_gnd.size();

  if (n > 3) {
    // Calculate centroid
    Vec3 centroid;

    std::vector<Vec3>::iterator it;

    for (it = cur_p_gnd.begin(); it != cur_p_gnd.end(); it++) {
      centroid.x += it->x;
      centroid.y += it->y;
      centroid.z += it->z;
    }

    centroid.x /= n;
    centroid.y /= n;
    centroid.z /= n;

    cloud_centroid = centroid;

    // Calc covarience matrix
    double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;

    Vec3 r;

    for (it = cur_p_gnd.begin(); it != cur_p_gnd.end(); it++) {
      r.x = it->x - centroid.x;
      r.y = it->y - centroid.y;
      r.z = it->z - centroid.z;

      xx += r.x * r.x;
      xy += r.x * r.y;
      xz += r.x * r.z;
      yy += r.y * r.y;
      yz += r.y * r.z;
      zz += r.z * r.z;
    }

    double det_x = yy * zz - yz * yz;
    double det_y = xx * zz - xz * xz;
    double det_z = xx * yy - xy * xy;

    double det_max = std::max(det_x, std::max(det_y, det_z));

    Vec3 norm;

    // Pick path with best conditioning
    if (det_max == det_x) {
      norm.x = det_x;
      norm.y = xz * yz - xy * zz;
      norm.z = xy * yz - xz * yy;
    } else if (det_max == det_y) {
      norm.x = xz * yz - xy * zz;
      norm.y = det_y;
      norm.z = xy * xz - yz * xx;
    } else {
      norm.x = xy * yz - xz * yy;
      norm.y = xy * xz - yz * xx;
      norm.z = det_z;
    }

    double dir_mag = sqrt(pow(norm.x, 2) + pow(norm.y, 2) + pow(norm.z, 2));

    norm.x /= dir_mag;
    norm.y /= dir_mag;
    norm.z /= dir_mag;

    return norm;
  }

  return Vec3();
}

//**************************************************************** SCAN LINE RUN
//**********************************************************************

/*
  Main Implementation of Scan Line Run:

  TODO: Properly identify scan lines, Change label implementation
*/
std::vector<Vec3> PointCloudSegmenter::ScanLineRun(std::vector<Vec3>& cloud) {
  int start_s = clock();  // Take start timestamp

  this->new_label = 0;
  this->next.push_back(-1);
  this->tail.push_back(this->new_label);
  this->rtable.push_back(this->new_label);

  std::vector<Scanline> scanlines;

  std::vector<Vec3>::iterator v_it;
  std::vector<Scanline>::iterator s_it;
  std::vector<int>::iterator r_it;
  int i;

  // Organize points into scanlines
  Scanline scanline;
  scanline.points = std::vector<Vec3>();
  scanline.s_queue = std::vector<int>();
  scanline.e_queue = std::vector<int>();
  i = 0;

  for (v_it = cloud.begin(); v_it != cloud.end(); v_it++, i++) {
    int scan_index;

    // if (v_it->theta <= 2 && v_it->theta > -8.333) {
    scan_index = (int)abs((v_it->theta - 3) / (0.6));
    // } else if (v_it->theta <= -8.333 && v_it->theta >= -24.333 ) {
    //   scan_index = 31 + (int)round(abs((v_it->theta + 8.333) / (0.5)));
    // }

    if (scan_index >= scanlines.size()) {
      scanlines.resize(scan_index + 1);
      scanlines.insert(scanlines.begin() + scan_index, scanline);
    }
    scanlines[scan_index].points.push_back(*v_it);
  }

  // Index scanlines by theta
  for (s_it = scanlines.begin(); s_it != scanlines.end(); s_it++) {
    if (s_it->points.size() == 0) {
      //(s_it - 1)->points.insert((s_it - 1)->points.end(),
      //s_it->points.begin(), s_it->points.end());
      scanlines.erase(s_it);
      s_it--;
    } else {
      std::sort(s_it->points.begin(), s_it->points.end(), CompareTheta);

      if (s_it->points.size() > 0) {
        ScanlinePointCloud* tree_point_cloud =
            new ScanlinePointCloud(s_it->points);
        ScanlineKDTree* tree =
            new ScanlineKDTree(3, *tree_point_cloud,
                               nanoflann::KDTreeSingleIndexAdaptorParams(10));
        tree->buildIndex();

        s_it->tree_point_cloud = tree_point_cloud;
        s_it->tree = tree;
      }
    }
  }

  std::cout << "Num Scanlines: " << scanlines.size() << std::endl;

  // Assign inital labels
  Scanline* scan_above = &scanlines[0];
  Scanline* scan_current = nullptr;
  FindRuns(*scan_above);
  int start_index;
  int end_index;

  for (int i = 0; i < scan_above->s_queue.size(); i++) {
    start_index = scan_above->s_queue[i];
    end_index = scan_above->e_queue[i];

    SetRunLabel(*scan_above, start_index, end_index, this->new_label);
    this->new_label += 1;
    this->next.push_back(-1);
    this->tail.push_back(this->new_label);
    this->rtable.push_back(this->new_label);
  }

  // Find runs for all subsequent scanlines and propogate labels
  for (int i = 1; i < scanlines.size(); i++) {
    scan_current = &(scanlines[i]);
    FindRuns(*scan_current);
    UpdateLabels(*scan_current, *scan_above);
    scan_above = scan_current;
  }

  // Take end timestamp
  int stop_s = clock();
  std::cout << "SLR runtime: "
            << (stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 << " ms"
            << std::endl;

  // Extract and return clusters
  return ExtractClusters(scanlines);
}

/*
  Find Runs:
    Find runs and add start and end indices to s_queue and e_queue respectively
    Take into account: ground points, edge cases

  TODO: Redo logic and make prettier
*/
void PointCloudSegmenter::FindRuns(Scanline& cur_scanline) {
  int i = 0;

  std::vector<Vec3>::iterator it;

  while (cur_scanline.points[i].label == -3) {
    i++;
  }

  if (i < cur_scanline.points.size()) {
    cur_scanline.s_queue.push_back(i);

    if (i == cur_scanline.points.size() - 1) {
      cur_scanline.e_queue.push_back(i);
      return;
    }

    i += 1;
  } else {
    return;
  }

  for (; i < cur_scanline.points.size(); i++) {
    Vec3* cur_p = &cur_scanline.points[i];
    Vec3* prev_p = &cur_scanline.points[i - 1];

    if (cur_p->label != -3) {
      if (prev_p->label == -3) {
        // Start new run if immediately after gnd pt
        cur_scanline.s_queue.push_back(i);
        if (i == cur_scanline.points.size() - 1) {
          cur_scanline.e_queue.push_back(i);
        }
      } else {
        // check distance to prev point
        if ((cur_scanline.points[i]).distance(cur_scanline.points[i - 1]) >
            this->th_run) {
          cur_scanline.e_queue.push_back(i - 1);
          cur_scanline.s_queue.push_back(i);
        }

        if (i == cur_scanline.points.size() - 1) {
          cur_scanline.e_queue.push_back(i);
        }
      }
    } else {
      // Current point is ground & previous point is not
      // then end that run
      if (prev_p->label != -3) {
        cur_scanline.e_queue.push_back(i - 1);
      }
    }
  }
}

/*
  Update Labels:
    1. Loop through all runs
    2. For each point in each run, find its nearest neigbor
       in the above scanline
    3. If there are neigbors within th->merge:
         Pick the min label of the label candidates and merge all labels to that
  one Else: Set all labels in run to newlabel and increment newlabel
*/
void PointCloudSegmenter::UpdateLabels(Scanline& scan_current,
                                       Scanline& scan_above) {
  std::vector<int> labels_to_merge;
  std::vector<int>::iterator i_it;
  int start_index;
  int end_index;

  if (scan_current.s_queue.size() != scan_current.e_queue.size()) {
    std::cout << "Start & End: " << scan_current.s_queue.size() << "  "
              << scan_current.e_queue.size() << std::endl;
    return;
  }

  for (int i = 0; i < scan_current.s_queue.size(); i++) {
    start_index = scan_current.s_queue[i];
    end_index = scan_current.e_queue[i];

    for (int k = start_index; k <= end_index; k++) {
      if (scan_current.points[k].label != -3) {
        int nearest_label = FindNearestNeighbor(scan_current, scan_above,
                                                start_index, end_index, k);

        if (nearest_label != -1 &&
            std::find(labels_to_merge.begin(), labels_to_merge.end(),
                      nearest_label) == labels_to_merge.end()) {
          labels_to_merge.push_back(nearest_label);
        }
      }
    }

    if (labels_to_merge.size() == 0) {
      SetRunLabel(scan_current, start_index, end_index, this->new_label);
      this->new_label += 1;
      this->next.push_back(-1);
      this->tail.push_back(this->new_label);
      this->rtable.push_back(this->new_label);
    }

    else {
      int min_label =
          *std::min_element(labels_to_merge.begin(), labels_to_merge.end());
      ;

      SetRunLabel(scan_current, start_index, end_index, min_label);
      MergeLabels(labels_to_merge, min_label);
    }

    labels_to_merge.clear();
  }
}

/*
  Merge all labels in the input array into the established min label
*/
void PointCloudSegmenter::MergeLabels(std::vector<int>& labels_to_merge,
                                      int min_label) {
  std::vector<int>::iterator it;

  for (int i = 0; i < labels_to_merge.size(); i++) {
    ResolveMerge(labels_to_merge[i], min_label);
  }
}

/*
  Handle merge cases of labels
  Note: if u == v, nothing happens
*/
void PointCloudSegmenter::ResolveMerge(int x, int y) {
  int u = this->rtable.at(x);
  int v = this->rtable.at(y);

  if (u < v) {
    MergeOperation(u, v);
  } else if (u > v) {
    MergeOperation(v, u);
  }
}

/*
  Merge label v into label u and update next, tail, and rtable accordingly.
*/
void PointCloudSegmenter::MergeOperation(int u, int v) {
  int i = v;

  while (i != -1) {
    this->rtable[i] = u;
    i = this->next[i];
  }

  this->next[this->tail[u]] = v;
  this->tail[u] = this->tail[v];
}

/*
  Update labels of points in a run.

  TODO: Make array of run labels to coincide with
        s_queue and e_queue to reduce vector accesses.
*/
void PointCloudSegmenter::SetRunLabel(Scanline& scan_current, int start_index,
                                      int end_index, int set_label) {
  for (int k = start_index; k <= end_index; k++) {
    scan_current.points[k].label = set_label;
  }
}

/*
  Given a point, find its nearest neigbor in the above scanline and return its
  label.

  TODO: Implement smart indexing and kdtree
*/
int PointCloudSegmenter::FindNearestNeighbor(Scanline& scan_current,
                                             Scanline& scan_above,
                                             int start_index, int end_index,
                                             int point_index) {
  double conversion_factor =
      scan_above.points.size() / scan_current.points.size();
  int above_start = floor(start_index * conversion_factor);
  int above_end = floor(end_index * conversion_factor);

  // Determine nearest point to search at
  Vec3 current = scan_current.points[point_index];
  float query[3] = {current.x, current.y, current.z};

  // Set up KNN result set
  size_t result_index;
  float result_dist_squared;
  nanoflann::KNNResultSet<float> results(1);
  results.init(&result_index, &result_dist_squared);

  // Perform search
  scan_above.tree->findNeighbors(results, &query[0],
                                 nanoflann::SearchParams(10));

  if (result_dist_squared < this->th_merge * this->th_merge) {
    return scan_above.tree_point_cloud->get_label(result_index);
  }

  return -1;
}

// Perform label updating by looking up current labels in rtable
// Group points into clusters
std::vector<Vec3> PointCloudSegmenter::ExtractClusters(
    std::vector<Scanline>& scanlines) {
  std::vector<Scanline>::iterator s_it;
  std::vector<Vec3>::iterator v_it;
  std::vector<Vec3> out_points;
  int i = 0;

  for (s_it = scanlines.begin(); s_it != scanlines.end(); s_it++, i++) {
    for (v_it = s_it->points.begin(); v_it != s_it->points.end(); v_it++) {
      if (v_it->label < this->rtable.size() && v_it->label != -3) {
        v_it->label = this->rtable[v_it->label];
        out_points.push_back(*v_it);
      }
    }
  }
  return out_points;
}
