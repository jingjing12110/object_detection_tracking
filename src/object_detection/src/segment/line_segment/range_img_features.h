/**
 * @file range_img_features.h
 * @author Jingjing Jiang (jingjingjiang2017@gmail.com)
 * @brief methods to deal with range image, which refers Moonsmann's thesis
 * paper
 * @version 0.1
 * @date 2019-03-14
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef RANGE_IMG_FEATURES_H_
#define RANGE_IMG_FEATURES_H_

#include "object_d/common_include.h"

// 四邻域...
enum Neighborhood { NBH4, NBH8, NBH24 };

/**
 * @brief
 *
 */
class NormalConfidenceLookup {
 public:
  static NormalConfidenceLookup *get(float resolHAngRAD, float resolVAngRAD,
                                     float stdDevDist, float stdDevHAngRAD,
                                     float stdDevVAngRAD);
  //! returns the expected standard deviation of the normal vector's angle in
  //! randian
  float getStdDevRAD(float dist, float distHoriz, float distVert) const;
  //! returns the expected covariance of the normal vector's coordinates
  //! (assuming yaw/pitch = 0/0 of the center pixel)
  Eigen::MatrixXf getCovar(float dist, float distHoriz, float distVert) const;

 private:
  NormalConfidenceLookup(float resolHAngRAD, float resolVAngRAD,
                         float stdDevDist, float stdDevHAngRAD,
                         float stdDevVAngRAD);
  ~NormalConfidenceLookup();
  bool loadFromFile(std::string filename = "normalConfidence.lookup");
  bool saveToFile(std::string filename = "normalConfidence.lookup");
  void saveToTextFile(std::string filename);
  //!< saves data to a text-file for plotting(values are in degrees!)

  static NormalConfidenceLookup *uniqueInstance;
  // volatile -> might be changed by other thread

  float resolHAngRAD;
  float resolVAngRAD;
  float stdDevDist;
  float stdDevHAngRAD;
  float stdDevVAngRAD;
  float tableDistance;  //!< reference distance at which the lookup table was
                        //!< generated in meter
  float lrange;  //!< lower range of lookup-table distance interval in meter
  float urange;  //!< upper range of lookup-table distance interval in meter
  unsigned int lookupsize;  //!< size of the lookup-table = number of entries
                            //!< per dimension
  float stepsize;           //!< angle increment between entries in the
                            // lookup-table in rad
  float *lookuptableAngle;  //!< the lookup-table, values are in rad
  // As a covariance is always symmetric it would be sufficient to store 6
  // values only
  float *lookuptableCovar;  //!< the lookup-table of the 3D covariance matrix,
                            //!< all 9 values flattened
  // temporary variables that are used frequently: allocated only once
  mutable float fac, dhRel, dvRel;
  mutable unsigned int ih, iv;
  mutable Eigen::Vector3f center, top, left, normal;
  mutable Eigen::MatrixXf covarTop3, covarLeft3, Samples;
  mutable Eigen::Vector3f add;
};

/**
 * @brief method to weight connections of a range image
 * The method weights connections between pixels, by comparing the range
 * values to the neighbors. Based on the uncertainty of the scanner it tries
 * to figure out if the sampling theorem was violated or not. The returned
 * weights are between 0.0 (do not use this connection, also in case of
 * invalid measurements) and 1.0 (valid neighbor).
 * @param in range image reading in meters of sise.
 * @param deriv_h the horizontal derivative of the range image,
 * @param deriv_v the vertical derivative of the range image.
 * @param conn_weight_h the returned weights for horizontal neighbors
 * @param conn_weight_v the returned weights for vertical neighbors,
 * @param MAX_DST_PM don't use connections with a maximum range difference
 * (per distance) of this value
 * @param MAX_DST_FAC don't use connections with a distance-factor
 * higher than this value
 * @param A
 * @param B
 * @param C
 * @param D
 */
void connection_weights(const cv::Mat &in, const cv::Mat &deriv_h,
                        const cv::Mat &deriv_v, cv::Mat &conn_weight_h,
                        cv::Mat &conn_weight_v, float MAX_DST_PM,
                        float MAX_DST_FAC, float A, float B, float C, float D);

/**
 * @brief method to calculate normal vectors, which takes object 
 * borders in account.
 * 
 * @param in range image readings.
 * @param points 3D point coordinates.
 * @param normals the returned normal vectors, (0,0,0) if invalid, otherwise
 * euclidean length = 1. 
 * @param conn_weight_h if pointer valid, horizontal connection weights 
 * will be used. 
 * @param conn_weight_v if pointer valid, vertical connection weights 
 * will be used.
 * @param normalStdDevRAD if pointer valid, an estimate of the standard 
 * deviation of the normal vectors' angle will also be returned.
 * @param normalVariance3D if point valid, the 3x3 covariance matrix of 
 * the normal vectors will be calculated.
 * @param normalConfidence if pointer valid, a confidence estimate will 
 * also be returned, 0=invalid, 0=valid
 * @param tmpVec3D if pointer valid, this temporary buffer will decrease 
 * computation time.
 * @param tmpDbl if pointer valid, this temporary buffer will decrease 
 * computation time. 
 * @param DIST_DIFF  if neighbor is relatively further away than this threshold,
 * use one of the next constants to weight connections. This can be used to 
 * enforce outer edges (because in this case the connection weighting 
 * will be low).
 * @param W_FAC_CREATE connection-weight on creation (see DIST_DIFF). 
 * @param resolHAngRAD angular resolution of image in horizontal direction
 * @param resolVAngRAD resolVAngRAD angular resolution of image in vertical
 * direction 
 * @param stdDevDist standard deviation of distance measurements 
 * @param stdDevHAngRAD standard deviation of the angles of measurements 
 * in horizontal direction standard deviation of the angles of measurements 
 * in vertical direction 
 * @param stdDevVAngRAD 
 * @param nbNConfMedianPasses number of median passes across normal 
 * confidence image
 * @param nConfNeighb type of neighborhood used for median on normal confidence
 * @param nConfMin f true the median is assigned via the minimum operator with
 * the previous normal confidence values.
 * @param verbose 
 */
void moosmann_normal(
    const cv::Mat &in,
    const cv::Mat &points,
    cv::Mat &normals,
    const cv::Mat *conn_weight_h = NULL, 
    const cv::Mat *conn_weight_v = NULL,
    cv::Mat *normalStdDevRAD = NULL,
    cv::Mat *normalVariance3D = NULL,
    cv::Mat *normalConfidence = NULL,
    cv::Mat *tmpVec3D = NULL,
    cv::Mat *tmpDbl = NULL,
    // const LidarImageProjector *projector = NULL,
    float DIST_DIFF = 0.03, float W_FAC_CREATE = 2.0,
    /*float W_FAC_SMOOTH = 2.0,*/ float resolHAngRAD = 0.007,
    float resolVAngRAD = 0.007, float stdDevDist = 0.015,
    float stdDevHAngRAD = 0.0015, float stdDevVAngRAD = 0.00017,
    unsigned int nbNConfMedianPasses = 1, Neighborhood nConfNeighb = NBH4,
    bool nConfMin = false);

/**
 * @brief  method to median-filter of an image
 * 
 * @param data 
 * @param target 
 * @param invalidVal value considered as invalid, thus it is ignored
 */
void median(cv::Mat &data,
            cv::Mat &target, float invalidVal);

/**
 * @brief method to take element-wise maximum
 * 
 * @param data1 
 * @param data2 
 * @param target 
 */
inline void maximum(cv::Mat &data1, cv::Mat &data2,
             cv::Mat &target) {
  cv::Mat_<float>::iterator d1It = data1.begin<float>();
  cv::Mat_<float>::iterator d1End = data1.end<float>();
  cv::Mat_<float>::iterator d2It = data2.begin<float>();
  cv::Mat_<float>::iterator d2End = data2.end<float>();
  cv::Mat_<float>::iterator dtIt = target.begin<float>();
  cv::Mat_<float>::iterator dtEnd = target.end<float>();
  while ((d1It != d1End) && (d2It != d2End) && (dtIt != dtEnd)) {
    *dtIt = std::max(*d1It, *d2It);
    ++d1It;
    ++d2It;
    ++dtIt;
  }
}
/**
 * @brief method to take element-wise minimum
 * 
 * @param data1 
 * @param data2 
 * @param target 
 */
inline void minimum(cv::Mat &data1, cv::Mat &data2,
             cv::Mat &target) {
  cv::Mat_<float>::iterator d1It = data1.begin<float>();
  cv::Mat_<float>::iterator d1End = data1.end<float>();
  cv::Mat_<float>::iterator d2It = data2.begin<float>();
  cv::Mat_<float>::iterator d2End = data2.end<float>();
  cv::Mat_<float>::iterator dtIt = target.begin<float>();
  cv::Mat_<float>::iterator dtEnd = target.end<float>();
  while ((d1It != d1End) && (d2It != d2End) && (dtIt != dtEnd)) {
    *dtIt = std::min(*d1It, *d2It);
    ++d1It;
    ++d2It;
    ++dtIt;
  }
}
////////////////////////////////////////////////////
///////////       Utiliy Function         //////////
////////////////////////////////////////////////////

//! returns 1 for x << 0, and 0 for x >> 0, and 0.5 for x = 0
inline float sigmoidLikeShiftedUp(float x) {
  return 0.5f + ((-0.5f * x) / sqrt(1 + x * x));
}
//! returns 1 for x << thresh, and 0 for x >> thresh, |narrowFac|>1 makes the
//! threshold harder, negative narrowFac inverses result (0 for x<<thresh,
//! 1 for x>>thresh)
inline float sigmoidLikeSoftThresh(float x, float thresh, float narrowFac,
                                   bool normalizeAtZero = false) {
  x = (x - thresh) * narrowFac;
  if (normalizeAtZero)
    // normalize so value f(0)=1
    return sigmoidLikeShiftedUp(x) / sigmoidLikeShiftedUp(0.0f);
  else
    return sigmoidLikeShiftedUp(x);
}

/**
 * @brief tries to estimate whether two pixels/measurements
 * belong to the same surface
 *
 * @param d1 range of one pixel of connection
 * @param d2 range of other pixel of connection
 * @param diff distance-difference between the pixels
 * @param diffL distance differences of the neighboring connections
 * @param diffR distance differences of the neighboring connections
 * @param MAX_DST_PM don't use connections with a maximum distance
 * difference (per distance) of this value
 * @param MAX_DST_FAC don't use connections with a distance-factor
 * higher than this value
 * @param A
 * @param B
 * @param C
 * @param D
 * @return float
 */
inline float connectivity(float d1, float d2, float diff, float diffL,
                          float diffR, float MAX_DST_PM, float MAX_DST_FAC,
                          float A, float B, float C, float D) {
  if ((d1 == 0.0) || (d2 == 0.0) || (std::fabs(d1) < 0.01) ||
      (std::fabs(d2) < 0.01))
    return 0.0;
  // always accept a small connection, so only check further if
  // sufficient large connection (avoids singularity if
  // diffL/diffR are also small)
  if (std::fabs(diff) < D) return 1.0;
  // always reject connection if it is not small
  // but one neighboring connection is (avoids division by zero)
  if ((std::fabs(diffL) < 0.01f) || (std::fabs(diffR) < 0.01f)) return 0.0;

  float dist = std::min(d1, d2);
  // more relative noise if closer -> adjust inclination
  const float nFac = A - B * exp(-dist * C);
  // MAX_DST_PM -> angle of surface plane
  float keep =
      sigmoidLikeSoftThresh(std::fabs(diff) / dist, MAX_DST_PM, 2 / MAX_DST_PM);
  keep = fmin(keep, sigmoidLikeSoftThresh(std::fabs((diff - diffL) / diffL),
                                          MAX_DST_FAC, nFac));
  keep = fmin(keep, sigmoidLikeSoftThresh(std::fabs((diff - diffR) / diffR),
                                          MAX_DST_FAC, nFac));
  return keep;
};

#endif  // RANGE_IMG_FEATURES_H_
