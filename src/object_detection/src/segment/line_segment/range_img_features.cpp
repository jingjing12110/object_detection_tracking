#include "range_img_features.h"

// class
NormalConfidenceLookup *NormalConfidenceLookup::uniqueInstance = 0;

NormalConfidenceLookup *NormalConfidenceLookup::get(float resolHAngRAD,
                                                    float resolVAngRAD,
                                                    float stdDevDist,
                                                    float stdDevHAngRAD,
                                                    float stdDevVAngRAD) {
  if (uniqueInstance == 0) {
    // float check in case multithreaded apps call it in parallel
    uniqueInstance = new NormalConfidenceLookup(
        resolHAngRAD, resolVAngRAD, stdDevDist, stdDevHAngRAD, stdDevVAngRAD);
  }
  return uniqueInstance;
}

NormalConfidenceLookup::NormalConfidenceLookup(float resolHAngRAD_,
                                               float resolVAngRAD_,
                                               float stdDevDist_,
                                               float stdDevHAngRAD_,
                                               float stdDevVAngRAD_)
    : center(3),
      top(3),
      left(3),
      normal(3),
      covarTop3(3, 3),
      covarLeft3(3, 3),
      Samples(3, 12),
      add(3) {
  resolHAngRAD = resolHAngRAD_;
  resolVAngRAD = resolVAngRAD_;
  stdDevDist = stdDevDist_;
  stdDevHAngRAD = stdDevHAngRAD_;
  stdDevVAngRAD = stdDevVAngRAD_;

  tableDistance = 10.0;
  // distance of center point for which the lookup table is calculated
  lrange = 1.0;  // start investigating at 1m from the scanner
  urange = tableDistance + 2 * (tableDistance - lrange);
  // stop investigating at float the lower range
  lookupsize = 80;  // number of entries per dimension
  lookuptableAngle = new float[lookupsize * lookupsize];
  lookuptableCovar = new float[9 * lookupsize * lookupsize];
  stepsize = (urange - lrange) / (float)lookupsize;

  // try to load from default file
  if (loadFromFile()) {
    std::cout << "lookup-table loaded successfully" << std::flush;
  } else {
    // loading was not successful -> generate lookup table and save to default
    // file investigate on (tableDistance+lrange .. tableDistance+urange)
    unsigned int nbMaxMCSamplings = 1000;
    // number of samplings in MC-simulation to generate table
    std::cout << "generating lookup-table at " << tableDistance
              << "m in range [" << lrange << "," << urange << "]..."
              << std::flush;

    //######## calculate MC-estimate of angle variance ########
    // Create a Mersenne twister random number generator that is seeded once
    // with #seconds since 1970
    static boost::mt19937 rng(static_cast<unsigned>(std::time(0)));
    // MUST BE STATIC!!!!!
    // select Gaussian probability distribution
    boost::normal_distribution<float> norm_dist(0, stdDevDist);
    boost::normal_distribution<float> norm_angH(0, stdDevHAngRAD);
    boost::normal_distribution<float> norm_angV(0, stdDevVAngRAD);
    // bind random number generator to distribution, forming a function
    boost::variate_generator<boost::mt19937 &,
                             boost::normal_distribution<float> >
        dist_sampler(rng, norm_dist);
    boost::variate_generator<boost::mt19937 &,
                             boost::normal_distribution<float> >
        angH_sampler(rng, norm_angH);
    boost::variate_generator<boost::mt19937 &,
                             boost::normal_distribution<float> >
        angV_sampler(rng, norm_angV);
    // sample from the distribution
    Eigen::Vector3f center(3), left(3), top(3), leftRel(3), topRel(3),
        normal(3);
    Eigen::Vector3f leftnoise(3), topnoise(3), leftnoiseRel(3), topnoiseRel(3),
        normalnoise(3);
    Eigen::MatrixXf normalSamples(3, nbMaxMCSamplings);
    Eigen::MatrixXf covar(3, 3);
    center(0) = tableDistance;
    center(1) = 0;
    center(2) = 0;
    unsigned int nbTotalSamplings = 0;
    for (ih = 0; ih < lookupsize; ++ih) {
      for (iv = 0; iv < lookupsize; ++iv) {
        float dh = lrange + stepsize * (float)ih;
        float dv = lrange + stepsize * (float)iv;
        top(0) = dv;
        top(1) = 0;
        top(2) = dv * sin(resolHAngRAD);
        left(0) = dh;
        left(1) = dh * sin(resolVAngRAD);
        left(2) = 0;
        topRel = top - center;
        leftRel = left - center;
        normal = topRel.cross(leftRel);
        normal.squaredNorm();
        // assumption: mean = 0.0, which is justified as the expected normal is
        // defined as the calculated one
        float angSqSumRAD = 0.0;
        float lastAngVar = 0.0;
        unsigned int nbSamplings = 0;
        while (nbSamplings < nbMaxMCSamplings) {
          topnoise(0) = top(0) + dist_sampler();
          topnoise(1) = top(0) * sin(angH_sampler());
          topnoise(2) = top(0) * sin(resolVAngRAD + angV_sampler());
          leftnoise(0) = left(0) + dist_sampler();
          leftnoise(1) = left(0) * sin(resolHAngRAD + angH_sampler());
          leftnoise(2) = left(0) * sin(angV_sampler());
          topnoiseRel = topnoise - center;
          leftnoiseRel = leftnoise - center;
          normalnoise = topnoiseRel.cross(leftnoiseRel);
          normalnoise.squaredNorm();

          normalSamples.row(nbSamplings) = normalnoise - normal;
          // DMatrixCol(normalSamples, nbSamplings) = normalnoise - normal;
          float angRAD = acos(normal.dot(normalnoise));
          angSqSumRAD += angRAD * angRAD;
          ++nbSamplings;
          // test every 50 iterations for loop-exit
          if (nbSamplings % 50 == 0) {
            float currAngVar = angSqSumRAD / (float)nbSamplings;
            // variance didn't change too much (<1%) exit loop
            if ((currAngVar - lastAngVar) / currAngVar < 0.01) break;
            lastAngVar = currAngVar;
          }
        }
        Eigen::MatrixXf validNSamples =
            normalSamples.block(0, 0, 3, nbSamplings);
        covar = validNSamples * validNSamples.transpose();
        covar /= (float)(nbSamplings);
        lookuptableAngle[iv * lookupsize + ih] =
            sqrt(angSqSumRAD / (float)nbSamplings);
        for (unsigned int d = 0; d < 9; ++d)
          lookuptableCovar[(iv * lookupsize + ih) * 9 + d] =
              covar(d / 3, d % 3);
        nbTotalSamplings += nbSamplings;
      }
    }
    std::cout << "used " << nbTotalSamplings / (lookupsize * lookupsize)
              << " iterations on average" << std::flush;
    saveToFile();
    // saveToTextFile("normalConfidence.txt");
  }
}

NormalConfidenceLookup::~NormalConfidenceLookup() {
  delete[] lookuptableAngle;
  delete[] lookuptableCovar;
}

bool NormalConfidenceLookup::loadFromFile(std::string filename) {
  try {
    std::ifstream infile(filename.c_str(), std::ios::binary);
    float vald[9];
    unsigned int valui;
    infile.read(reinterpret_cast<char *>(vald), sizeof(float) * 9);
    infile.read(reinterpret_cast<char *>(&valui), sizeof(unsigned int));
    if ((vald[0] != resolHAngRAD) || (vald[1] != resolVAngRAD) ||
        (vald[2] != stdDevDist) || (vald[3] != stdDevHAngRAD) ||
        (vald[4] != stdDevVAngRAD) || (vald[5] != tableDistance) ||
        (vald[6] != lrange) || (vald[7] != urange) || (vald[8] != stepsize) ||
        (valui != lookupsize)) {
      return false;  // parameters of file are not the same as desired
    }
    infile.read(reinterpret_cast<char *>(lookuptableAngle),
                sizeof(float) * lookupsize * lookupsize);
    infile.read(reinterpret_cast<char *>(lookuptableCovar),
                sizeof(float) * lookupsize * lookupsize * 9);
    return true;
  } catch (const std::exception &e) {
    return false;
  }
}

bool NormalConfidenceLookup::saveToFile(std::string filename) {
  try {
    std::ofstream outfile(filename.c_str(), std::ios::out | std::ios::binary);
    outfile.write(reinterpret_cast<char *>(&resolHAngRAD), sizeof(float));
    outfile.write(reinterpret_cast<char *>(&resolVAngRAD), sizeof(float));
    outfile.write(reinterpret_cast<char *>(&stdDevDist), sizeof(float));
    outfile.write(reinterpret_cast<char *>(&stdDevHAngRAD), sizeof(float));
    outfile.write(reinterpret_cast<char *>(&stdDevVAngRAD), sizeof(float));
    outfile.write(reinterpret_cast<char *>(&tableDistance), sizeof(float));
    outfile.write(reinterpret_cast<char *>(&lrange), sizeof(float));
    outfile.write(reinterpret_cast<char *>(&urange), sizeof(float));
    outfile.write(reinterpret_cast<char *>(&stepsize), sizeof(float));
    outfile.write(reinterpret_cast<char *>(&lookupsize), sizeof(unsigned int));
    outfile.write(reinterpret_cast<char *>(lookuptableAngle),
                  sizeof(float) * lookupsize * lookupsize);
    outfile.write(reinterpret_cast<char *>(lookuptableCovar),
                  sizeof(float) * lookupsize * lookupsize * 9);
    return true;
  } catch (const std::exception &e) {
    return false;
  }
}

void NormalConfidenceLookup::saveToTextFile(std::string filename) {
  try {
    std::ofstream outfile(filename.c_str());
    for (ih = 0; ih < lookupsize; ++ih) {
      for (iv = 0; iv < lookupsize; ++iv) {
        outfile << lrange + ih * stepsize << "\t" << lrange + iv * stepsize
                << "\t" << lookuptableAngle[iv * lookupsize + ih] * 180.0 / M_PI
                << std::endl;
      }
      outfile << std::endl;
    }
    for (ih = 0; ih < lookupsize; ++ih) {
      for (iv = 0; iv < lookupsize; ++iv) {
        outfile << lrange + ih * stepsize << "\t" << lrange + iv * stepsize
                << "\t";
        for (unsigned int d = 0; d < 9; ++d) {
          outfile << "\t" << lookuptableAngle[(iv * lookupsize + ih) * 9 + d];
        }
        outfile << std::endl;
      }
      outfile << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }
}

float NormalConfidenceLookup::getStdDevRAD(float dist, float distHoriz,
                                           float distVert) const {
  // 1st) scale values as table was generated for only 1 distance
  fac = tableDistance / dist;
  dhRel = distHoriz * fac;
  dvRel = distVert * fac;
  if ((dhRel < lrange) || (dhRel >= urange) || (dvRel < lrange) ||
      (dvRel >= urange))
    return M_PI / 2.0;
  ih = (unsigned int)((dhRel - lrange) / stepsize);
  iv = (unsigned int)((dvRel - lrange) / stepsize);
  return lookuptableAngle[iv * lookupsize + ih];
}

Eigen::MatrixXf NormalConfidenceLookup::getCovar(float dist, float distHoriz,
                                                 float distVert) const {
  // 1st) scale values as table was generated for only 1 distance
  //  fac = tableDistance/dist;
  //  dhRel = distHoriz * fac;
  //  dvRel = distVert * fac;
  //  if ((dhRel < lrange) || (dhRel >= urange) || (dvRel < lrange) || (dvRel >=
  //  urange))
  //    return DZeroMatrix(3,3);
  //  ih = (unsigned int)((dhRel-lrange) / stepsize);
  //  iv = (unsigned int)((dvRel-lrange) / stepsize);
  //  DMatrix ret(3,3);
  //  for (unsigned int d=0; d<9; ++d)
  //    ret(d/3,d%3) = lookuptableCovar[(iv*lookupsize+ih)*9+d];
  //  return ret;

  // TODO (5): optimize normal-covariance-calculation -> correct lookup-table
  // generation unscented transform
  center(0) = dist;
  center(1) = 0;
  center(2) = 0;
  top(0) = distVert;  // can be modified
  top(1) = 0;
  top(2) = top(0) * sin(resolHAngRAD);
  left(0) = distHoriz;  // can be modified
  left(1) = left(0) * sin(resolVAngRAD);
  left(2) = 0;
  // calculate normal and sample noise
  normal = (top - center).cross((left - center));
  normal.squaredNorm();
  covarTop3 = Eigen::Matrix3f::Zero();
  covarTop3(0, 0) = pow(stdDevDist, 2);  // 0.015^2 = 0,00225
  // (8sin(0.0015))^2 = 0,00014
  covarTop3(1, 1) = pow(top(0) * sin(stdDevHAngRAD), 2);
  // (8sin(0.00017))^2 = 1,8e-6
  covarTop3(2, 2) = pow(top(0) * sin(stdDevVAngRAD), 2);
  covarLeft3 = Eigen::Matrix3f::Zero();
  covarLeft3(0, 0) = pow(stdDevDist, 2);
  covarLeft3(1, 1) = pow(left(0) * sin(stdDevHAngRAD), 2);
  covarLeft3(2, 2) = pow(left(0) * sin(stdDevVAngRAD), 2);
  for (unsigned int i = 0; i < 3; ++i) {
    add = Eigen::Vector3f::Zero();
    add(i) = sqrt(covarTop3(i, i));
    Eigen::MatrixXf sampleP = Samples.row(i);
    sampleP = (top + add - center).cross((left - center));
    sampleP.squaredNorm();

    Eigen::MatrixXf sampleM = Samples.row(i + 3);
    sampleM = (top - add - center).cross((left - center));
    sampleM.squaredNorm();
  }
  for (unsigned int i = 0; i < 3; ++i) {
    add = Eigen::Vector3f::Zero();
    add(i) = sqrt(covarLeft3(i, i));
    Eigen::MatrixXf sampleP = Samples.row(i + 6);
    sampleP = (top - center).cross((left - center + add));
    sampleP.squaredNorm();

    Eigen::MatrixXf sampleM = Samples.row(i + 9);
    sampleM = (top - center).cross((left - center - add));
    sampleM.squaredNorm();
  }
  for (unsigned int i = 0; i < 12; ++i) Samples.row(i) -= normal;
  return (Samples * Samples.transpose()) / 12.0;
}

// function/////////////////////////////////////////////
// a linkage measure that estimates for two neighboring
// points how likely they belong to the same object.
void connection_weights(const cv::Mat &in, const cv::Mat &deriv_h,
                        const cv::Mat &deriv_v, cv::Mat &conn_weight_h,
                        cv::Mat &conn_weight_v, float MAX_DST_PM,
                        float MAX_DST_FAC, float A, float B, float C, float D) {
  int h_size = in.cols;
  int v_size = in.rows;
  // horizontally
  for (int row = 0; row < v_size; ++row) {
    for (int col = 0; col < h_size; ++col) {
      // connection between "col" and "col+1"
      float weight = connectivity(
          in.at<float>(row, col), in.at<float>(row, col + 1),
          deriv_h.at<float>(row, col), deriv_h.at<float>(row, col - 1),
          deriv_h.at<float>(row, col + 1), MAX_DST_PM, MAX_DST_FAC, A, B, C, D);
      conn_weight_h.at<float>(row, col) = weight;
    }
  }
  // vertically
  for (int row = 0; row < v_size; ++row) {
    for (int col = 0; col < h_size; ++col) {
      // connection between "row" and "row+1"
      float weight = connectivity(
          in.at<float>(row, col), in.at<float>(row + 1, col),
          deriv_v.at<float>(row, col), deriv_v.at<float>(row - 1, col),
          deriv_v.at<float>(row + 1, col), MAX_DST_PM, MAX_DST_FAC, A, B, C, D);
      conn_weight_v.at<float>(row, col) = weight;
    }
  }
}

void moosmann_normal(const cv::Mat &in, const cv::Mat &points, cv::Mat &normals,
                     const cv::Mat *conn_weight_h, const cv::Mat *conn_weight_v,
                     cv::Mat *normalStdDevRAD, cv::Mat *normalVariance3D,
                     cv::Mat *normalConfidence, cv::Mat *tmpVec3D,
                     cv::Mat *tmpDbl, float DIST_DIFF, float W_FAC_CREATE,
                     /*float W_FAC_SMOOTH = 2.0,*/
                     float resolHAngRAD, float resolVAngRAD, float stdDevDist,
                     float stdDevHAngRAD, float stdDevVAngRAD,
                     unsigned int nbNConfMedianPasses, Neighborhood nConfNeighb,
                     bool nConfMin) {
  normalVariance3D = NULL;
  int h_size = in.cols;
  int v_size = in.rows;

  NormalConfidenceLookup *ncLookup = NULL;
  // normal stdDev / covariance desired
  if ((normalStdDevRAD) || (normalVariance3D))
    ncLookup = NormalConfidenceLookup::get(
        resolHAngRAD, resolVAngRAD, stdDevDist, stdDevHAngRAD, stdDevVAngRAD); 
  // relative Cartesian coordinates (m,m,m)
  // relative Cartesian coordinates (m,m,m), normalized to ||1||
  // Eigen::VectorXf pi[5]; 
  // Eigen::VectorXf pin[5];
  std::vector<Eigen::Vector3f>pi;
  std::vector<Eigen::Vector3f>pin;
  for (unsigned int i = 0; i < 5; ++i) {
    Eigen::Vector3f temp;
    pi.push_back(temp);
    pin.push_back(temp);
    // pi[i].resize(3);
    // pin[i].resize(3);
  }

  Eigen::Matrix3f rot(3, 3);
  Eigen::Matrix3f rotT(3, 3);

  cv::Mat *ownTmpDbl = NULL;
  if (tmpDbl == NULL) {
    ownTmpDbl = new cv::Mat(v_size, h_size, CV_32FC1);
    tmpDbl = ownTmpDbl;
  }
  tmpVec3D = &normals;    // directly store result in "normals" image
  Eigen::Vector3f cp(3);  // cross product
  // first pass over image to calculate normals and confidence values
  // float wi[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  // weight of connection [0.0..1.0]
  std::vector<float> wi(5, 0.0);
  float di[5];  // distance (m)
  bool bi[5];   // indicator if connection is valid [true/false]
  for (int row = 0; row < v_size; ++row) {
    for (int col = 0; col < h_size; ++col) {
      // get data of point itself
      float d = in.at<float>(row, col);
      cv::Vec3f tmpN_temp = tmpVec3D->at<float>(row, col);
      // Eigen::Vector3f tmpN(tmpN_temp);
      Eigen::Vector3f tmpN(0.0, 0.0, 0.0);
      // tmpN[0] = 0.0;
      // tmpN[1] = 0.0;
      // tmpN[2] = 0.0;  // faster than assigning zero_vector

      bool skip = (d == 0.0);
      // get data of 4 neighbors. store first connection twice, so a simple
      // for-loop can be used for calculation use connection-weights: if
      // connecting-point is more distant, increase weight (edge of object ->
      // let normal point outwards -> good for ICP)
      float sumcpw = 0.0;
      if (!skip) {
        const Eigen::Vector3f &p = points.at<Eigen::Vector3f>(row, col);
        di[0] = in.at<float>(row, col + 1);
        di[1] = in.at<float>(row - 1, col);
        di[2] = in.at<float>(row, col - 1);
        di[3] = in.at<float>(row + 1, col);
        di[4] = di[0];
        bi[0] = (di[0] != 0.0);
        bi[1] = (di[1] != 0.0);
        bi[2] = (di[2] != 0.0);
        bi[3] = (di[3] != 0.0);
        bi[4] = bi[0];
        if (bi[0]) {
          const Eigen::Vector3f &p2 = points.at<Eigen::Vector3f>(row, col + 1);
          pi[0][0] = p2[0] - p[0];
          pi[0][1] = p2[1] - p[1];
          pi[0][2] = p2[2] - p[2];
          float pinorm = pi[0].squaredNorm();
          pin[0][0] = pi[0][0] / pinorm;
          pin[0][1] = pi[0][1] / pinorm;
          pin[0][2] = pi[0][2] / pinorm;
        }
        if (bi[1]) {
          //        pi[1] = points.get(col, row - 1) - p;
          //        pin[1] = pi[1] / norm_2(pi[1]);
          const Eigen::Vector3f &p2 = points.at<Eigen::Vector3f>(row - 1, col);
          pi[1][0] = p2[0] - p[0];
          pi[1][1] = p2[1] - p[1];
          pi[1][2] = p2[2] - p[2];
          float pinorm = pi[1].squaredNorm();
          pin[1][0] = pi[1][0] / pinorm;
          pin[1][1] = pi[1][1] / pinorm;
          pin[1][2] = pi[1][2] / pinorm;
        }
        if (bi[2]) {
          //        pi[2] = points.get(col - 1, row) - p;
          //        pin[2] = pi[2] / norm_2(pi[2]);
          const Eigen::Vector3f &p2 = points.at<Eigen::Vector3f>(row, col - 1);
          pi[2][0] = p2[0] - p[0];
          pi[2][1] = p2[1] - p[1];
          pi[2][2] = p2[2] - p[2];
          float pinorm = pi[2].squaredNorm();
          pin[2][0] = pi[2][0] / pinorm;
          pin[2][1] = pi[2][1] / pinorm;
          pin[2][2] = pi[2][2] / pinorm;
        }
        if (bi[3]) {
          const Eigen::Vector3f &p2 = points.at<Eigen::Vector3f>(row + 1, col);
          pi[3][0] = p2[0] - p[0];
          pi[3][1] = p2[1] - p[1];
          pi[3][2] = p2[2] - p[2];
          float pinorm = pi[3].squaredNorm();
          pin[3][0] = pi[3][0] / pinorm;
          pin[3][1] = pi[3][1] / pinorm;
          pin[3][2] = pi[3][2] / pinorm;
        }
        if (bi[4]) {
          //        pi[4] = pi[0]; // fucking slow
          //        pin[4] = pin[0]; // fucking slow
          pi[4][0] = pi[0][0];
          pi[4][1] = pi[0][1];
          pi[4][2] = pi[0][2];
          pin[4][0] = pin[0][0];
          pin[4][1] = pin[0][1];
          pin[4][2] = pin[0][2];
        }
        if (conn_weight_h && conn_weight_v) {
          // wi = {0.0, 0.0, 0.0, 0.0, 0.0};
          if (bi[0])  // to the right
            wi[0] = (di[0] > d + DIST_DIFF)
                        ? W_FAC_CREATE
                        : conn_weight_h->at<float>(row, col);
          if (bi[1])  // to the top
            wi[1] = (di[1] > d + DIST_DIFF)
                        ? W_FAC_CREATE
                        : conn_weight_v->at<float>(row - 1, col);
          if (bi[2])  // to the left
            wi[2] = (di[2] > d + DIST_DIFF)
                        ? W_FAC_CREATE
                        : conn_weight_h->at<float>(row, col - 1);
          if (bi[3])  // to the bottom
            wi[3] = (di[3] > d + DIST_DIFF)
                        ? W_FAC_CREATE
                        : conn_weight_v->at<float>(row, col);
          wi[4] = wi[0];
        } else {
          wi = {1.0, 1.0, 1.0, 1.0, 1.0};
        }
        sumcpw = wi[0] + wi[1] + wi[2] + wi[3];
      }  // end: gather neighbor-data

      skip = skip || (sumcpw < 0.01);
      // calculate 4 cross products and sum them up
      float nstdDevRAD = 0.0001;
      // uncertainty of normal vector in terms of angle
      Eigen::Matrix3f nCovar = Eigen::Matrix3f::Zero();
      float cw = 0.0;    // cumulative weight
      float mcpw = 0.0;  // max weight -> used as normal confidence
      if (!skip) {
        //      cout << endl << row << "\t" << vsize << "\t" << col << "\t" <<
        //      hsize << "\t" << DBL_MAX << "\t" << "\t" << "\t" << flush;
        for (unsigned int i = 0; i < 4; ++i) {
          // calculates right-top, top-left, left-bottom and
          // bottom-left cross products
          //        cout << di[i] << "\t" << bi[i] << "\t" << pi[i] << di[i+1]
          //        << "\t" << bi[i+1] << "\t" << pi[i+1] << flush;
          if (bi[i] && bi[i + 1]) {
            float cpw = wi[i] * wi[i + 1];  // weight by connection weights
            mcpw = std::max(mcpw, cpw);
            cw += cpw;
            cp = pin[i].cross(pin[i + 1]);
            // |cp| = |pi||pi+1|sin(ang) =>  normal length ~ sin(ang)
            tmpN += cpw * cp;  // normal points towards scanner
            unsigned int ih = i + (i % 2);
            // horizontal connection index (0->0, 1/2->2, 3->4)
            unsigned int iv = i + ((i + 1) % 2);
            // vertical connection index (0/1->1, 2/3->3)
            if (normalStdDevRAD)
              nstdDevRAD += cpw * ncLookup->getStdDevRAD(d, di[ih], di[iv]);
            if (normalVariance3D)
              nCovar += cpw * ncLookup->getCovar(d, di[ih], di[iv]);
          }
        }                             // end: loop over 4 neighbors
      }                               // end: calculate cross products
      float nl = tmpN.squaredNorm();  // norm_2(tmpN);
      // assert(!isnan(tmpN(0)) && "tmpVec3D is NAN");
      // assert(!isnan(tmpN(1)) && "tmpVec3D is NAN");
      // assert(!isnan(tmpN(2)) && "tmpVec3D is NAN");

      // check if invalid normal
      skip = skip || (nl < 0.01f);
      if (!skip) {
        // set normal vector based on sum of 4 cross-product-normal-vectors
        tmpN /= nl;  // normalize normal to length 1
        nstdDevRAD /= cw;

        // calculate standard deviation of angle of normal vector
        if (normalStdDevRAD) normalStdDevRAD->at<float>(row, col) = nstdDevRAD;
        if (normalVariance3D) {
          nCovar /= cw;
          // projector->getRotMatrix(col, row, rot);
          rotT = rot.transpose();
          nCovar = rot * nCovar;
          nCovar = nCovar * rotT;
          normalVariance3D->at<Eigen::Matrix3f>(row, col) = nCovar;
          // normalVariance3D->set(col, row, nCovar);
        }

        // calculate confidence in normal vector estimation
        if (normalConfidence) {
          // maximum confidence of one of the 4 cross products
          float confidence = std::min(1.0f, mcpw);
          // probability that the plane-assumption holds horizontally
          float phpl = 1.0;
          // probability that the plane-assumption holds vertically
          float pvpl = 1.0;
          // TODO (5): optimize: asin and exp need a lot of time to calculate
          // (10% and 11% of this function respectively)
          if (bi[0])  // to the right
            phpl *=
                exp(-0.5 * pow(asin(abs((pin[0].dot(tmpN)))) / nstdDevRAD, 2));
          if (bi[2])  // to the right
            phpl *=
                exp(-0.5 * pow(asin(abs((pin[2].dot(tmpN)))) / nstdDevRAD, 2));
          if (bi[1])  // to the right
            pvpl *=
                exp(-0.5 * pow(asin(abs((pin[1].dot(tmpN)))) / nstdDevRAD, 2));
          if (bi[3])  // to the right
            pvpl *=
                exp(-0.5 * pow(asin(abs((pin[3].dot(tmpN)))) / nstdDevRAD, 2));
          confidence = std::min(confidence, std::max(phpl, pvpl));
          normalConfidence->at<float>(row, col) = confidence;
        }  // end: calculate normal confidence
      }

      if (skip) {
        tmpN[0] = 0.0;
        tmpN[1] = 0.0;
        tmpN[2] = 0.0;  // faster than assigning zero
        if (normalStdDevRAD) normalStdDevRAD->at<float>(row, col) = M_PI / 2;
        if (normalVariance3D)
          normalVariance3D->at<Eigen::Matrix3f>(row, col) =
              Eigen::Matrix3f::Zero();
        if (normalConfidence) normalConfidence->at<float>(row, col) = 0.0;
      }
    }  // end: column
  }    // end: row
  // smooth normal confidence
  if (normalConfidence) {
    for (unsigned int i = 0; i < nbNConfMedianPasses; i++) {
      // stores result in *tmpDbl
      median(*normalConfidence, *tmpDbl, 0.0);
      if (nConfMin)
        minimum(*normalConfidence, *tmpDbl, *normalConfidence);
      else
        // sufficient for type float
        normalConfidence=tmpDbl;  
    }
  }

  delete ownTmpDbl;  // temporary lidar image, null if one was provided
  //  delete ownTmpVec3D; // temporary lidar image, null if one was provided
}

void median(cv::Mat &data,
            cv::Mat &target, float invalidVal) {
  // implementation-idea: simply iterate over pixels making use of
  // iterator-functions to get 4 neighbors
  // cv::Mat::const_iterator dIt = data.begin();
  cv::Mat_<float>::iterator dIt = data.begin<float>();
  // cv::MatIterator_<float> dIt= data.begin<float>();
  cv::Mat_<float>::iterator dEnd = data.end<float>();
  cv::Mat_<float>::iterator tbIt = target.begin<float>();
  cv::Mat_<float>::iterator tbEnd = target.end<float>();
  std::vector<float> elements;
  elements.reserve(5);
  while ((dIt != dEnd) && (tbIt != tbEnd)) {
    elements.clear();
    if (*dIt == invalidVal) {
      *tbIt = invalidVal;
    } else {
      elements.push_back(*dIt);
      float val;
      val = *dIt;
      if (val != invalidVal) elements.push_back(val);
      val = *dIt;
      if (val != invalidVal) elements.push_back(val);
      val = *dIt;
      if (val != invalidVal) elements.push_back(val);
      val = *dIt;
      if (val != invalidVal) elements.push_back(val);
      std::sort(elements.begin(), elements.end());
      float median = elements[elements.size() / 2];  
      // there is at least 1 element
      *tbIt = median;
    }
    ++dIt;
    ++tbIt;
  }
}

