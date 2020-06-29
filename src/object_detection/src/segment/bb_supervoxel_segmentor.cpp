/*
    @author     Jay (jaypanda16@gmail.com)
    @file       bb_supervoxel_segmentor.cpp
    @date       2018/02/03
    @version    0.1

    @brief      3D person identification on basketball court using supervoxel segmentation followed
                by k-means.

    @section    DESCRIPTION
            - Uses Voxel Cloud Connectivity Segmentation (VCCS) which generates volumetric
                over-segmentations of 3D point cloud data, known as supervoxels.
            - The above step generates individual 3D point clouds for each person in scene.
            - Now, the color histograms for each person point cloud as features in a k-means
                clustering into 3 clusters corresponding to groups of people belonging to teamA,
                teamB and Referees.
*/
#include "bb_supervoxel_segmentor.hpp"

BBSupervoxelSegmentor::BBSupervoxelSegmentor(char *point_cloud_data_file) {
    cloud = boost::shared_ptr <PointCloudT> (new PointCloudT ());

    pcl::console::print_highlight("Loading point cloud...\n");
    if (!LoadPointCloud(point_cloud_data_file)) {
        pcl::console::print_error("Error loading cloud file!\n");
    }
    if (DEBUG )
        pcl::console::print_highlight("%d cloud size loaded", cloud -> size());
}

void BBSupervoxelSegmentor::SetParameters(float voxel_resolution, float seed_resolution, float color_importance,
                            float spatial_importance, float normal_importance, int rgb_histbins,
                            bool DEBUG, bool VISUALIZE) {
    pcl::console::print_highlight("Setting config parameters...\n");
    voxel_resolution = voxel_resolution;
    seed_resolution = seed_resolution;
    color_importance = color_importance;
    spatial_importance = spatial_importance;
    normal_importance = normal_importance;
    rgb_histbins = rgb_histbins;
    DEBUG = DEBUG;
    VISUALIZE = VISUALIZE;
}

bool BBSupervoxelSegmentor::LoadPointCloud(char* point_cloud_data_file) {
    ifstream fp(point_cloud_data_file);
    PointT currPoint;
    uint32_t r, g, b, a = 255;

    while (fp >> currPoint.x) {
        try {
            fp >> currPoint.z >> currPoint.y;
            fp >> r >> g >> b;
        } catch (int e) {
            pcl::console::print_error("Error %d occurred\n", e);
            return false;
        }
        uint32_t rgba = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b)); //  | static_cast<uint32_t>(a)); Tried RGBA not working
        currPoint.rgba = *reinterpret_cast<uint32_t*>(&rgba);
        cloud -> points.push_back(currPoint);
    }
    return true;
}

void BBSupervoxelSegmentor::SupervoxelClustering(std::vector<PointCloudT::Ptr>& final_objclouds,
                                                 std::vector<PointT>& final_centroids) {
    // Initialize SupervoxelClustering object with voxel_resolution and seed_resolution
    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform(false);
    super.setInputCloud(cloud);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);

    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

    pcl::console::print_highlight("Extracting supervoxels...\n");
    super.extract(supervoxel_clusters);
    if (DEBUG)
        pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

    pcl::console::print_highlight ("Getting supervoxel adjacency...\n");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    // To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel
    // adjacency multimap.
    std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
    std::map<uint32_t, bool> processed_clusters;

    for (; label_itr != supervoxel_adjacency.end();) {
        // Gets the label and the corresopnding supervoxel
        uint32_t supervoxel_label = label_itr->first;
        if( processed_clusters.find(supervoxel_label) != processed_clusters.end()) {
            label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
            continue;
        }
        processed_clusters[supervoxel_label] = true;
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);
        if (DEBUG)
            pcl::console::print_info("Supervoxel cluster: %d\n", supervoxel_label);

        // Iterates through the adjacent supervoxels and makes a point cloud of them.
        PointCloudT adjacent_supervoxel_centers;
        PointCloudT::Ptr curr_obj = supervoxel->voxels_;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
        for (; adjacent_itr!=supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr) {
            processed_clusters[adjacent_itr->second] = true;
            if (DEBUG)
                pcl::console::print_info("merging with cluster: %d\n", adjacent_itr->second);
            pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
            adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
            PointCloudT::Ptr tmp_cloud = neighbor_supervoxel->voxels_;

            curr_obj->insert(curr_obj->end(),tmp_cloud->begin(), tmp_cloud->end());
        }

        // Add this adjacent_supervoxel_centers point cloud - considered as a single object in the scene.
        final_objclouds.push_back(curr_obj);
        final_centroids.push_back(supervoxel->centroid_);

        // Move iterator forward to next label.
        label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
    }

    // Add individual object point clouds to final_objclouds, (those left out in the adjacency graph).
    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >::iterator cluster_itr = supervoxel_clusters.begin();
    for ( ; cluster_itr != supervoxel_clusters.end() ; ++cluster_itr) {
        if (processed_clusters.find(cluster_itr->first) == processed_clusters.end()) {
            // Naive thresholding based on size of point cloud to eliminate small/noisy objects/points.
            // I found this threshold limit was sufficient to eliminate the basketball net and another noisy
            // point cloud.
            if (cluster_itr->second->voxels_->size() < 25)
                continue;
            // Add this point cloud - considered as a single object in the scene.
            final_objclouds.push_back(cluster_itr->second->voxels_);
            final_centroids.push_back(cluster_itr->second->centroid_);
        }
    }
}

void BBSupervoxelSegmentor::ColorHistClustering(std::vector<PointCloudT::Ptr> final_objclouds,
                                                    std::vector<int>& labelvecs, int hist_size) {
    float range[] = { 0, 255 } ;
    const float* hist_range = { range };
    bool uniform = true;
    bool accumulate = false;

    cv::Mat hist_features = cv::Mat::zeros(final_objclouds.size(), hist_size*3, CV_32FC1);
    for (int c = 0 ; c < final_objclouds.size() ; c++) {
        PointCloudT::Ptr cloud = final_objclouds[c];
        cv::Mat b_hist, g_hist, r_hist;

        cv::Mat rvals = cv::Mat::zeros(1, cloud->size(), CV_32FC1);
        cv::Mat gvals = cv::Mat::zeros(1, cloud->size(), CV_32FC1);
        cv::Mat bvals = cv::Mat::zeros(1, cloud->size(), CV_32FC1);

        int i = 0;
        // Iterate through points in the point cloud to get 1-D Mat arrays for R, G, B values
        for (PointCloudT::iterator pitr = cloud->begin()  ; pitr != cloud->end() ; ++pitr, ++i) {
            uint32_t rgbD = pitr->rgba;
            uint16_t rD = (rgbD >> 16) & 0x0000ff;
            uint16_t gD = (rgbD >> 8) & 0x0000ff;
            uint16_t bD = (rgbD) & 0x0000ff;
            rvals.at<float>(0,i) = (float)rD;
            gvals.at<float>(0,i) = (float)gD;
            bvals.at<float>(0,i) = (float)bD;
        }

        // Compute individual R,G,B histograms using OpenCV calcHist.
        cv::calcHist(&rvals, 1, 0, cv::Mat(), r_hist, 1, &hist_size, &hist_range, uniform, accumulate);
        cv::calcHist(&gvals, 1, 0, cv::Mat(), g_hist, 1, &hist_size, &hist_range, uniform, accumulate);
        cv::calcHist(&bvals, 1, 0, cv::Mat(), b_hist, 1, &hist_size, &hist_range, uniform, accumulate);

        // Normalize the result to [ 0, 1].
        cv::normalize(b_hist, b_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        cv::normalize(g_hist, g_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        cv::normalize(r_hist, r_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

        // Concatenate the R-G-B histograms to form a single feature vector for each point cloud.
        // Update the corresponding row in hist_features Mat object for the same.
        size_t offset = 0;
        r_hist = r_hist.t();
        r_hist.copyTo(hist_features.row(c).colRange(offset, offset+hist_size));
        offset += hist_size;
        g_hist = g_hist.t();
        g_hist.copyTo(hist_features.row(c).colRange(offset, offset+hist_size));
        offset += hist_size;
        b_hist = b_hist.t();
        b_hist.row(0).copyTo(hist_features.row(c).colRange(offset, offset+hist_size));
    }

    // Cluster with k-means for 3 groups (teamA, teamB and Referees).
    cv::Mat labels, centers;
    int attempts = 5;
    cv::kmeans (hist_features, 3, labels,
        cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001),
        attempts, cv::KMEANS_PP_CENTERS, centers);
    for (int i = 0 ; i < labels.rows ; i++ ) {
        labelvecs.push_back (labels.at<int>(i,0));
        if (DEBUG)
            pcl::console::print_info ("%d\n", labels.at<int>(i,0));
    }
}

bool BBSupervoxelSegmentor::ProcessPointCloud() {
    if (cloud -> size() == 0) {
        // Point cloud data not loaded.
        return false;
    }
    pcl::console::print_highlight("Processing starts...\n");

    // Person point clouds and their centroid points as lists: [PointCloudT] & [PointXYZRGBA]
    std::vector<PointCloudT::Ptr> final_objclouds;
    std::vector<PointT> final_centroids;
    obj_centroids = boost::shared_ptr <PointCloudT> (new PointCloudT());
    viewer =  boost::shared_ptr <pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    SupervoxelClustering(final_objclouds, final_centroids);

    // Cluster final object clouds based on their rgb histograms.
    std::vector<int> labels;
    std::map<int,int> labels_count;
    ColorHistClustering(final_objclouds,labels, rgb_histbins);

    // Output result and visualize the same.
    std::vector<std::string> output(3,"[");
    // Defines 3 unique colors to identify 3 types of persons on the point cloud.
    uint32_t uniq_colors[] = {255 << 16 | 0 << 8 | 0,
                           0 << 16 | 255 << 8 | 0,
                           0 << 16 | 0 << 8 | 255};

    // Visualize and process the final obj clouds for RGB based person classification.
    for (int i = 0 ; i < final_objclouds.size() ; i++) {
        uint32_t rgbD = (uint32_t)final_centroids[i].rgba;
        uint16_t rD = (rgbD >> 16) & 0x0000ff;
        uint16_t gD = (rgbD >> 8) & 0x0000ff;
        uint16_t bD = (rgbD) & 0x0000ff;
        if (DEBUG) {
            pcl::console::print_info("RGB: (%d)", (int)final_centroids[i].rgba);
            pcl::console::print_info("RGB: (%d, %d, %d)", rD, gD, bD);
            pcl::console::print_highlight("Person %d at X,Y = (%.2f, %.2f)\n",
                                    i+1,
                                    final_centroids[i].x,
                                    final_centroids[i].y);
        }

        // Color the object centroid with unique color for teamA/teamB/referree.
        PointT centroid = final_centroids[i];
        centroid.rgba = uniq_colors[labels[i]];
        obj_centroids -> push_back(centroid);

        // Output for corresponding teamA/teamB/referee and track label occurrence.
        std::stringstream curr_output;
        curr_output << std::setprecision(2);
        curr_output << " (" << final_centroids[i].x << "," << final_centroids[i].y << "), ";
        output[labels[i]] += curr_output.str();
        if (labels_count.find(labels[i]) == labels_count.end()) {
            labels_count[labels[i]] = 1;
        } else {
            labels_count[labels[i]] += 1;
        }

        // Visualize  RGB colored point cloud for each identified object i.e. individual cluster.
        if (VISUALIZE) {
            std::stringstream obj_id;
            obj_id << "P_" << i+1 ;
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgba(final_objclouds[i]);
            viewer->setBackgroundColor(255,255,255);
            viewer->addPointCloud<PointT> (final_objclouds[i], rgba, obj_id.str());
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, obj_id.str());
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, obj_id.str());
        }
    }

    // Display final output as required:
    // TeamA: [[x1,y1],[x2,y2]...]
    // TeamB: [[x1,y1],[x2,y2]...]
    // Referees: [[x1,y1],[x2,y2]...]
    int teamA_flag = true;
    for (std::map<int,int>::iterator l_itr = labels_count.begin() ; l_itr != labels_count.end() ; ++l_itr) {
        std::string val = output[l_itr->first];
        val[val.size() - 2] = ' ';
        val[val.size() - 1] = ']';
        if (l_itr->second < 4) {
            pcl::console::print_highlight("Referees: %s\n", val.c_str());
        } else if (teamA_flag) {
            pcl::console::print_highlight("TeamA: %s\n", val.c_str());
            teamA_flag = false;
        } else {
            pcl::console::print_highlight("TeamB: %s\n", val.c_str());
        }
    }
    if (VISUALIZE) {
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> uniqcol(obj_centroids);
        viewer->addPointCloud<PointT> (obj_centroids, uniqcol, "people");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, "people");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "people");
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
        }
    }
    return true;
}
