#include "ground_filter.h"

using namespace std;

//pcl::PointCloud<pcl::PointXYZI>::Ptr removeground_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
//static ros::Publisher filtered_points_pub;

void GroundFilter::init_conditional_removal(pcl::ConditionalRemoval<pcl::PointXYZI>& condrem)
{
    pcl::ConditionOr<pcl::PointXYZI>::Ptr car_range_cond(new pcl::ConditionOr<pcl::PointXYZI>);
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr x_cond_L(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,-1.5));
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr x_cond_G(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,2.8));
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr y_cond_L(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,-0.7));
	pcl::FieldComparison<pcl::PointXYZI>::ConstPtr y_cond_G(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,0.7));
	car_range_cond->addComparison(x_cond_L);
	car_range_cond->addComparison(x_cond_G);
	car_range_cond->addComparison(y_cond_L);
	car_range_cond->addComparison(y_cond_G);
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr car_full_range_cond(new pcl::ConditionAnd<pcl::PointXYZI>);
    car_full_range_cond->addCondition(car_range_cond);
    // car_full_range_cond->addCondition(rearview_range_cond);

    condrem.setCondition(car_full_range_cond);
	condrem.setKeepOrganized(false);
}

void GroundFilter::ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_, pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc, pcl::PointCloud<pcl::PointXYZI>::Ptr not_ground_pc)
{
    ground_pc->clear();
    not_ground_pc->clear();
    // cout << "ground points size: " << ground_pc->points.size() << endl;
    // cout << "not ground points size: " << not_ground_pc->points.size() << endl;

    ////////////////////////车体点云滤波/////////////////////////
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
   
    pcl::IndicesPtr indices_sky(new std::vector<int>());
    pcl::PassThrough<pcl::PointXYZI> ptfilter(false);
    ptfilter.setInputCloud(cloud_in_);
    ptfilter.setFilterFieldName("z");
    ptfilter.setFilterLimits(-2.0, 3.0);
    ptfilter.filter (*indices_sky);

    pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
    init_conditional_removal(condrem);  

    condrem.setInputCloud(cloud_in_);
    condrem.setIndices(indices_sky);
    condrem.filter(*temp); 

    cout << "temp size: " << temp->points.size() << endl;

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (temp);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1000.0, 0.5);//
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_in);
    pass.setFilterLimitsNegative (true);
    pass.filter (*not_ground_pc);

    cout << "cloud_in size: " << cloud_in->points.size() << endl;
    cout << "not ground pc size: " << not_ground_pc->points.size() << endl;

    ////////////////////////车体点云滤波/////////////////////////

    // grid
    float grid_size_c = 0.2; //0.45 0.4
    float grid_size_r = 1.0; // 1.2
    int column = 60 / grid_size_c;  //y:120
    int row = 120 / grid_size_r;     //x:160
    GroundFilter grid[column][row];
    int j = 0;
    int i = 0;

    for (int m = 0; m < cloud_in->size(); m++)
    {
        for( i = -column/2; i < column/2; i++)
        {
            if ((cloud_in->points[m].y > i * grid_size_c) && (cloud_in->points[m].y < i * grid_size_c + grid_size_c)) //如果y在某范围内
            {
                for(j = -row/2; j < row/2; j++)
                {
                    if((cloud_in->points[m].x > j * grid_size_r) && (cloud_in->points[m].x < j * grid_size_r + grid_size_r)) //如果x在某范围内
                    {    
                         grid[i+column/2][j+row/2].grid_cloud->points.push_back(cloud_in->points[m]); //等价于pcl::ExtractIndices
                    }               
                }           
            }       
        }   
    }//for


    int cnt = 0;
    for(int ki = 0; ki < column; ki++)
    {
        for(int kj = 0; kj < row; kj++)
        {
            // 计算栅格几何特征
            if (grid[ki][kj].grid_cloud->size() == 0) {continue;}
            else
            {
                if(grid[ki][kj].grid_cloud->size() == 1)
                {
                    // grid[ki][kj].h_mean = grid[ki][kj].grid_cloud->points[0].z; //均值
                    grid[ki][kj].h_min = grid[ki][kj].grid_cloud->points[0].z;
                    grid[ki][kj].h_difference = 0.0; //?
                    cnt++;
                }
                else
                {
                    // Eigen::Vector4f centroid;
                    // pcl::compute3DCentroid(*grid[ki][kj].grid_cloud, centroid);
                    // grid[ki][kj].h_mean = centroid[2]; //均值

                    pcl::PointXYZI min; //用于存放三个轴的最小值
                    pcl::PointXYZI max; //用于存放三个轴的最大值
                    pcl::getMinMax3D(*grid[ki][kj].grid_cloud, min, max);
                    grid[ki][kj].h_min = min.z;
                    grid[ki][kj].h_difference = max.z - min.z;
// 
                }
                // std::cout<<"======"<<std::endl; ->points.push_back(scan_seg->points[r]);
                // if(grid[ki][kj].h_difference < 0.1 && grid[ki][kj].h_mean < 0.3 )
                if(grid[ki][kj].h_difference < 0.25)
                {
                    ground_pc->insert(ground_pc->end(), grid[ki][kj].grid_cloud->begin(), grid[ki][kj].grid_cloud->end());
                }
                else
                {
                    if(grid[ki][kj].h_min < 0.2)
                    {
                        for(int gi=0; gi<grid[ki][kj].grid_cloud->size(); gi++)
                        {
                            if(grid[ki][kj].grid_cloud->points[gi].z < 0.2){
                                ground_pc->insert(ground_pc->end(), grid[ki][kj].grid_cloud->points[gi]);
                            }
                            else{
                                not_ground_pc->insert(not_ground_pc->end(), grid[ki][kj].grid_cloud->points[gi]);
                            }
                        }

                    }
                    else
                    {
                        not_ground_pc->insert(not_ground_pc->end(), grid[ki][kj].grid_cloud->begin(), 
                                                grid[ki][kj].grid_cloud->end());
                    }

                }
                
            } 
        }
    }
    // pcl::PassThrough<pcl::PointXYZI> pass_a;
    // pass_a.setInputCloud (not_ground_pc);
    // pass_a.setFilterFieldName ("x");
    // pass_a.setFilterLimits (-2.2, 1.8);//
    // pass_a.setFilterLimitsNegative (false);
    // pass_a.filter (*n_ground_pc);
}
