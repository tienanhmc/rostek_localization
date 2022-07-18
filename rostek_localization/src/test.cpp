// ros
#include <ros/ros.h>
//msgs
#include <sensor_msgs/LaserScan.h>
//nav
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
//tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
//geometry
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
// standard
#include <math.h>
#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include "absl/memory/memory.h"

// cartographer
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
cartographer::mapping::ProbabilityGrid *probabilityGrid;
cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D *fastCorrelativeScanMatcher2D;
cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D fastCorrelativeScanMatcherOptions2D;
cartographer::sensor::proto::AdaptiveVoxelFilterOptions adaptiveVoxel;
// cartographer::sensor::proto::AdAdaptiveVoxelFilterOptions voxelOptionts;
bool haveMap = false;
bool first = true;
int limit_int, width, height;
float resolution;
std::array<float,3> origin; //0:x, 1:y, 2:yaw

void map_clbk(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    cartographer::mapping::ValueConversionTables converisionTable;
    ROS_WARN("width: %i,height: %i", msg->info.width,msg->info.height);
    limit_int = ((msg->info.width > msg->info.height)? msg->info.width: msg->info.height);
    // Get map meta data
    resolution = msg->info.resolution;
    width = msg->info.width;
    height = msg->info.height;
    origin[0] = msg->info.origin.position.x;
    origin[1] = msg->info.origin.position.y;
    origin[2] = std::atan2(2*msg->info.origin.orientation.w*msg->info.origin.orientation.z, 1-2*msg->info.origin.orientation.z*msg->info.origin.orientation.z);
    // Creat probabilityGrid map
    probabilityGrid = new cartographer::mapping::ProbabilityGrid(cartographer::mapping::MapLimits(msg->info.resolution,
        Eigen::Vector2d(((double)limit_int)*resolution, ((double)limit_int)*resolution)
        , cartographer::mapping::CellLimits(limit_int, limit_int)),
        &converisionTable);
    for(int i=0; i < msg->data.size(); i++)
    {

        if (msg->data[i] == 100)
        {
            int x = limit_int - i/width - 1;
            int y = limit_int - i%width - 1;

            probabilityGrid->SetProbability({x,y}, 0.9);
        }
        else if (msg->data[i] == 0)
        {
            int x = limit_int - i/width - 1;
            int y = limit_int - i%width - 1;
            probabilityGrid->SetProbability({x,y}, 0.45);
        }        
    }
    probabilityGrid->FinishUpdate();
    //creat fast correlative scan matcher 2d
    fastCorrelativeScanMatcher2D = new cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D(*probabilityGrid, fastCorrelativeScanMatcherOptions2D);
    haveMap = true;
}

void initialization(cartographer::sensor::PointCloud pointClound, cartographer::transform::Rigid2f expected_pose)
{
    cartographer::transform::Rigid2d pose_estimate;
    float score = 0.5;
    float min_score = 0.1;
    fastCorrelativeScanMatcher2D->Match(expected_pose.cast<double>(), pointClound, min_score, &score, &pose_estimate);
    ROS_ERROR("score: %f",score);
    std::cout << "pose estimate: " << pose_estimate.DebugString() << std::endl;
    std::cout << "expected pose: " << expected_pose.DebugString() <<std::endl;
}

void laser_clbk(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!haveMap) return;
    if (!first) return;
    first = false;
    float angleIncr = msg->angle_increment;
    float angleMin = msg->angle_min;
    //range data pre process
    cartographer::sensor::PointCloud pointClound;
    //Range filter 
    for(int i=0 ;  i < msg->ranges.size(); i++)
    {
        if (msg->ranges[i] > 100.0 || msg->ranges[i] < 0.2) continue;
        pointClound.push_back({Eigen::Vector3f{std::cos(angleMin+angleIncr*i)*msg->ranges[i], std::sin(angleMin+angleIncr*i)*msg->ranges[i], 0.f}});
    }
    //Voxel filter
    pointClound = cartographer::sensor::VoxelFilter(pointClound, 0.025);
    //Adaptive Voxel filter
    pointClound = cartographer::sensor::AdaptiveVoxelFilter(pointClound, adaptiveVoxel);
    cartographer::transform::Rigid2f expected_pose(
        {10.,10.}, 0.
    );
    initialization(pointClound, expected_pose);
    first = true;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_fast");
    ros::NodeHandle n;
    fastCorrelativeScanMatcherOptions2D.set_linear_search_window(2.0);
    fastCorrelativeScanMatcherOptions2D.set_angular_search_window(3.14);
    fastCorrelativeScanMatcherOptions2D.set_branch_and_bound_depth(7);
    adaptiveVoxel.set_max_length(0.5);
    adaptiveVoxel.set_min_num_points(200);
    adaptiveVoxel.set_max_range(50.0);
    ros::Subscriber mapSub = n.subscribe("map", 1, &map_clbk);
    ros::Subscriber laserSub = n.subscribe("/scan", 1, &laser_clbk);
    ros::spin();
}