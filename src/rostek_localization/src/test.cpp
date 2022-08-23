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
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"

#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"

#include <thread>  

cartographer::mapping::ProbabilityGrid *probabilityGrid;
cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D *fastCorrelativeScanMatcher2D;
cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D fastCorrelativeScanMatcherOptions2D;

std::unique_ptr<cartographer::mapping::scan_matching::RealTimeCorrelativeScanMatcher2D> real_time_correlative_scan_matcher_;


cartographer::sensor::proto::AdaptiveVoxelFilterOptions adaptiveVoxel;
// cartographer::sensor::proto::AdAdaptiveVoxelFilterOptions voxelOptionts;
bool haveMap = false;
bool first = true;
// bool first_fast = true;
int limit_int, width, height;
float resolution;
std::array<float,3> origin; //0:x, 1:y, 2:yaw



    cartographer::transform::Rigid2d pose_estimate;
    cartographer::transform::Rigid2d fast_pose_estimate;
    cartographer::transform::Rigid2d initial_ceres_pose; 


    //pose test truyenn vao ceres, de resolution  0.05 thi bi loi
     cartographer::transform::Rigid2d test_pose({4.,8.6}, 0.162429);

   auto ceres_parameter_dictionary = cartographer::common::MakeDictionary(R"text(
        return {
          occupied_space_weight = 1.,
          translation_weight = 0.1,
          rotation_weight = 1.5,
          ceres_solver_options = {
            use_nonmonotonic_steps = true,
            max_num_iterations = 50,
            num_threads = 1,
          },
        })text");

cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D ceresScanMatcherOptions2D = cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions2D(ceres_parameter_dictionary.get());


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
    //msg->info.resolution*1000
    probabilityGrid = new cartographer::mapping::ProbabilityGrid(cartographer::mapping::MapLimits(msg->info.resolution*1000,
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
            // probabilityGrid->SetProbability({x,y}, 0.45);
            probabilityGrid->SetProbability({x,y}, 0.1);
        }        
    }
    probabilityGrid->FinishUpdate();
    //creat fast correlative scan matcher 2d
    fastCorrelativeScanMatcher2D = new cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D(*probabilityGrid, fastCorrelativeScanMatcherOptions2D);
    
    haveMap = true;
}

// void initialization(cartographer::sensor::PointCloud pointCloud, const cartographer::transform::Rigid2d expected_pose)
void initialization(cartographer::sensor::PointCloud pointCloud, const cartographer::transform::Rigid2d expected_pose)
{
    // cartographer::transform::Rigid2d pose_estimate;
    // float score = 0.5;
    // float min_score = 0.1;
    // fastCorrelativeScanMatcher2D->Match(expected_pose.cast<double>(), pointCloud, min_score, &score, &pose_estimate);

    // real_time_correlative_scan_matcher_ = new cartographer::mapping::scan_matching::RealTimeCorrelativeScanMatcher2D(options_.real_time_correlative_scan_matcher_options());
    // ceres_scan_matcher_ = new cartographer::mapping::scan_matching::CeresScanMatcher2D(options_.ceres_scan_matcher_options());
    
    initial_ceres_pose = expected_pose; 

 
        float score = 0.5;
        float kMinScore = 0.1;
        fastCorrelativeScanMatcher2D->Match(
                expected_pose.cast<double>(), pointCloud, kMinScore, &score, &fast_pose_estimate);
        std::cout << "Score: " << score << std::endl;
        
}

void ceres_scan_match(cartographer::sensor::PointCloud pointCloud, cartographer::transform::Rigid2d fast_pose_estimate){

    initial_ceres_pose = fast_pose_estimate;

    auto realtime_parameter_dictionary = cartographer::common::MakeDictionary(
    "return {"
    "linear_search_window  = 0.6, "
    "angular_search_window = 0.16, "
    "translation_delta_cost_weight = 0., "
    "rotation_delta_cost_weight = 0., "
    "}");

    real_time_correlative_scan_matcher_ =
    absl::make_unique<cartographer::mapping::scan_matching::RealTimeCorrelativeScanMatcher2D>(
        cartographer::mapping::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(realtime_parameter_dictionary.get()));

    const double score = real_time_correlative_scan_matcher_->Match(
    fast_pose_estimate, pointCloud,
    *probabilityGrid, &initial_ceres_pose);

    std::cout << "Pose initial for ceres: " << initial_ceres_pose.DebugString() << std::endl;



double a,b,c;
    auto creat = [&a,&b,&c](cartographer::sensor::PointCloud pointCloud, 
                    cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D ceresScanMatcherOptions2D,
                    cartographer::mapping::ProbabilityGrid probabilityGrid,
                    cartographer::transform::Rigid2d fast_pose_estimate,
                    cartographer::transform::Rigid2d initial_ceres_pose,
                    cartographer::transform::Rigid2d pose_estimate)
    {
        ceres::Solver::Summary sum;
        cartographer::mapping::scan_matching::CeresScanMatcher2D ceresScanMatcher2D(ceresScanMatcherOptions2D);
        // ceresScanMatcher2D.Match(expected_pose.translation(), initial_ceres_pose, pointCloud, probabilityGrid, &pose_estimate, &sum);
        ceresScanMatcher2D.Match(fast_pose_estimate.translation(), initial_ceres_pose, pointCloud, probabilityGrid, &pose_estimate, &sum);

        a = pose_estimate.translation().x();
        b = pose_estimate.translation().y();
        c = pose_estimate.rotation().angle();
    };
    std::thread run(creat, pointCloud, ceresScanMatcherOptions2D, *probabilityGrid,fast_pose_estimate, initial_ceres_pose, pose_estimate);
    run.join();

    pose_estimate.Translation({a,b});
    pose_estimate.Rotation(c);
    cartographer::transform::Rigid2d inPose({a,b},c);
    pose_estimate = inPose;


    // ROS_ERROR("score: %d",score);
    std::cout << "pose estimate: " << pose_estimate.DebugString() << std::endl;
    std::cout << "expected pose: " << fast_pose_estimate.DebugString() <<std::endl;
}

void laser_clbk(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!haveMap) return;

    float angleIncr = msg->angle_increment;
    float angleMin = msg->angle_min;
    //range data pre process
    cartographer::sensor::PointCloud pointCloud;
    //Range filter 
    for(int i=0 ;  i < msg->ranges.size(); i++)
    {
        if (msg->ranges[i] > 100.0 || msg->ranges[i] < 0.2) continue;
        pointCloud.push_back({Eigen::Vector3f{std::cos(angleMin+angleIncr*i)*msg->ranges[i], std::sin(angleMin+angleIncr*i)*msg->ranges[i], 0.f}});
    }
    //Voxel filter
    pointCloud = cartographer::sensor::VoxelFilter(pointCloud, 0.025);
    //Adaptive Voxel filter
    pointCloud = cartographer::sensor::AdaptiveVoxelFilter(pointCloud, adaptiveVoxel);
    const cartographer::transform::Rigid2d expected_pose(
        {4.,17.}, 0.5
    );
    if (first){
        initialization(pointCloud, expected_pose);
        std::cout << "Fast scan match pose update: " << fast_pose_estimate.DebugString() << std::endl;
        first = false;
    }
    else{
        ceres_scan_match(pointCloud, fast_pose_estimate);
    }

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_fast");
    ros::NodeHandle n;
    fastCorrelativeScanMatcherOptions2D.set_linear_search_window(10.0);
    fastCorrelativeScanMatcherOptions2D.set_angular_search_window(3.14);
    fastCorrelativeScanMatcherOptions2D.set_branch_and_bound_depth(7);
    adaptiveVoxel.set_max_length(0.5);
    adaptiveVoxel.set_min_num_points(200);
    adaptiveVoxel.set_max_range(50.0);
    ros::Subscriber mapSub = n.subscribe("map", 1, &map_clbk);
    ros::Subscriber laserSub = n.subscribe("/scan", 1, &laser_clbk);
    ros::spin();
}