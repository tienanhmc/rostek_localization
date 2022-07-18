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


// cartographer
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
// ceress
// #include "ceres/solver.h"

#define quaternion_to_euler(qz, qw) (std::atan2((2 * qw * qz), (1 - 2 * qz * qz)))
class RobotLocalization {
public:
    RobotLocalization(ros::NodeHandle &nh);
    virtual ~RobotLocalization();
private:
    //common
    ros::NodeHandle nh_;
    bool haveMap;
    bool isInitPose;
    std::string lidarFrame;
    geometry_msgs::PoseStamped lastPose;
    std::string mapTopic, LaserTopic;
    float lastTimeUpdateLidar;
    // ros pub/sub
    // TODO: add the publishers and subscribers you need

    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber initial_pose_;

    //tf

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;


    //Occupancy Grid map implement

    nav_msgs::MapMetaData mapMetaData;

    //Robot parameters

    // random generator, use this
    cartographer::sensor::PointCloud *pointClound;


    // callbacks
    void map_clbk(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void scan_clbk(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odom_clbk(const nav_msgs::Odometry::ConstPtr& msg);
    void initial_pos_clbk(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void tf_publisher();

    //cartographer
    void real_time_scan_match(const cartographer::transform::Rigid2d &pose_prediction,
        const cartographer::sensor::PointCloud cloud);
    cartographer::transform::Rigid2d fast_correlative_matcher(std::vector<cartographer::transform::Rigid2f> points, float &score);
    cartographer::transform::Rigid2d fast_correlative_matcher(cartographer::transform::Rigid2f point, float &score);
    // ProbabilityGrid *probability_grid;
    std::array<double,5> score;
    cartographer::mapping::ProbabilityGrid *probabilityGrid;
    cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D *fastCorrelativeScanMatcher2D;
    cartographer::mapping::scan_matching::CeresScanMatcher2D *ceressScanMatcher;
    cartographer::mapping::scan_matching::RealTimeCorrelativeScanMatcher2D *realTimeCorrelativeScanMatcher;
};

RobotLocalization::RobotLocalization(ros::NodeHandle &nh):
nh_(nh)
{
    haveMap = false;
    map_sub_ = nh_.subscribe("/map", 1, &RobotLocalization::map_clbk, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &RobotLocalization::odom_clbk, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &RobotLocalization::scan_clbk, this);
    boost::shared_ptr<nav_msgs::OccupancyGrid const> sharedMap;
    while (sharedMap != nullptr)
    {
        sharedMap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", nh_);
        ROS_WARN("Wait for map server");
    }
    tf::StampedTransform transform;
    while(ros::ok())
    {
        try
        {
            listener.lookupTransform("/odom", "laser", ros::Time(1.0), transform);
            break;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            ROS_WARN("Wait for tf from odom to baselink");
            continue;
        }
    }
}

void RobotLocalization::map_clbk(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    mapMetaData = msg->info;
    cartographer::mapping::ValueConversionTables converisionTable;
    //Create ProbabilityGrid map from OccupancyGrid
    probabilityGrid = new cartographer::mapping::ProbabilityGrid(cartographer::mapping::MapLimits(msg->info.resolution,
        Eigen::Vector2d(((double)msg->info.resolution)*msg->info.width, ((double)msg->info.resolution)*msg->info.height)
        , cartographer::mapping::CellLimits(msg->info.width, msg->info.height)),
        &converisionTable);
    for(size_t i; i < msg->data.size(); i++)
    {
        if (msg->data[i] == 100) probabilityGrid->SetProbability(Eigen::Array2i(i%msg->info.width, i/msg->info.width), 1.0);
        if (msg->data[i] == 0) probabilityGrid->SetProbability(Eigen::Array2i(i%msg->info.width, i/msg->info.width), 0.0);
    }
    haveMap = true;
}

void RobotLocalization::scan_clbk(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!haveMap) return;
    lidarFrame = msg->header.frame_id;
    float angleIncr = msg->angle_increment;
    float angleMin = msg->angle_min;
    //create pointClound from LaserMsgs
    tf::StampedTransform lidarPose;
    try
    {
        listener.lookupTransform("map", lidarFrame, ros::Time(0), lidarPose);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }
    pointClound = new cartographer::sensor::PointCloud();
    for(size_t i ;  i < msg->ranges.size(); i++)
    {
        pointClound->push_back({Eigen::Vector3f{std::cos(angleMin+angleIncr*i)*msg->ranges[i], std::sin(angleMin+angleIncr*i)*msg->ranges[i], 0.f}});
    }
    lastTimeUpdateLidar = msg->header.stamp.toSec();
    float lidarAngle = quaternion_to_euler(lidarPose.getRotation().getZ(), lidarPose.getRotation().getW());
    float mapAngle = quaternion_to_euler(mapMetaData.origin.orientation.z, mapMetaData.origin.orientation.w);
    lidarAngle = lidarAngle + mapAngle;
    // change point cloud position
    cartographer::transform::Rigid2f expected_pose({
        mapMetaData.origin.position.x + lidarPose.getOrigin().getX() * std::cos(lidarAngle) - lidarPose.getOrigin().getY() * std::sin(lidarAngle),
        mapMetaData.origin.position.x + lidarPose.getOrigin().getX() * std::sin(lidarAngle) + lidarPose.getOrigin().getY() * std::cos(lidarAngle)},
        lidarAngle
    );
    cartographer::sensor::PointCloud cloud = cartographer::sensor::TransformPointCloud(*pointClound,
        cartographer::transform::Embed3D(expected_pose.cast<float>()));
    float score;
    cartographer::transform::Rigid2d pose_predict();
    // real_time_scan_match()
}

void RobotLocalization::initial_pos_clbk(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    cartographer::transform::Rigid2f point({msg->pose.pose.position.x, msg->pose.pose.position.y}, 
        quaternion_to_euler(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    float score;
    cartographer::transform::Rigid2d pointDouble = fast_correlative_matcher(point, score);
    
}

void RobotLocalization::real_time_scan_match(const cartographer::transform::Rigid2d& pose_prediction,
    const cartographer::sensor::PointCloud cloud)
{
    cartographer::transform::Rigid2d initial_ceres_pose = pose_prediction;
    score[0] = realTimeCorrelativeScanMatcher->Match(pose_prediction, 
        cloud, *probabilityGrid, &initial_ceres_pose);
    auto pose_observation = absl::make_unique<cartographer::transform::Rigid2d>();
    ceres::Solver::Summary summary;
    ceressScanMatcher->Match(pose_prediction.translation(), initial_ceres_pose
        , cloud, *probabilityGrid, pose_observation.get(), &summary);
    if(pose_observation)
    {
        score[1] = summary.final_cost;
        score[2] = (pose_observation->translation() - pose_prediction.translation())
            .norm();
        score[3] = std::abs(pose_observation->rotation().angle() - pose_prediction.rotation().angle());
    }
}

cartographer::transform::Rigid2d RobotLocalization::fast_correlative_matcher(cartographer::transform::Rigid2f point, float &score)
{
    float mapAngle = quaternion_to_euler(mapMetaData.origin.orientation.z, mapMetaData.origin.orientation.w);
    tf::StampedTransform lidarPose;

    float lidarAngle = point.rotation().angle();
    lidarAngle = lidarAngle + mapAngle;
    cartographer::transform::Rigid2f expected_pose({
        mapMetaData.origin.position.x + point.translation().x() * std::cos(lidarAngle) - point.translation().y() * std::sin(lidarAngle),
        mapMetaData.origin.position.x + point.translation().x() * std::sin(lidarAngle) + point.translation().y() * std::cos(lidarAngle)},
        lidarAngle
    );
    cartographer::transform::Rigid2d expected_pose_double({
        double(mapMetaData.origin.position.x + point.translation().x() * std::cos(lidarAngle) - point.translation().y() * std::sin(lidarAngle)),
        double(mapMetaData.origin.position.x + point.translation().x() * std::sin(lidarAngle) + point.translation().y() * std::cos(lidarAngle))},
        double(lidarAngle)
    );
    cartographer::sensor::PointCloud cloud = cartographer::sensor::TransformPointCloud(*pointClound,
        cartographer::transform::Embed3D(expected_pose.cast<float>()));
    cartographer::transform::Rigid2d pose_estimate;
    float min_score = 0.5;
    fastCorrelativeScanMatcher2D->Match(expected_pose_double, cloud, min_score, &score, &pose_estimate);
    return expected_pose_double;
}

cartographer::transform::Rigid2d RobotLocalization::fast_correlative_matcher(std::vector<cartographer::transform::Rigid2f> points, float &score)
{
    float max = 0.0;
    cartographer::transform::Rigid2d poseEstimate;
    for(auto point: points)
    {
        score = -1;
        cartographer::transform::Rigid2d expected_pose_double = fast_correlative_matcher(point, score);
        if(score > max)
        {
            poseEstimate = expected_pose_double;
            max = score;
        }
    }
    score = max;
    return poseEstimate;
}


void main(int args, char** argw)
{
    ros::NodeHandle n;
    RobotLocalization a(n);
}