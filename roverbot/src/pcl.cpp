#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/NavSatFix.h>

ros::Publisher laser_scan_pub;
ros::Publisher cmd_vel_pub;
tf::TransformListener* tf_listener_ptr;
bool obstacle_detected = false;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Convert the PointCloud2 message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

  // Transform the point cloud into the base frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  tf_listener_ptr->waitForTransform(robot_footprint, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(1.0));
  pcl_ros::transformPointCloud(robot_footprint, *pcl_cloud, *transformed_cloud, *tf_listener_ptr);

  // Create a LaserScan message
  sensor_msgs::LaserScan laser_scan_msg;
  laser_scan_msg.header = cloud_msg->header;
  laser_scan_msg.angle_min = -M_PI / 2.0;  // Define the minimum angle of the laser scan (adjust as needed)
  laser_scan_msg.angle_max = M_PI / 2.0;   // Define the maximum angle of the laser scan (adjust as needed)
  laser_scan_msg.angle_increment = (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / pcl_cloud->width;
  laser_scan_msg.time_increment = 0.0;      // Set to 0 for simulated data or adjust as needed for real-time data
  laser_scan_msg.scan_time = 0.1;           // Set to 0.1 for simulated data or adjust as needed for real-time data
  laser_scan_msg.range_min = 0.0;           // Define the minimum range of the laser scan (adjust as needed)
  laser_scan_msg.range_max = 10.0;          // Define the maximum range of the laser scan (adjust as needed)

  // Populate the ranges array of the LaserScan message
  laser_scan_msg.ranges.resize(pcl_cloud->width);
  for (size_t i = 0; i < pcl_cloud->width; ++i)
  {
    const pcl::PointXYZ& point = transformed_cloud->points[i];
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
    {
      laser_scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
    }
    else
    {
      // Calculate the range using Euclidean distance
      laser_scan_msg.ranges[i] = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    }
  }

  // Publish the LaserScan message
  laser_scan_pub.publish(laser_scan_msg);

  // Perform obstacle avoidance (Bug 0 algorithm)
  if (obstacle_detected)
  {
    // Rotate counter-clockwise (can be adjusted based on your robot's control)
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = 0.5; // Adjust the angular velocity as needed
    cmd_vel_pub.publish(cmd_vel);
  }
  else
  {
    // Move forward (can be adjusted based on your robot's control)
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.2; // Adjust the linear velocity as needed
    cmd_vel_pub.publish(cmd_vel);
  }
}

void obstacleDetectionCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg)
{
  // Check if an obstacle is detected
  for (const float range : laser_scan_msg->ranges)
  {
    if (range < 1.0) // Adjust the threshold distance as needed
    {
      obstacle_detected = true;
      return;
    }
  }
  obstacle_detected = false;
}


void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg)
{
  // Retrieve target longitude and latitude from the GPS fix message
  double target_longitude = gps_fix_msg->longitude;
  double target_latitude = gps_fix_msg->latitude;

  // Implement your navigation logic here using the target longitude and latitude
  // Adjust the velocity commands to move the rover towards the target location
  // ...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_obstacle_avoidance");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener;
  tf_listener_ptr = &tf_listener;

  laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/roverbot/cmd_vel", 1);

  ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("kinect_point_cloud", 1, pointCloudCallback);
  ros::Subscriber laser_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("laser_scan", 1, obstacleDetectionCallback);

  ros::spin();

  return 0;
}




#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>

class ObstacleDetection
{
public:
  ObstacleDetection(std::string sub_topic_pc, std::string sub_topic_gps, ros::NodeHandle n);
  float obstacle_threshold =0.5;

  ros::Subscriber sub_pc_;
  ros::Subscriber sub_gps_;
  ros::Publisher cmd_vel_pub_;

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg);
  void gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg);
  void filterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud);
  void groundPlaneElimination(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing);
  void clusterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
  pcl::PointXYZ calculateCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  void avoidObstacles(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
};
ObstacleDetection oa;
ObstacleDetection::ObstacleDetection(std::string sub_topic_pc, std::string sub_topic_gps, ros::NodeHandle n)
{
  sub_pc_ = n.subscribe(sub_topic_pc, 1, &ObstacleDetection::pointCloudCallback, this);
  sub_gps_ = n.subscribe(sub_topic_gps, 1, &ObstacleDetection::gpsCallback, this);
  cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/roverbot/cmd_vel", 1);
}

void ObstacleDetection::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg)
{
  // Convert the received PointCloud2 message to PCL format
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*point_cloud_msg, *cloud);

  // Preprocessing: Voxelization and Ground Plane Elimination
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  filterPointCloud(cloud, filtered_cloud);
  groundPlaneElimination(filtered_cloud);

  // Clustering
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  clusterPointCloud(filtered_cloud, clusters);

  // Avoid obstacles
  avoidObstacles(clusters);
}
void imucallback(const sensor_msgs::Imu::ConstPtr &msg)
{

    tf::Quaternion orientation_q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3(orientation_q).getRPY(nav.roll, nav.pitch, nav.yaw);
}

void ObstacleDetection::gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg)
{
  double latitude = gps_msg->latitude;
  double longitude = gps_msg->longitude;
  double altitude = gps_msg->altitude;

  // Use GPS information for navigation or contextual awareness
}

void ObstacleDetection::filterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud)
{
  // Voxelization (Downsampling)
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(input_cloud);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*filtered_cloud);
}

void ObstacleDetection::groundPlaneElimination(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setEpsAngle(0.5);

  seg.setInputCloud(cloud_processing);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    ROS_INFO("Could not estimate a planar model for the given dataset.");
    return;
  }

  // Extract the ground plane inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud_processing);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_processing);
}

void ObstacleDetection::clusterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
  // Clustering parameters
  float cluster_tolerance = 0.2;    // Maximum distance between points in the same cluster
  int min_cluster_size = 100;       // Minimum number of points that constitute a cluster
  int max_cluster_size = 100000;    // Maximum number of points that constitute a cluster

  // Euclidean clustering
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud_processing);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_processing);
  ec.extract(cluster_indices);

  // Extract cluster point clouds
  for (const auto &indices : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &index : indices.indices)
    {
      cluster->points.push_back(cloud_processing->points[index]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
  }
}

pcl::PointXYZ ObstacleDetection::calculateCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
  for (const pcl::PointXYZRGB &point : *cloud)
  {
    centroid.add(point);
  }

  pcl::PointXYZRGB centroid_point;
  centroid.get(centroid_point);
  pcl::PointXYZ centroid_xyz;
  centroid_xyz.x = centroid_point.x;
  centroid_xyz.y = centroid_point.y;
  centroid_xyz.z = centroid_point.z;
  return centroid_xyz;
}

void ObstacleDetection::avoidObstacles(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
  // Find the cluster with the closest centroid
  pcl::PointXYZ closest_centroid;
  double closest_distance = std::numeric_limits<double>::max();
  for (const auto &cluster : clusters)
  {
    pcl::PointXYZ centroid = calculateCentroid(cluster);
    double distance = std::sqrt(std::pow(centroid.x, 2) + std::pow(centroid.y, 2));
    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest_centroid = centroid;
    }
  }

  // Perform obstacle avoidance based on the closest centroid
  if (closest_distance < obstacle_threshold)
  {
    // Adjust robot's motion to avoid the obstacle
    double linear_velocity = 0.0;   // Adjust according to your requirements
    double angular_velocity = 0.0;  // Adjust according to your requirements

    // Publish motion command
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;
    cmd_vel_pub_.publish(cmd_vel);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud");
  ros::NodeHandle nh;

  std::string sub_topic_pc = "/camera/depth/points";
  std::string sub_topic_gps = "/gps/fix";

  ObstacleDetection obstacle_detection(sub_topic_pc, sub_topic_gps, nh);

  ros::spin();

  return 0;
}
