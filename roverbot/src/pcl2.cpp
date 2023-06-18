#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
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
#include <iostream>
#include <cmath>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

class ObstacleDetection{
public:


}
















ros::Publisher laser_scan_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*point_cloud_msg, *cloud);

    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(0.01, 0.01, 0.01);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    vox.filter(*filtered_cloud);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(filtered_cloud);
    seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*ground_removed_cloud);

    sensor_msgs::LaserScan laser_scan_msg;
    laser_scan_msg.header = point_cloud_msg->header;
    laser_scan_msg.angle_min = -M_PI / 2.0;
    laser_scan_msg.angle_max = M_PI / 2.0;
    laser_scan_msg.angle_increment = (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / ground_removed_cloud->width;
    laser_scan_msg.time_increment = 0.0;
    laser_scan_msg.scan_time = 0.1;
    laser_scan_msg.range_min = 0.0;
    laser_scan_msg.range_max = 10.0;
    laser_scan_msg.ranges.resize(ground_removed_cloud->width);

    for (size_t i = 0; i < ground_removed_cloud->width; ++i)
    {
        const pcl::PointXYZRGB &point = ground_removed_cloud->points[i];
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
        {
            laser_scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        }
        else
        {
            laser_scan_msg.ranges[i] = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        }
    }

    laser_scan_pub.publish(laser_scan_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl2");
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub = nh.subscribe("/camera/depth/points", 1, pointCloudCallback);
    laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/laser_scan", 1);
    geometry_msgs::Twist cmd_vel;
    sensor_msgs::LaserScan laser_scan_msg;
    ros::Rate rate(10.0);
    ros::spin();

    while (ros::ok())
    {
        ROS_INFO("%f", laser_scan_msg.ranges);
        laser_scan_pub.publish(laser_scan_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
