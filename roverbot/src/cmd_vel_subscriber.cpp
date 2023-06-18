#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cmath>
#include<tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

using namespace std;
nav_msgs::Odometry odom;
sensor_msgs::NavSatFix gps_msg;
float current_latitude ;
float current_longitude ;

double roll=0.0,pitch =0.0,yaw=0.0;
void odomcallback(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
    tf::Quaternion orientation_q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
        tf::Matrix3x3(orientation_q).getRPY(roll,pitch,yaw);
        }

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){

    current_latitude =msg->latitude;
    current_longitude =msg->longitude;
}       

float distancefn(float target_latitude,float target_longitude){
    
int R = 6371e3;
float phi1 = current_latitude* M_PI/180; 
float phi2=  target_latitude* M_PI/180;
float dphi = (target_latitude- current_latitude) * M_PI/180;
float dlambda = (target_longitude-current_longitude) * M_PI/180;

float a = sin(dphi/2) * sin(dphi/2) +
           cos(phi1) * cos(phi2) *
           sin(dlambda/2) * sin(dlambda/2);
float c = 2 * atan2(sqrt(a), sqrt(1-a));

float distance1 = R * c;
return distance1;

}
float current_distancefn(float current_latitude,float current_longitude){
    
int R = 6371e3;
float phi1 = 0* M_PI/180; 
float phi2=  current_latitude* M_PI/180;
float dphi = (current_latitude) * M_PI/180;
float dlambda = (current_longitude) * M_PI/180;

float a = sin(dphi/2) * sin(dphi/2) +
           cos(phi1) * cos(phi2) *
           sin(dlambda/2) * sin(dlambda/2);
float c = 2 * atan2(sqrt(a), sqrt(1-a));

float distance2 = R * c;
return distance2;}

float target_distancefn(float target_latitude,float target_longitude){
    
int R = 6371e3;
float phi1 = 0* M_PI/180; 
float phi2=  (target_latitude)* M_PI/180;
float dphi = (target_latitude) * M_PI/180;
float dlambda = (target_longitude) * M_PI/180;

float a = sin(dphi/2) * sin(dphi/2) +
           cos(phi1) * cos(phi2) *
           sin(dlambda/2) * sin(dlambda/2);
float c = 2 * atan2(sqrt(a), sqrt(1-a));

float distance3 = R * c;
return distance3;}

float targetfn( float target_latitude ,float target_longitude){
int R = 6371e3;
float phi1 = (0)* M_PI/180; 
float phi2=  (target_latitude)* M_PI/180;
float dphi = (target_latitude) * M_PI/180;
float dlambda = (target_longitude )* M_PI/180;
float y = sin(dlambda) * cos(phi2);
float x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dlambda);
float theta =   M_PI_2 - atan2(y, x);
float target1= fmod((theta*180/M_PI + 360), 360); 
return target1;
}

int main(int argc, char **argv){
ros::init(argc, argv, "globalplanner");
ros::NodeHandle nh;
ros::Subscriber gps_sub = nh.subscribe("/gps/fix", 100, gpsCallback);
ros::Subscriber odom_sub = nh.subscribe("/roverbot/odom", 100, odomcallback);
ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/roverbot/cmd_vel", 100);
geometry_msgs::Twist cmd_vel;
//sensor_msgs::NavSatFix:: gps_msg;

ros::Rate rate(10.0);

double target_latitude, target_longitude ;
cout << "Enter latitude ";
cin >> target_latitude;
cout << "Enter longitude ";
cin >> target_longitude; 

while (ros::ok()){   
float target =targetfn(target_latitude, target_longitude)*M_PI/180;
float distance =distancefn(target_latitude, target_longitude);
float target_distance =target_distancefn(target_latitude, target_longitude);
float current_distance =current_distancefn(current_latitude, current_longitude);

cout<<"target angle: "<<target<<"current angle: "<<yaw<<endl;
cout<<" target-yaw: "<<abs(target-yaw)<<endl;
cout<<" "<<endl;
cout << "target_distance: "<< target_distance<< endl;
cout << "current_distance: "<< current_distance<< endl;
//cout<<"How much near is the rover from the target?: "<<distance <<endl;
cout<<"current latitude: "<< current_latitude <<"  current longitude: "<< current_longitude<<endl;
cout<<"target latitude: "<< target_latitude<<"  target longitude: "<< target_longitude<<endl;


if((abs(target - yaw)>0.01 ) && current_distance< target_distance){
    cmd_vel.angular.z=0.5*(target-yaw);
} 
else if((abs(target-yaw)<=0.01) && current_distance< target_distance){
    cmd_vel.angular.z= 0.0;
    cmd_vel.linear.x=0.5* (target_distance- current_distance);
    cmd_vel.linear.y=0.5*(target_distance - current_distance);}

else if((abs(target - yaw)<=0.01) && (current_distance == target_distance)){
    cmd_vel.linear.x=0.0;
    cmd_vel.linear.y=0.0;
    cmd_vel.angular.z= 0.0;
}
else{
    cmd_vel.linear.x=0.0;
    cmd_vel.linear.y=0.0;
    cmd_vel.angular.z= 0.0;}

  
  
cmd_vel_pub.publish(cmd_vel);
rate.sleep();
ros::spinOnce();
}
    return 0;
}
