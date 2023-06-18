#include <ros/ros.h>
#include <std_msgs/String.h>
#include <chatroom/msge.h>

using namespace std;

void Callback(const chatroom::msge::ConstPtr& msg) {
  ROS_INFO("%s: %s", msg->username.c_str(), msg->chat.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "node3");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("chatroom", 1000, Callback);
  ros::Publisher pub = nh.advertise<chatroom::msge>("chatroom", 1000);

  chatroom::msge msg;

  string user;
  cout<<"Enter Name: ";
  getline(cin,user);
  msg.username = user;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  while (ros::ok()) {
    string input;
    getline(cin, input);
    msg.chat = input;
    pub.publish(msg);
  }

  return 0;
}