#include <ros/ros.h>
#include <turtlebot3_msgs/SensorState.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

int left_encoder =0;
int right_encoder =0;

int lastleft = 0;
int lastright = 0;

double x = 0;
double y = 0;
double th = 0;

ros::Publisher odometry_pub;

// Odom Topic Publisher
// TF Publisher

void SensorCallback(const turtlebot3_msgs::SensorState::ConstPtr& sensor_state)
{
  ROS_INFO("subscribe sensor_state : %.d, %.d", sensor_state->left_encoder, sensor_state->right_encoder);
  left_encoder = sensor_state->left_encoder;
  right_encoder = sensor_state->right_encoder;

  double distance_per_tick = (2*M_PI*0.033)/4096;

  int delta_left_tick = left_encoder - lastleft;
  int delta_right_tick = right_encoder - lastright;

  double delta_d_l = distance_per_tick*delta_left_tick;
  double delta_d_r = distance_per_tick*delta_right_tick;

  double delta_d = (delta_d_l + delta_d_r)/2;
  double delta_th = (delta_d_r - delta_d_l)/0.16;

  x = x + delta_d*cos(th + delta_th);
  y = y + delta_d*sin(th + delta_th);
  th = th + delta_th;

  lastleft = left_encoder;
  lastright = right_encoder;

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, th);
  odom_msg.pose.pose.orientation = tf2::toMsg(q);
  odometry_pub.publish(odom_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_node");
  ros::NodeHandle n;

  ros::Subscriber subscriber_sensor_state;
  subscriber_sensor_state = n.subscribe("sensor_state", 1000, SensorCallback);

  odometry_pub = n.advertise<nav_msgs::Odometry>("turtlebot_odom",1000);

  tf::TransformBroadcaster broadcaster;
   
  ros::Rate loop_rate(60);

  while (ros::ok())
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, th);
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(x, y, 0.0)),
                                                 ros::Time::now(),
                                                 "turtlebot_odom",
                                                 "turtlebot_base_link"));
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
