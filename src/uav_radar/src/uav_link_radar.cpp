#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

static double current_x =0.0;
static double current_y =0.0;
static double current_z =0.0;
static geometry_msgs::Quaternion current_q;
ros::Publisher odom_pub;

void pub_tf_odom(void);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
int dummyid =0;

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle nh;

  //nh.getParam("dummyid", dummyid);
  std::string localstr("/mavros");
  if(dummyid!=0)
    localstr = localstr + patch::to_string(dummyid);

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>((localstr + "/local_position/pose").data(), 10, pose_cb);

  ros::spin();

  return 0;
}

void pub_tf_odom(void)
{
  static tf::TransformBroadcaster br;
  ros::Time current_time = ros::Time::now();
  //tf::Transform odom_trans;
  // transform.setOrigin( tf::Vector3(current_x, current_y, current_z) );
  // // tf::Quaternion q;
  // // q.setRPY(0, 0, current_yaw);
  // transform.setRotation(current_q);

  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";

  odom_trans.transform.translation.x = current_x;
  odom_trans.transform.translation.y = current_y;
  odom_trans.transform.translation.z = current_z;
  odom_trans.transform.rotation = current_q;

  //send the transform
  br.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = current_x;
  odom.pose.pose.position.y = current_y;
  odom.pose.pose.position.z = current_z;
  odom.pose.pose.orientation = current_q;

  //publish the message
  odom_pub.publish(odom);
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  //alt_value.local is the height
  current_x = msg->pose.position.x;
  current_y = msg->pose.position.y;
  current_z = msg->pose.position.z;
  current_q = msg->pose.orientation;
  //ROS_INFO("current_yaw %f",current_yaw);
  pub_tf_odom();
}