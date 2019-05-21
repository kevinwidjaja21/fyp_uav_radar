#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  // tf::TransformBroadcaster br;
  tf::TransformBroadcaster br1;

  tf::Transform transform;
  tf::Transform transform1;

  ros::Rate rate(10.0);
  while (node.ok()){
    // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    // transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint"));

    transform1.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform1.setRotation( tf::Quaternion(1, 0, 0, 0) );
    br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_footprint", "base_radar_link"));
    rate.sleep();
  }
  return 0;
};
