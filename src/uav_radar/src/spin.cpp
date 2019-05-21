/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
/*
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
*/
#include <mavros_msgs/State.h>


using namespace std;

geometry_msgs::PoseStamped local_posMsg;
geometry_msgs::Twist velo;

double local_posx;
double local_posy;
double local_alt;
double current_yaw;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double QuadtoEuler(double q_w, double q_x, double q_y, double q_z)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q_w * q_x + q_y * q_z);
  double cosr_cosp = +1.0 - 2.0 * (q_x * q_x + q_y * q_y);
  double roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q_w * q_y - q_z * q_x);
  if (fabs(sinp) >= 1)
  double   pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
  double   pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
  double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);  
  double yaw = atan2(siny_cosp, cosy_cosp);
  return yaw;
}

void posestamped_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_posMsg = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "spin");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    /*ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);*/
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, posestamped_cb);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1000);
    /*ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");*/

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    velo.linear.x = 0;
    velo.linear.y = 0;
    velo.linear.z = 0;
    velo.angular.x = 0;
    velo.angular.y = 0;
    velo.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_posx = local_posMsg.pose.position.x;
        local_posy = local_posMsg.pose.position.y;
        local_alt  = local_posMsg.pose.position.z;


        current_yaw = QuadtoEuler(local_posMsg.pose.orientation.w,local_posMsg.pose.orientation.x,
            local_posMsg.pose.orientation.y,local_posMsg.pose.orientation.z);
        /*
                (current_imu.orientation.w,current_imu.orientation.x,
                current_imu.orientation.y,current_imu.orientation.z);
        */
        //local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    /*
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    */

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        /*
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }*/
        local_posx = local_posMsg.pose.position.x;
        local_posy = local_posMsg.pose.position.y;
        local_alt  = local_posMsg.pose.position.z;

        current_yaw = QuadtoEuler(local_posMsg.pose.orientation.w,local_posMsg.pose.orientation.x,
            local_posMsg.pose.orientation.y,local_posMsg.pose.orientation.z);
        /*
                (current_imu.orientation.w,current_imu.orientation.x,
                current_imu.orientation.y,current_imu.orientation.z);
        */

        //local_pos_pub.publish(pose);
        
        if (ros::Time::now() - last_request > ros::Duration(20.0))
        {
            
            if (current_yaw >= -0.1 && current_yaw <= 0.1)
            {
                velo.angular.z = 0;
            }
            else if (current_yaw < -0.1)
            {
                velo.angular.z = 0.02;
            }
            else if (current_yaw > 0)
            {
                velo.angular.z = -0.02;
            }

            if (local_alt >= 0.79 && local_alt <= 0.81)
            {
                velo.linear.z = 0;
            }
            else if (local_alt < 0.79)
            {
                velo.linear.z = 0.02;
            }
            else if (local_alt > 0.81)
            {
                velo.linear.z = -0.02;
            }
        }

        printf("pos X   = %lf \n",local_posx);
        printf("pos Y   = %lf \n",local_posy);
        printf("alti    = %lf \n",local_alt);
        printf("heading = %lf \n",current_yaw * 180/ 3.14159265359);
        printf("\n");

        local_vel_pub.publish(velo);
        ros::spinOnce();
        rate.sleep();

        //else
        //{
        //    pose.pose.orientation.z = 3.14159265359;
        //    velo.twist.angular.z = 0;
        //}
        /*
        if (ros::Time::now() - last_request > ros::Duration(40.0))
        {
            pose.pose.position.z = 0.5;
        }*/
    }

    return 0;
}
