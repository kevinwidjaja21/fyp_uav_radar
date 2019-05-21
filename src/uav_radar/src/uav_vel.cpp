#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/State.h>

using namespace std;

geometry_msgs::PoseStamped local_posMsg;
geometry_msgs::Twist velo;
geometry_msgs::Twist move_base_velMsg;

double local_posx;
double local_posy;
double local_alt;
double current_yaw;

double velo_x;
double velo_y;
double velo_xt;
double velo_yt;


double target_posx = 0;
double target_posy = 0;
double target_alt  = 0.8;
double target_yaw  = 0;

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

void sm_vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
    move_base_velMsg = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, posestamped_cb);

    ros::Subscriber move_base_sm_vel_pub = nh.subscribe<geometry_msgs::Twist>
            ("cmd_vel_mux/output", 10, sm_vel_cb);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1000);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

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

        target_posx = local_posMsg.pose.position.x;
        target_posy = local_posMsg.pose.position.y;
        //target_alt  = local_posMsg.pose.position.z;

        current_yaw = QuadtoEuler(local_posMsg.pose.orientation.w,local_posMsg.pose.orientation.x,
            local_posMsg.pose.orientation.y,local_posMsg.pose.orientation.z);
        /*
                (current_imu.orientation.w,current_imu.orientation.x,
                current_imu.orientation.y,current_imu.orientation.z);
        */

        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();
	
	while(ros::ok()){

        local_posx = local_posMsg.pose.position.x;
        local_posy = local_posMsg.pose.position.y;
        local_alt  = local_posMsg.pose.position.z;

        current_yaw = QuadtoEuler(local_posMsg.pose.orientation.w,local_posMsg.pose.orientation.x,
            local_posMsg.pose.orientation.y,local_posMsg.pose.orientation.z);

        velo.angular.z = 1 * move_base_velMsg.angular.z;

	//velo.linear.x = 1 * (move_base_velMsg.linear.x);
	//velo.linear.y = 1 * (move_base_velMsg.linear.y);

        velo_x = 1 * (move_base_velMsg.linear.x);
        velo_y = 1 * (move_base_velMsg.linear.y);
	
        
        if ((velo_x < 0.001) && (velo_x > -0.001))
        {
            velo.linear.x = (target_posx - local_posx);
        }
        else
        {
            velo.linear.x = velo_x * cos(current_yaw);
            target_posx = local_posMsg.pose.position.x;
        }
        
        if ((velo_x < 0.001) && (velo_x > -0.001))
        {
            velo.linear.y = (target_posy - local_posy);
        }
        else
        {
            velo.linear.y = velo_x * sin(current_yaw);
            target_posy = local_posMsg.pose.position.y;
        }

        velo.linear.z = 1 * (target_alt  -  local_alt);
		
        /*
        printf("pos X   = %lf \n",local_posx);
        printf("pos Y   = %lf \n",local_posy);
        printf("alti    = %lf \n",local_alt);
        printf("heading = %lf \n",current_yaw * 180/ 3.14159265359);
        printf("\n");
        */
 
        local_vel_pub.publish(velo);
        ros::spinOnce();
        rate.sleep();
		
 }

    return 0;
}

