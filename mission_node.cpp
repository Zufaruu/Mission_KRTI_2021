/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "pid.hpp"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

void moving_to_wp(){
    
}

void info_wp(bool done, int wp_next, float x, float y, float z){
    if(done){
        ROS_INFO("DONE");
    }
    else if(wp_next == 6){
        ROS_INFO("Menuju ELP");
    }
    else if(wp_next % 2 == 0){
        ROS_INFO("Scanning gedung");   
    }
    else {
        ROS_INFO("Menuju gedung");
    }
    ROS_INFO("x = %f, y = %f, z = %f", x, y, z);
}

float jarak_titik(float tujuan_x, float tujuan_y, float tujuan_z, float asal_x, float asal_y, float asal_z){
    float jarak_x = tujuan_x - asal_x;
    float jarak_y = tujuan_y - asal_y;
    float jarak_z = tujuan_z - asal_z;
    float jarak_total = sqrt(pow(jarak_x, 2) + pow(jarak_y, 2) + pow(jarak_z, 2));
    return jarak_total;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    bool takeoff = false;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist> //Publish velocity
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Subscriber current_pos = nh.subscribe<geometry_msgs::PoseStamped> //Subscribe position
            ("mavros/local_position/pose", 10, callback);
    ros::Publisher wp_x = nh.advertise<std_msgs::Float32>
            ("wp/x", 10);
    ros::Publisher wp_y = nh.advertise<std_msgs::Float32>
            ("wp/y", 10);
    ros::Publisher wp_z = nh.advertise<std_msgs::Float32>
            ("wp/z", 10);
            
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 0;

    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok() && (current_state.mode != "OFFBOARD" || !current_state.armed)){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(3.0))){
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
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    PID mypid_x(0.02, 1, -1, 0.5, 0.001, 0.001);
    PID mypid_y(0.02, 1, -1, 0.5, 0.001, 0.001);
    PID mypid_z(0.02, 1, -1, 0.5, 0.001, 0.001); 

    ros::Time cek_waktu;

    float wp[][3] = {
        {   8.0,  8.0,  4.0},  //scanning gedung I
        {   15.0,  8.0,  5.0}, //gedung I
        {   8.0,  1.0,  4.0},  //scanning gedung II
        {   17.5,  0.0,  5.0}, //gedung II
        {   8.0, -8.0,  4.0},  //scanning gedung III
        {   15.5, -8.0,  5.0}, //gedung III
        {   9.0,  4.0,  4.0}, //elp
    };
     
    std_msgs::Float32 wp23[7][3];
    for(int i=0;i<7;i++){
        for(int j=0;j<3;j++){
            wp23[i][j].data = wp[i][j];
        }
    }

    ros::Time waktu_wp;
    ros::Duration selang_waktu[7];   

    int wp_next = 0;
    bool done = false;
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        if (!takeoff){
            vel.linear.x = mypid_x.calculate(0, current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(0, current_position.pose.position.y);
            vel.linear.z = mypid_z.calculate(3, current_position.pose.position.z);
            cmd_pub.publish(vel);

            if (current_position.pose.position.z < 3){
                cek_waktu = ros::Time::now();
                ROS_INFO("Taking off!");
                ROS_INFO("pos_z = %f. vel_z = %f", current_position.pose.position.z, vel.linear.z);
            }
            else if (ros::Time::now() - cek_waktu > ros::Duration(1.5)){
                takeoff = true;
                ROS_INFO("Ready to Move");
                waktu_wp = ros::Time::now();
            }
        }
        
        else if (wp_next < 7){
            vel.linear.x = mypid_x.calculate(wp[wp_next][0], current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(wp[wp_next][1], current_position.pose.position.y);
            vel.linear.z = mypid_z.calculate(wp[wp_next][2], current_position.pose.position.z);
            cmd_pub.publish(vel);

            wp_x.publish(wp23[wp_next][0]);
            wp_y.publish(wp23[wp_next][1]);
            wp_z.publish(wp23[wp_next][2]);

            if (jarak_titik(wp[wp_next][0], wp[wp_next][1], wp[wp_next][2], 
            current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z) > 0.3){
                cek_waktu = ros::Time::now();
                done = false;
                info_wp(done, wp_next, current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z);
            }
            else if (ros::Time::now() - cek_waktu > ros::Duration(2)){
                selang_waktu[wp_next] = ros::Time::now() - waktu_wp;
                waktu_wp = ros::Time::now();
                wp_next += 1;
                done = true;
                info_wp(done, wp_next, current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z);
            }
        }
       
        else {
            vel.linear.x = mypid_x.calculate(wp[6][0], current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(wp[6][1], current_position.pose.position.y);
            vel.linear.z = mypid_z.calculate(0, current_position.pose.position.z);
            cmd_pub.publish(vel);

            ROS_INFO("Landing...");
            ROS_INFO("z = %f", current_position.pose.position.z);
            if (jarak_titik(wp[6][0], wp[6][1], 0, current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z) < 0.5){
                offb_set_mode.request.custom_mode = "AUTO.LAND";
		        set_mode_client.call(offb_set_mode);
            } 
        } 

        ros::spinOnce();
        rate.sleep();
    }
    
    for(int i=0;i<7;i++){
        std::cout << i << " ke " << i+1 << ": " << selang_waktu[i] << std::endl;  
    }

    ROS_INFO("MAY DAY!, MAYDAY!");
    

    return 0;
}
