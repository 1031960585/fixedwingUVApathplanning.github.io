#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>

// 获取UTM坐标系位置和gps速度矢量   1、发布飞行器位姿作为起点   2、发布gps坐标系到map坐标系的转换

// 坐标变换的数据
geometry_msgs::TransformStamped tfs;

// 用于欧拉角和四元数之间转换
tf2::Quaternion qtn;

// 用于发布的起始位姿
geometry_msgs::PoseWithCovarianceStamped pose_cur;

// 飞行器状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// gps位置
nav_msgs::Odometry UTM_position;
void gps_cb(const nav_msgs::Odometry::ConstPtr& msg){
    UTM_position = *msg;
}

// gps速度
geometry_msgs::TwistStamped gp_vel;
void gp_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    gp_vel = *msg;
}

// 来自 FCU 的本地位置
// geometry_msgs::PoseStamped local_position;
// void local_cb(const geometry_msgs::PoseStampedConstPtr& msg){
//     local_position = *msg;
// }

// 来自 FCU 的速度数据
geometry_msgs::TwistStamped local_vel;
void local_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_vel = *msg;
}


//起始位置           经纬度（117.391556，39.546074） UTM(534205.005024,4377862.363357)

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "gps_2_init_pose");
    ros::NodeHandle nh("~");

    std::string ns = nh.param<std::string>("ns","");

    // // 获取UTM坐标系位置
    ros::Subscriber global_position_sub = nh.subscribe<nav_msgs::Odometry>("/UTM_position", 1, gps_cb);
    // // 获取GPS速度
    // ros::Subscriber gp_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(ns+"/mavros/global_position/raw/gps_vel", 1, gp_vel_cb);
    // 来自 FCU 的本地位置
    // ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(ns+"/mavros/local_position/pose", 1, local_cb);
    // 来自 FCU 的速度数据
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(ns+"/mavros/local_position/velocity_local", 1, local_velocity_cb);
    // 获取飞行器状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(ns+"/mavros/state", 1, state_cb);
    // 发布起始点位姿
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_cur", 10);
    // 用于发布坐标变换
    tf2_ros::TransformBroadcaster broadcaster;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    tfs.header.frame_id = "map";
    tfs.child_frame_id = "gps";

    while(ros::ok()){

        // 加限制条件
        qtn.setRPY(0,0,atan2(local_vel.twist.linear.y, local_vel.twist.linear.x));

        pose_cur.pose.pose.position.x = UTM_position.pose.pose.position.x-533081.8341;
        pose_cur.pose.pose.position.y = UTM_position.pose.pose.position.y-4377039.7674;
        pose_cur.pose.pose.orientation.x = qtn.getX();
        pose_cur.pose.pose.orientation.y = qtn.getY();
        pose_cur.pose.pose.orientation.z = qtn.getZ();
        pose_cur.pose.pose.orientation.w = qtn.getW();
        // 发布规划的起始位置
        local_pos_pub.publish(pose_cur);

        tfs.header.stamp = ros::Time::now();
        tfs.transform.translation.x = UTM_position.pose.pose.position.x-533081.8341;
        tfs.transform.translation.y = UTM_position.pose.pose.position.y-4377039.7674;
        tfs.transform.translation.z = 0;

        tfs.transform.rotation.x =qtn.getX();
        tfs.transform.rotation.y =qtn.getY();
        tfs.transform.rotation.z =qtn.getZ();
        tfs.transform.rotation.w =qtn.getW();
        // 通过GPS数据，发布飞机到map的坐标转换
        broadcaster.sendTransform(tfs);

        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}