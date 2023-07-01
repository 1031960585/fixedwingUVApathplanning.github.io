#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <mavros_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>

// 弃用
bool desired_is_get = false;
bool generated_is_get = false;

mavros_msgs::Trajectory trajectory_desired;
void trajectory_desired_cb(const mavros_msgs::Trajectory::ConstPtr &msg){
    trajectory_desired = *msg;
    desired_is_get = true;
}

mavros_msgs::Trajectory trajectory_generated;
void trajectory_generated_cb(const mavros_msgs::Trajectory::ConstPtr &msg){
    trajectory_generated = *msg;
    generated_is_get = true;
}

geometry_msgs::PoseStamped point;
nav_msgs::Path trajectory_pub;

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_path");
    ros::NodeHandle nh("~");

    std::string ns = nh.param<std::string>("ns","");

    ros::Subscriber trajectory_desired_sub = nh.subscribe<mavros_msgs::Trajectory>(ns+"/mavros/trajectory/desired", 1, trajectory_desired_cb);
    ros::Subscriber trajectory_generated_sub = nh.subscribe<mavros_msgs::Trajectory>(ns+"/mavros/trajectory/generated", 1, trajectory_generated_cb);

    ros::Publisher trajectory_desired_pub = nh.advertise<nav_msgs::Path>("/trajectory_path/desired", 1);
    ros::Publisher trajectory_generated_pub = nh.advertise<nav_msgs::Path>("/trajectory_path/generated", 1);


    ros::Rate rate(5);
    trajectory_pub.header.frame_id = "map";

    while(ros::ok()){
        // 回调函数拿到数据
        ros::spinOnce();

        if(desired_is_get){
            //geometry_msgs::PoseStamped
            point.pose.position = trajectory_desired.point_1.position;
            trajectory_pub.poses.push_back(point);
            point.pose.position = trajectory_desired.point_2.position;
            trajectory_pub.poses.push_back(point);
            point.pose.position = trajectory_desired.point_3.position;
            trajectory_pub.poses.push_back(point);
            point.pose.position = trajectory_desired.point_4.position;
            trajectory_pub.poses.push_back(point);
            point.pose.position = trajectory_desired.point_5.position;
            trajectory_pub.poses.push_back(point);

            trajectory_desired_pub.publish(trajectory_pub);

            trajectory_pub.poses.clear();
            desired_is_get = false;
        }

        if(generated_is_get){
            //geometry_msgs::PoseStamped
            point.pose.position = trajectory_generated.point_1.position;
            trajectory_pub.poses.push_back(point);
            point.pose.position = trajectory_generated.point_2.position;
            trajectory_pub.poses.push_back(point);
            point.pose.position = trajectory_generated.point_3.position;
            trajectory_pub.poses.push_back(point);
            point.pose.position = trajectory_generated.point_4.position;
            trajectory_pub.poses.push_back(point);
            point.pose.position = trajectory_generated.point_5.position;
            trajectory_pub.poses.push_back(point);

            trajectory_generated_pub.publish(trajectory_pub);

            trajectory_pub.poses.clear();
            generated_is_get = false;
        }

    rate.sleep();
    }
}