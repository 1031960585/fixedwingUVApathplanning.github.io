#include <ros/ros.h>

#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/NavSatFix.h>


mavros_msgs::HomePosition home_position;
void home_position_cb(const mavros_msgs::HomePosition::ConstPtr& msg){
    home_position = *msg;
}

sensor_msgs::NavSatFix home;

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "pub_home");
    ros::NodeHandle nh("~");

    std::string ns = nh.param<std::string>("ns","");

    // 订阅home位置
    ros::Subscriber home_position_sub = nh.subscribe<mavros_msgs::HomePosition>(ns+"/mavros/home_position/home", 1, home_position_cb);

    // 发布home位置到utm_odometry_node
    ros::Publisher home_position_pub = nh.advertise<sensor_msgs::NavSatFix>("/home_position/home", 10);

    ros::Rate rate(4.0);
    home.header.frame_id = "base_link";
    home.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    home.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    while(ros::ok()){
        home.header.stamp = ros::Time::now();
        home.latitude = home_position.geo.latitude;
        home.longitude = home_position.geo.longitude;
        
        home_position_pub.publish(home);
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}