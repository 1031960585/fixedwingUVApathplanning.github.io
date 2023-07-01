#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// 飞控将下一个任务航点发送回来，收到后加上home点的偏移量，显示到rivz上
// 不封装在类里面的话，回调函数goal_cb里拿不到pub_
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal_offset", 1);
 
    //Topic you want to subscribe
    goal_sub_ = n_.subscribe("/move_base_simple/goal", 1, &SubscribeAndPublish::goal_cb, this);
    home_sub_ = n_.subscribe("/UTM_position/home", 1, &SubscribeAndPublish::home_cb, this);
  }

  void home_cb(const nav_msgs::Odometry& input){
    home_UTM = input;
  }

  void goal_cb(const geometry_msgs::PoseStamped& input)
  {
    geometry_msgs::PoseStamped output = input;
    //.... do something with the input and generate the output...
    output.pose.position.x += home_UTM.pose.pose.position.x-533081.8341;
    output.pose.position.y += home_UTM.pose.pose.position.y-4377039.7674;

    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber home_sub_;
  nav_msgs::Odometry home_UTM;
 
};


int main(int argc, char **argv){
    ros::init(argc, argv, "goal_offset");
    SubscribeAndPublish SAPObject;

    ros::spin();
    return 0;

}