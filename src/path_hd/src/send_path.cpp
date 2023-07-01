#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>

#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>

// #include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandHome.h>

#include <gps_common/conversions.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

visualization_msgs::MarkerArray vehicle_array;
void vehicle_path_cb(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    vehicle_array = *msg;
}

nav_msgs::Path nav_path;
void searched_path_cb(const nav_msgs::Path::ConstPtr &msg)
{
    nav_path = *msg;
}

// 来自 FCU 的本地位置
geometry_msgs::PoseStamped local_position;
void local_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    local_position = *msg;
}

nav_msgs::Odometry home_UTM;
void home_UTM_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    home_UTM = *msg;
}

geographic_msgs::GeoPoseStamped target_pos_global;
geometry_msgs::PoseStamped target_pos_local;

// nav_msgs::Path target_nav_path;

bool follow_trajectory(ros::Publisher &target_trajectory_pub);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_path");
    ros::NodeHandle nh("~");

    std::string ns = nh.param<std::string>("ns", "");
    std::string firmware = nh.param<std::string>("firmware", "px4");

    // 发送航点或轨迹
    // ros::Publisher target_global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>(ns + "/mavros/setpoint_position/global", 1);
    ros::Publisher target_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(ns + "/mavros/setpoint_position/local", 1);
    ros::Publisher target_trajectory_pub = nh.advertise<mavros_msgs::Trajectory>(ns + "/mavros/trajectory/generated", 1);
    // ros::Publisher target_path_pub = nh.advertise<nav_msgs::Path>(ns+"/mavros/trajectory/path", 1);

    // 起始位置
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(ns + "/mavros/local_position/pose", 1, local_cb);
    // 获取home的UTM坐标系位置
    ros::Subscriber home_position_sub = nh.subscribe<nav_msgs::Odometry>("/UTM_position/home", 1, home_UTM_cb);

    // 设置模式、参数和home
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(ns + "/mavros/set_mode");
    ros::ServiceClient set_param_client = nh.serviceClient<mavros_msgs::ParamSet>(ns + "/mavros/param/set");
    // ros::ServiceClient set_home_client = nh.serviceClient<mavros_msgs::CommandHome>(ns + "/mavros/cmd/set_home");

    // 清除、设置航线
    ros::ServiceClient clear_waypoint_client = nh.serviceClient<mavros_msgs::WaypointClear>(ns + "/mavros/mission/clear");
    ros::ServiceClient push_waypoint_client = nh.serviceClient<mavros_msgs::WaypointPush>(ns + "/mavros/mission/push");

    // 获取飞行器状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(ns + "/mavros/state", 1, state_cb);

    // 订阅规划路径
    ros::Subscriber path_pub_ = nh.subscribe<nav_msgs::Path>("/run_hybrid_astar/searched_path", 1, searched_path_cb);
    // ros::Subscriber searched_tree_pub_ = nh.subscribe<visualization_msgs::Marker>("searched_tree", 1,);
    ros::Subscriber vehicle_path_pub_ = nh.subscribe<visualization_msgs::MarkerArray>("/run_hybrid_astar/vehicle_path", 1, vehicle_path_cb);

    std::cout << "生成订阅对象" << std::endl;

    // 跟踪
    ros::Rate rate(5);

    // 设置home点位置
    /*
    mavros_msgs::CommandHome set_home;
    set_home.request.altitude = 0;
    set_home.request.latitude = 39.54239;
    set_home.request.longitude = 117.38500;
    set_home.request.yaw = 0;
    set_home.request.current_gps = false;
    while (true)
    {
        if (set_home_client.call(set_home) && set_home.response.success)
        {
            std::cout << "set home to latitude:" << set_home.request.latitude << " longitude:" << set_home.request.longitude << std::endl;
            break;
        }
        rate.sleep();
    }
    */

    double lat,lon;
    gps_common::UTMtoLL(4377268.7674, 533393.8341, "50S", lat, lon);
    std::cout <<"lat:" << std::fixed << std::setprecision(5) <<lat <<"lon:"<< lon<< std::endl;

    if (firmware == "px4")
    {
        // 参考https://docs.px4.io/main/en/computer_vision/path_planning_interface.html
        /*
        mavros_msgs::ParamSet set_param;
        set_param.request.param_id = "COM_OBS_AVOID";
        set_param.request.value.integer = 0;
        if (set_param_client.call(set_param) && set_param.response.success)
            std::cout << "set COM_OBS_AVOID to 0" << std::endl;
        else
            std::cout << "set COM_OBS_AVOID to 0 failed" << std::endl;

        target_pos_local.header.frame_id = "map";
        target_pos_global.pose.position.altitude = 100;
        */

        while (ros::ok())
        {
            if (!vehicle_array.markers.size())
            {
                rate.sleep();
                ros::spinOnce();
                continue;
            }
            std::cout << "收到路径" << std::endl;

            // 单点跟踪
            /*
            进入OFFBOARD模式
            mavros_msgs::SetMode offb_set_mode;
            if (current_state.mode != "OFFBOARD")
            {
                offb_set_mode.request.custom_mode = "OFFBOARD";
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    std::cout << "Offboard enabled" << std::endl;
                }
            }

            发送航点
            for (unsigned int i = 0; i < vehicle_array.markers.size(); i++)
            {
                // UTM坐标转GPS
                auto pose = vehicle_array.markers[i];

                target_pos_local.pose.position.x = pose.pose.position.x;
                // target_pos_local.pose.position.x = 150;
                target_pos_local.pose.position.y = pose.pose.position.y;
                // target_pos_local.pose.position.y = 150;

                gps_common::UTMtoLL(target_pos_local.pose.position.y + 4377582.43347, target_pos_local.pose.position.x + 533825.373381, "50S", target_pos_global.pose.position.latitude, target_pos_global.pose.position.longitude);

                while (pow(local_position.pose.position.x - target_pos_local.pose.position.x, 2) + pow(local_position.pose.position.y - target_pos_local.pose.position.y, 2) > pow(15, 2))
                {
                    target_global_pos_pub.publish(target_pos_global);
                    target_local_pos_pub.publish(target_pos_local);
                    // target_global_to_local_pos_pub.publish(target_pos_global);

                    rate.sleep();
                    ros::spinOnce();
                }
            }

            退出OFFBOARD模式
            offb_set_mode.request.custom_mode = "AUTO.LOITER";
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                std::cout << "AUTO.LOITER enabled" << std::endl;
            }

            target_nav_path = nav_path;
            while(true){
                target_path_pub.publish(target_nav_path);
                rate.sleep();
            }
            */

            // 轨迹跟踪  参考https://docs.px4.io/main/en/computer_vision/path_planning_interface.html
            // 没搞懂怎么用
            /*
            // COM_OBS_AVOID设为1
            set_param.request.param_id = "COM_OBS_AVOID";
            set_param.request.value.integer = 1;
            if (set_param_client.call(set_param) && set_param.response.success)
                std::cout << "set COM_OBS_AVOID to 1" << std::endl;
            else
                std::cout << "set COM_OBS_AVOID to 1 failed" << std::endl;

            // 发送轨迹
            while (follow_trajectory(target_trajectory_pub))
            {
                ros::spinOnce();
                rate.sleep();
            }

            // COM_OBS_AVOID设为0
            set_param.request.param_id = "COM_OBS_AVOID";
            set_param.request.value.integer = 0;
            if (set_param_client.call(set_param) && set_param.response.success)
                std::cout << "set COM_OBS_AVOID to 0" << std::endl;
            else
                std::cout << "set COM_OBS_AVOID to 0 failed" << std::endl;

            vehicle_array.markers.clear();
            */

            mavros_msgs::SetMode offb_set_mode;

            // 发布WaypointList
            ros::spinOnce();
            mavros_msgs::WaypointPush push_waypoint;
            push_waypoint.request.start_index = 0;
            mavros_msgs::Waypoint waypoint;
            waypoint.frame = mavros_msgs::Waypoint::FRAME_GLOBAL;
            waypoint.command = 16;
            waypoint.is_current = false;
            waypoint.autocontinue = true;
            waypoint.param1 = 0;
            waypoint.param2 = 5;
            waypoint.param3 = 0;
            // waypoint.param4 = 0;
            waypoint.z_alt = 30;
            int x_offset = home_UTM.pose.pose.position.x - 533081.8341;
            int y_offset = home_UTM.pose.pose.position.y - 4377039.7674;

            std::cout << vehicle_array.markers.size() << std::endl;
            int closet_i = 0;
            for (int i = 0; i < vehicle_array.markers.size(); i++)
            {

                auto marker = vehicle_array.markers[i];
                auto closet_marker = vehicle_array.markers[closet_i];
                // 找到距飞机最近的航点

                if (pow(local_position.pose.position.x + x_offset - marker.pose.position.x, 2) + pow(local_position.pose.position.y + y_offset - marker.pose.position.y, 2) < pow(local_position.pose.position.x + x_offset - closet_marker.pose.position.x, 2) + pow(local_position.pose.position.y + y_offset - closet_marker.pose.position.y, 2))
                    closet_i = i;
                // 计算通过航点的角度  单位deg
                if (i + 1 < vehicle_array.markers.size())
                {
                    auto next_marker = vehicle_array.markers[i + 1];
                    waypoint.param4 = atan2(next_marker.pose.position.y - marker.pose.position.y, next_marker.pose.position.x - marker.pose.position.x) / M_PI * 180;
                }
                // UTM坐标转GPS
                gps_common::UTMtoLL(marker.pose.position.y + 4377039.7674, marker.pose.position.x + 533081.8341, "50S", waypoint.x_lat, waypoint.y_long);

                push_waypoint.request.waypoints.push_back(waypoint);
            }

            // 在最后一个航点loiter
            push_waypoint.request.waypoints.back().command = 17;
            push_waypoint.request.waypoints.back().param3 = 20;
            // push_waypoint.request.waypoints.back().is_current = true;

            // hybrid A*搜索是个耗时过程，擦除在这个过程中已经飞过的航点
            // set_current可以一试
            std::cout << closet_i << std::endl;
            push_waypoint.request.waypoints[closet_i+1].is_current = true;
            std::cout << "push_waypoint ready" << std::endl;

            // 发送WaypointList
            while (ros::ok())
            {
                if (push_waypoint_client.call(push_waypoint) && push_waypoint.response.success)
                {
                    std::cout << "update waypoints success" << std::endl;
                    break;
                }
                std::cout << "update waypoints failed" << std::endl;
                rate.sleep();
            }

            // 进入AUTO.MISSION模式
            // mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "AUTO.MISSION";
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                std::cout << "AUTO.MISSION enabled" << std::endl;
            }

            // 清空vehicle_array
            vehicle_array.markers.clear();
        }
    }
    else if (firmware == "apm")
    {
        while (ros::ok())
        {
            if (!vehicle_array.markers.size())
            {
                rate.sleep();
                ros::spinOnce();
                continue;
            }
            std::cout << "收到路径" << std::endl;

            // 路径跟踪策略

            // 单点跟踪
            /*
            // 进入GUIDED模式
            mavros_msgs::SetMode offb_set_mode;
            if (current_state.mode != "GUIDED")
            {
                offb_set_mode.request.custom_mode = "GUIDED";
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    std::cout << "GUIDED enabled" << std::endl;
                }
            }

            target_pos_local.header.frame_id = "map";
            target_pos_global.pose.position.altitude = 100;

            // 发送航点
            for (unsigned int i = 0; i < vehicle_array.markers.size(); i++)
            {
                std::cout<<vehicle_array.markers.size()<<std::endl;
                auto pose = vehicle_array.markers[i];

                target_pos_local.pose.position.x = pose.pose.position.x;
                target_pos_local.pose.position.y = pose.pose.position.y;

                // UTM坐标转GPS
                // gps_common::UTMtoLL(target_pos_local.pose.position.y + 4377582.43347, target_pos_local.pose.position.x + 533825.373381, "50S", target_pos_global.pose.position.latitude, target_pos_global.pose.position.longitude);

                while (pow(local_position.pose.position.x - target_pos_local.pose.position.x, 2) + pow(local_position.pose.position.y - target_pos_local.pose.position.y, 2) > pow(15, 2))
                {
                    // target_global_pos_pub.publish(target_pos_global);
                    target_local_pos_pub.publish(target_pos_local);
                    // target_global_to_local_pos_pub.publish(target_pos_global);
                    std::cout<<"发送航点:"<<i<<std::endl;
                    rate.sleep();
                    ros::spinOnce();
                }
            }
            std::cout<<"退出"<<std::endl;

            // 退出GUIDED模式
            offb_set_mode.request.custom_mode = "RTL";
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                std::cout << "RTL enabled" << std::endl;
            }

            vehicle_array.markers.clear();
            */

            // 上传航点
            // 进入STABILIZE模式
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "STABILIZE";
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                std::cout << "STABILIZE enabled" << std::endl;
            }

            // 清空WaypointList
            mavros_msgs::WaypointClear clear_waypoint;
            while (ros::ok())
            {
                if (clear_waypoint_client.call(clear_waypoint) && clear_waypoint.response.success)
                {
                    std::cout << "clear waypoints success" << std::endl;
                    break;
                }
                rate.sleep();
            }

            // 发布WaypointList
            ros::spinOnce();
            mavros_msgs::WaypointPush push_waypoint;
            push_waypoint.request.start_index = 0;
            mavros_msgs::Waypoint waypoint;
            waypoint.frame = mavros_msgs::Waypoint::FRAME_GLOBAL;
            waypoint.command = 16;
            waypoint.is_current = false;
            waypoint.autocontinue = true;
            waypoint.param1 = 0;
            waypoint.param2 = 5;
            waypoint.param3 = 0;
            // waypoint.param4 = 0;
            waypoint.z_alt = 20;
            int x_offset = home_UTM.pose.pose.position.x - 533081.8341;
            int y_offset = home_UTM.pose.pose.position.y - 4377039.7674;

            std::cout << vehicle_array.markers.size() << std::endl;
            int closet_i = 0;
            for (int i = 0; i < vehicle_array.markers.size(); i++)
            {

                auto marker = vehicle_array.markers[i];
                auto closet_marker = vehicle_array.markers[closet_i];
                // 找到距飞机最近的航点

                if (pow(local_position.pose.position.x + x_offset - marker.pose.position.x, 2) + pow(local_position.pose.position.y + y_offset - marker.pose.position.y, 2) < pow(local_position.pose.position.x + x_offset - closet_marker.pose.position.x, 2) + pow(local_position.pose.position.y + y_offset - closet_marker.pose.position.y, 2))
                    closet_i = i;
                // 计算通过航点的角度  单位deg
                if (i + 1 < vehicle_array.markers.size())
                {
                    auto next_marker = vehicle_array.markers[i + 1];
                    waypoint.param4 = atan2(next_marker.pose.position.y - marker.pose.position.y, next_marker.pose.position.x - marker.pose.position.x) / M_PI * 180;
                }
                // UTM坐标转GPS
                gps_common::UTMtoLL(marker.pose.position.y + 4377039.7674, marker.pose.position.x + 533081.8341, "50S", waypoint.x_lat, waypoint.y_long);
                // waypoint.x_lat = marker.pose.position.x;
                // waypoint.y_long = marker.pose.position.y;
                push_waypoint.request.waypoints.push_back(waypoint);
            }

            // 在最后一个航点loiter
            push_waypoint.request.waypoints.back().command = 17;
            push_waypoint.request.waypoints.back().param3 = 20;

            // hybrid A*搜索是个耗时过程，擦除在这个过程中已经飞过的航点
            // set_current可以一试
            std::cout << closet_i << std::endl;
            push_waypoint.request.waypoints.erase(push_waypoint.request.waypoints.begin(), push_waypoint.request.waypoints.begin() + 1 + closet_i);

            std::cout << "push_waypoint ready" << std::endl;

            // 发送WaypointList
            while (ros::ok())
            {
                if (push_waypoint_client.call(push_waypoint) && push_waypoint.response.success)
                {
                    std::cout << "update waypoints success" << std::endl;
                    break;
                }
                std::cout << "update waypoints failed" << std::endl;
                rate.sleep();
            }

            // 进入AUTO模式
            // mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "AUTO";
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                std::cout << "AUTO enabled" << std::endl;
            }

            // 清空vehicle_array
            vehicle_array.markers.clear();
        }
    }

    return 0;
}

/*
bool follow_trajectory(ros::Publisher &target_trajectory_pub)
{
    int closest_index = 0;
    for (int i = 1; i < vehicle_array.markers.size(); i++)
    {
        if (pow(local_position.pose.position.x - vehicle_array.markers[i].pose.position.x, 2) + pow(local_position.pose.position.y - vehicle_array.markers[i].pose.position.y, 2) < pow(local_position.pose.position.x - vehicle_array.markers[closest_index].pose.position.x, 2) + pow(local_position.pose.position.y - vehicle_array.markers[closest_index].pose.position.y, 2))
            closest_index = i;
    }

    if (closest_index == vehicle_array.markers.size())
        return false;

    mavros_msgs::Trajectory trajectory;
    trajectory.type = mavros_msgs::Trajectory::MAV_TRAJECTORY_REPRESENTATION_BEZIER;

    std::function<void(mavros_msgs::PositionTarget &, geometry_msgs::Point &)> assign_point = [](mavros_msgs::PositionTarget &point, geometry_msgs::Point &path_point)
    {
        point.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        point.position = path_point;
        point.position.z = 50;
    };

    if (closest_index + 1 < vehicle_array.markers.size())
    {
        assign_point(trajectory.point_1, vehicle_array.markers[closest_index + 1].pose.position);
        trajectory.point_valid[0] = true;
    };
    if (closest_index + 2 < vehicle_array.markers.size())
    {
        assign_point(trajectory.point_2, vehicle_array.markers[closest_index + 2].pose.position);
        trajectory.point_valid[1] = true;
    };
    if (closest_index + 3 < vehicle_array.markers.size())
    {
        assign_point(trajectory.point_3, vehicle_array.markers[closest_index + 3].pose.position);
        trajectory.point_valid[2] = true;
    };
    if (closest_index + 4 < vehicle_array.markers.size())
    {
        assign_point(trajectory.point_4, vehicle_array.markers[closest_index + 4].pose.position);
        trajectory.point_valid[3] = true;
    };
    if (closest_index + 5 < vehicle_array.markers.size())
    {
        assign_point(trajectory.point_5, vehicle_array.markers[closest_index + 5].pose.position);
        trajectory.point_valid[4] = true;
    };

    target_trajectory_pub.publish(trajectory);
    return true;
}
*/
