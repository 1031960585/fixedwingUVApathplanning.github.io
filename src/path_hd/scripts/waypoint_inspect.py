#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

# from mavros_msgs.msg import WaypointList
# from mavros_msgs.msg import Waypoint

from mavros_msgs.srv import WaypointPull,WaypointPullRequest,WaypointPullResponse
from mavros_msgs.msg import WaypointList

def waypoints_callback(msg):
    print(msg)


if __name__ == '__main__':
    rospy.init_node('waypoint_inspect')

    rospy.wait_for_service("/mavros/mission/pull")
    pull_waypoint_client = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)

    rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoints_callback,queue_size=1)

    waypoints_request = WaypointPullRequest()
    waypoints = WaypointPullResponse()
    
    waypoints = pull_waypoint_client.call(waypoints_request)

    print(waypoints)

    rospy.spin()





# import rospy
# from geometry_msgs.msg import PoseStamped
# from mavros_msgs.msg import State
# from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# current_state = State()

# def state_cb(msg):
#     global current_state
#     current_state = msg


# if __name__ == "__main__":
#     rospy.init_node("offb_node_py")

#     state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

#     local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

#     rospy.wait_for_service("/mavros/cmd/arming")
#     arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

#     rospy.wait_for_service("/mavros/set_mode")
#     set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


#     # Setpoint publishing MUST be faster than 2Hz
#     rate = rospy.Rate(20)

#     # Wait for Flight Controller connection
#     while(not rospy.is_shutdown() and not current_state.connected):
#         rate.sleep()

#     pose = PoseStamped()

#     pose.pose.position.x = 0
#     pose.pose.position.y = 0
#     pose.pose.position.z = 2

#     # Send a few setpoints before starting
#     for i in range(100):   
#         if(rospy.is_shutdown()):
#             break

#         local_pos_pub.publish(pose)
#         rate.sleep()

#     offb_set_mode = SetModeRequest()
#     offb_set_mode.custom_mode = 'OFFBOARD'

#     arm_cmd = CommandBoolRequest()
#     arm_cmd.value = True

#     last_req = rospy.Time.now()

#     while(not rospy.is_shutdown()):
#         if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
#             if(set_mode_client.call(offb_set_mode).mode_sent == True):
#                 rospy.loginfo("OFFBOARD enabled")

#             last_req = rospy.Time.now()
#         else:
#             if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
#                 if(arming_client.call(arm_cmd).success == True):
#                     rospy.loginfo("Vehicle armed")

#                 last_req = rospy.Time.now()

#         local_pos_pub.publish(pose)

#         rate.sleep()
