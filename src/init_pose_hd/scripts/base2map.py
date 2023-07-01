#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 从gazebo中获取模型的真实位姿,发布base_link到map的坐标转换
import rospy
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
import tf2_ros

# vehicle_type = sys.argv[1]


#         4-1.创建 TF 广播器
broadcaster = tf2_ros.TransformBroadcaster()
#         4-2.创建 广播的数据(通过 pose 设置)
tfs = TransformStamped()
tfs.header.frame_id = "map"
tfs.child_frame_id = "base_link"

def gazebo_model_state_callback(msg):
    #     4.回调函数处理
    #         4-1.创建 TF 广播器
    #         4-2.创建 广播的数据(通过 pose 设置)
    #         4-3.广播器发布数据
    tfs.header.stamp = rospy.Time.now()

    id = msg.name.index('plane_0')

    # print(type(msg.pose[id]))
    tfs.transform.translation.x = msg.pose[id].position.x+379
    tfs.transform.translation.y = msg.pose[id].position.y+278
    tfs.transform.translation.z = msg.pose[id].position.z
    tfs.transform.translation.z = 0

    # multi_speed.vector = msg.twist[id]

    # qtn = tf.transformations.quaternion_from_euler(0,0,0)
    tfs.transform.rotation.x = msg.pose[id].orientation.x
    tfs.transform.rotation.y = msg.pose[id].orientation.y
    tfs.transform.rotation.z = msg.pose[id].orientation.z
    tfs.transform.rotation.w = msg.pose[id].orientation.w

    # 4-3.广播器发布数据
    broadcaster.sendTransform(tfs)


if __name__ == '__main__':
    rospy.init_node('base_2_map')
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)

    #     5.spin
    rospy.spin()
