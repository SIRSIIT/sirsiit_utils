#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
rospy.init_node('tf_simple')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

def update_poses(data):
    #print(data)
    pose_in=tf2_ros.convert(data,tf2_geometry_msgs.PoseStamped);
    pose_new=tfBuffer.transform(data,'ati_base_measurement',timeout=rospy.Duration(0.1))
    print(pose_new)
    #tfBuffer.lookup_transform('ati_base_measurement','base_link',data.header.stamp,timeout=rospy.Duration(0.1))
    #tf2_ros.convert(data,)

sub=rospy.Subscriber('/ur5/ee_pose', PoseStamped, update_poses)
while not rospy.is_shutdown():
    rospy.spin()
