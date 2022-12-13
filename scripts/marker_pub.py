#!/usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker

def publisher():
    rospy.init_node('marker_node', anonymous=True)
    vis_pub = rospy.Publisher('marker_fov', Marker, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # marker1 = Marker()

        # marker1.header.frame_id = "base_footprint"
        # marker1.header.stamp = rospy.Time()

        # # IMPORTANT: If you're creating multiple markers, 
        # #            each need to have a separate marker ID.
        # marker1.id = 0

        # marker1.type = 0 # arrow

        # marker1.pose.position.x = 0
        # marker1.pose.position.y = 0
        # marker1.pose.position.z = 0

        # hFoV = 80*np.pi/180
        # marker1.pose.orientation.x = 0.0
        # marker1.pose.orientation.y = 0.0
        # marker1.pose.orientation.z = np.sin(hFoV/2/2)
        # marker1.pose.orientation.w = np.cos(hFoV/2/2)

        # marker1.scale.x = 0.5
        # marker1.scale.y = 0.01
        # marker1.scale.z = 0.01

        # marker1.color.a = 1.0 # Don't forget to set the alpha!
        # marker1.color.r = 1.0
        # marker1.color.g = 0.0
        # marker1.color.b = 0.0
        
        # marker2 = Marker()

        # marker2.header.frame_id = "base_footprint"
        # marker2.header.stamp = rospy.Time()

        # # IMPORTANT: If you're creating multiple markers, 
        # #            each need to have a separate marker ID.
        # marker2.id = 1

        # marker2.type = 0 # arrow

        # marker2.pose.position.x = 0
        # marker2.pose.position.y = 0
        # marker2.pose.position.z = 0

        # hFoV = 80*np.pi/180
        # marker2.pose.orientation.x = 0.0
        # marker2.pose.orientation.y = 0.0
        # marker2.pose.orientation.z = np.sin(-hFoV/2/2)
        # marker2.pose.orientation.w = np.cos(-hFoV/2/2)

        # marker2.scale.x = 0.5
        # marker2.scale.y = 0.01
        # marker2.scale.z = 0.01

        # marker2.color.a = 1.0 # Don't forget to set the alpha!
        # marker2.color.r = 1.0
        # marker2.color.g = 0.0
        # marker2.color.b = 0.0

        marker3 = Marker()

        marker3.header.frame_id = "base_footprint"
        marker3.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers,
        #            each need to have a separate marker ID.
        marker3.id = 2

        marker3.type = 10  # mesh
        marker3.mesh_resource = "package://asl_turtlebot/models/pyramid/pyramid.stl"

        marker3.pose.position.x = 0
        marker3.pose.position.y = 0
        marker3.pose.position.z = 0

        length = 0.025
        hFoV = 1.3962634 # from asl_turtlebot.gazebo.xacro
        marker3.scale.x = 2*length*np.tan(hFoV/2)
        marker3.scale.y = 2*length*np.tan(hFoV/2)
        marker3.scale.z = length

        marker3.pose.orientation.x = 0.0
        marker3.pose.orientation.y = np.sin(45)
        marker3.pose.orientation.z = 0.0
        marker3.pose.orientation.w = np.cos(45)

        marker3.color.a = 0.5  # Don't forget to set the alpha!
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        
        # vis_pub.publish(marker1)
        # vis_pub.publish(marker2)
        vis_pub.publish(marker3)
        # print('Published FoV marker!')
        
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
