#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker


def publisher():
    vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)
    rospy.init_node('marker_node', anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        marker1 = Marker()

        marker1.header.frame_id = "map"
        marker1.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        marker1.id = 0

        marker1.type = 0 # arrow

        marker1.pose.position.x = 0
        marker1.pose.position.y = 0
        marker1.pose.position.z = 0

        hFoV = 90*np.pi/180
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = np.sin(hFoV/2/2)
        marker1.pose.orientation.w = np.cos(hFoV/2/2)

        marker1.scale.x = 0.5
        marker1.scale.y = 0.01
        marker1.scale.z = 0.01

        marker1.color.a = 1.0 # Don't forget to set the alpha!
        marker1.color.r = 1.0
        marker1.color.g = 0.0
        marker1.color.b = 0.0
        
        marker2 = Marker()

        marker2.header.frame_id = "base_footprint"
        marker2.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        marker2.id = 1

        marker2.type = 0 # arrow

        marker2.pose.position.x = 0
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        hFoV = 120*np.pi/180
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = np.sin(-hFoV/2/2)
        marker2.pose.orientation.w = np.cos(-hFoV/2/2)

        marker2.scale.x = 0.5
        marker2.scale.y = 0.01
        marker2.scale.z = 0.01

        marker2.color.a = 1.0 # Don't forget to set the alpha!
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        
        vis_pub.publish(marker1)
        vis_pub.publish(marker2)
        #print('Published marker!')
        
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
