#!/usr/bin/env python3

from enum import Enum

import rospy
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String
import tf

KNOWN_OBJECT_TAGS = ['stop_sign', 'fire_hydrant', 'chair', 'cup', 'tree']

class ObjectTracker:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('object_tracker', anonymous=True)

        found_objects = {}

        ########## PUBLISHERS ##########

        #
        self.found_objects_publisher = rospy.Publisher('/found_objects', String, queue_size=10)


        ########## SUBSCRIBERS ##########

        # Stop sign detector
        rospy.Subscriber("/detector/objects", DetectedObjectList, self.object_detected_callback)


    ########## SUBSCRIBER CALLBACKS ##########

    def object_detected_callback(self, msg):
        print("Object Found")
        rospy.loginfo("Object Found: {}", msg.id)

    

        ############ Code ends here ############

    def loop(self):
        print("obj tracker looping\r\n")

    def run(self):
        rate = rospy.Rate(0.1) # 1 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    obj_tracker = ObjectTracker()
    obj_tracker.run()
