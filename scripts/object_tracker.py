#!/usr/bin/env python3

from enum import Enum

import rospy
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np

KNOWN_OBJECT_TAGS = ['stop_sign', 'fire_hydrant', 'chair', 'cup', 'tree']

class ObjectTracker:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('object_tracker', anonymous=True)
        self.x = 0
        self.y = 0
        self.theta = 0 
        self.found_objects = {}

        ########## PUBLISHERS ##########

        #
        self.found_objects_publisher = rospy.Publisher('/found_objects', String, queue_size=10)


        ########## SUBSCRIBERS ##########
        self.trans_listener = tf.TransformListener()

        # general detector
        rospy.Subscriber("/detector/objects", DetectedObjectList, self.object_detected_callback)
        # specific ones
        rospy.Subscriber("/detector/stop_sign", DetectedObject, self.stop_sign_callback)
        #rospy.Subscriber("/detector/fire_hydrant", DetectedObject, self.fire_hydrant_callback)
       # rospy.Subscriber("/detector/chair", DetectedObject, self.chair_callback)


    ########## SUBSCRIBER CALLBACKS ##########

    def object_detected_callback(self, msg):
        print("Generalized Object Found")
        #print(msg)
        rospy.loginfo("OBJ Tracker: Generalized Object Found\r\n")

    def stop_sign_callback(self, msg):
        self.found_objects['stop_sign'] = self.calc_object_coords(msg.distance, msg.thetaleft, msg.thetaright)
        print("Stop Sign Found at x: {}\ty: {}\r\n".format(self.found_objects['stop_sign'][0], self.found_objects['stop_sign'][1]))
        rospy.loginfo("Stop Sign Found at x: {}\ty: {}\r\n".format(self.found_objects['stop_sign'][0], self.found_objects['stop_sign'][1]))

        ############ Code ends here ############

    def calc_object_coords(self, distance, thetaleft, thetaright):
        theta2 = (thetaleft + thetaright) / 2
        x_obj = self.x + distance * np.cos(self.theta + theta2)
        y_obj = self.y + distance * np.sin(self.theta + theta2)
        return (x_obj, y_obj)

    def loop(self):
        # try to get state information to update self.x, self.y, self.theta
        try:
            (translation, rotation) = self.trans_listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0)
            )
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.loginfo("Obj Tracker: waiting for state info\r\n")
            print(e)
            pass


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    obj_tracker = ObjectTracker()
    obj_tracker.run()
