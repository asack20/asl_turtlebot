#!/usr/bin/env python3

from enum import Enum

import rospy
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Point
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np
from visualization_msgs.msg import Marker

KNOWN_OBJECT_TAGS = ['stop_sign', 'fire_hydrant', 'chair', 'cup', 'tree']

class ObjectTracker:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('object_tracker', anonymous=True)
        self.x = 0
        self.y = 0
        self.theta = 0 
        self.found_objects = {}
        self.found_stop_signs = []

        self.trans_listener = tf.TransformListener()

        self.stop_sign_marker_pub = rospy.Publisher('/marker/stop_signs', Marker, queue_size=10)

        # add a marker
        self.ss_m = Marker()

        self.ss_m.header.frame_id = "odom"
        self.ss_m.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        self.ss_m.id = 0
        self.ss_m.type = Marker.SPHERE_LIST # Sphere List
        self.ss_m.action = Marker.ADD

        self.ss_m.scale.x = 0.25
        self.ss_m.scale.y = 0.25
        self.ss_m.scale.z = 0.25

        self.ss_m.color.a = 1.0 # Don't forget to set the alpha!
        self.ss_m.color.r = 1.0
        self.ss_m.color.g = 0.0
        self.ss_m.color.b = 0.0

        ########## PUBLISHERS ##########

        #
        self.found_objects_publisher = rospy.Publisher('/found_objects', String, queue_size=10)


        ########## SUBSCRIBERS ##########
        

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
        new_coords = self.calc_object_coords(msg.distance, msg.thetaleft, msg.thetaright)
        for coords in self.found_stop_signs:
            if np.sqrt((coords[0] - new_coords[0])**2 + (coords[1] - new_coords[1])**2) < 0.5:
                # repeat sighting
                return
        self.found_stop_signs.append(new_coords)
        rospy.loginfo("Stop Sign Found at x: {}\ty: {}\r\n".format(new_coords[0], new_coords[1]))

        p = Point()
        p.x = new_coords[0]
        p.y = new_coords[1]
        p.z = 0.1
        self.ss_m.points.append(p)

        self.stop_sign_marker_pub.publish(self.ss_m)

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