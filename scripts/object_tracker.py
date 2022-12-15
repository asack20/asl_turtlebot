#!/usr/bin/env python3

from enum import Enum

import rospy
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, FoundObject
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Point
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np
from visualization_msgs.msg import Marker
from utils import wrapToPi
from copy import deepcopy

KNOWN_OBJECT_TAGS = ['fire_hydrant', 'chair', 'person', 'cow', 'bicycle', 'banana']

class ObjectTracker:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('object_tracker', anonymous=True)
        self.pose = Pose2D()
        self.found_objects = {}
        self.found_stop_signs = []
        self.rescue_list = []
        self.next_marker_id = 0
        self.project_phase = rospy.get_param("/project_phase", "EXPLORE")

        self.home_pose = Pose2D(x=3.15, y=1.6, theta=0)
        self.home_found_object = FoundObject(name='home', robot_pose=self.home_pose)

        # Stop sign Marker setup
        # add a marker
        self.ss_m = Marker()

        self.ss_m.header.frame_id = "map"
        self.ss_m.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        self.ss_m.id = 0
        self.ss_m.type = Marker.SPHERE_LIST # Sphere List
        self.ss_m.action = Marker.ADD

        self.ss_m.scale.x = 0.2
        self.ss_m.scale.y = 0.2
        self.ss_m.scale.z = 0.2

        self.ss_m.color.a = 1.0 # Don't forget to set the alpha!
        self.ss_m.color.r = 1.0
        self.ss_m.color.g = 0.0
        self.ss_m.color.b = 0.0

        self.ss_m.pose.position.x = 0
        self.ss_m.pose.position.y = 0
        self.ss_m.pose.position.z = 0

        self.ss_m.pose.orientation.x = 0.0
        self.ss_m.pose.orientation.y = 0.0
        self.ss_m.pose.orientation.z = 0.0
        self.ss_m.pose.orientation.w = 1


        ########## PUBLISHERS ##########

        self.trans_listener = tf.TransformListener()

        self.stop_sign_marker_pub = rospy.Publisher('/marker/stop_signs', Marker, queue_size=10)
        self.object_marker_pub = rospy.Publisher('/marker/objects', Marker, queue_size=10)
        self.object_rescue_pub = rospy.Publisher('/tracker/next_obj', FoundObject, queue_size=10)
        self.found_objects_pub = rospy.Publisher('/tracker/found_objects', String, queue_size=10)
        self.emote_pub = rospy.Publisher('/emotes', String, queue_size=10)

        ########## SUBSCRIBERS ##########
        

        # general detector
        #rospy.Subscriber("/detector/objects", DetectedObjectList, self.object_detected_callback)
        # specific ones
        rospy.Subscriber("/detector/stop_sign", DetectedObject, self.stop_sign_callback)
        rospy.Subscriber("/detector/fire_hydrant", DetectedObject, self.fire_hydrant_callback)
        rospy.Subscriber("/detector/chair", DetectedObject, self.chair_callback)
        rospy.Subscriber("/detector/person", DetectedObject, self.person_callback)
        rospy.Subscriber("/detector/bicycle", DetectedObject, self.bicycle_callback)
        rospy.Subscriber("/detector/banana", DetectedObject, self.banana_callback)
        rospy.Subscriber("/detector/cow", DetectedObject, self.cow_callback)

        rospy.Subscriber("/rescuer/obj_collected_name", String, self.collect_object_callback)

        rospy.Subscriber('/rescue_list', String, self.rescue_list_callback)


    ########## SUBSCRIBER CALLBACKS ##########

    def object_detected_callback(self, msg):
        #print("Generalized Object Found")
        #print(msg)
        #rospy.loginfo("OBJ Tracker: Generalized Object Found\r\n")
        pass

    def stop_sign_callback(self, msg):
        new_coords = self.calc_object_coords(msg.distance, msg.thetaleft, msg.thetaright)
        for coords in self.found_stop_signs:
            if np.sqrt((coords[0] - new_coords[0])**2 + (coords[1] - new_coords[1])**2) < 1:
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

    def fire_hydrant_callback(self, msg):
        self.emote_pub.publish("Splash")
        self.update_found_object(msg)

    def chair_callback(self, msg):
        self.emote_pub.publish("Take a seat")
        self.update_found_object(msg)

    def person_callback(self, msg):
        self.emote_pub.publish("Doh!")
        self.update_found_object(msg)
    
    def bicycle_callback(self, msg):
        self.emote_pub.publish("On your left")
        self.update_found_object(msg)
    
    def banana_callback(self, msg):
        self.emote_pub.publish("Watch your step")
        self.update_found_object(msg)
    
    def cow_callback(self, msg):
        self.emote_pub.publish("Moooooo")
        self.update_found_object(msg)

    def rescue_list_callback(self, msg):
        name = msg.data
        if name in self.found_objects.keys():
            self.rescue_list.append(name)
            print(self.rescue_list)

        ############ Code ends here ############

    def calc_object_coords(self, distance, thetaleft, thetaright):
        theta2 = (wrapToPi(thetaleft) + wrapToPi(thetaright)) / 2
        x_obj = self.pose.x + distance * np.cos(self.pose.theta + theta2)
        y_obj = self.pose.y + distance * np.sin(self.pose.theta + theta2)
        return (x_obj, y_obj)

    def update_found_object(self, obj_msg):
        if self.project_phase == 'EXPLORE':
            coords = self.calc_object_coords(obj_msg.distance, obj_msg.thetaleft, obj_msg.thetaright)
            if obj_msg.name in self.found_objects.keys():
                # update current position
                self.found_objects[obj_msg.name].xpos = coords[0]
                self.found_objects[obj_msg.name].ypos = coords[1]
                self.found_objects[obj_msg.name].marker.pose.position.x = coords[0]
                self.found_objects[obj_msg.name].marker.pose.position.y = coords[1]

                #update robot pose if closer
                if obj_msg.distance < self.found_objects[obj_msg.name].distance:
                    self.found_objects[obj_msg.name].distance = obj_msg.distance
                    self.found_objects[obj_msg.name].robot_pose = deepcopy(self.pose)

            else: # create new entry
                rospy.loginfo("New {} Found at x: {}\ty: {}\r\n".format(obj_msg.name, coords[0], coords[1]))
                self.found_objects_pub.publish(obj_msg.name)
                # create marker
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = rospy.Time()

                # IMPORTANT: If you're creating multiple markers, 
                #            each need to have a separate marker ID.
                m.id = self.next_marker_id
                self.next_marker_id += 1
                m.type = Marker.SPHERE # Sphere List
                m.action = Marker.ADD

                m.pose.position.x = coords[0]
                m.pose.position.y = coords[1]
                m.pose.position.z = 0

                m.pose.orientation.x = 0.0
                m.pose.orientation.y = 0.0
                m.pose.orientation.z = 0.0
                m.pose.orientation.w = 1

                m.scale.x = 0.2
                m.scale.y = 0.2
                m.scale.z = 0.2

                m.color.a = 1.0 # Don't forget to set the alpha!
                m.color.r = 1.0
                m.color.g = 1.0
                m.color.b = 0.0

                self.found_objects[obj_msg.name] = FoundObject(name=obj_msg.name, xpos=coords[0], ypos=coords[1], is_collected=False, marker=m, robot_pose=deepcopy(self.pose), distance=obj_msg.distance)
                
            self.object_marker_pub.publish(self.found_objects[obj_msg.name].marker)

    def collect_object_callback(self, msg):
        name = msg.data
        if self.project_phase == "RESCUE":
            if name in self.found_objects.keys():
                rospy.loginfo("Obj Tracker COLLECTED %s\r\n", name)
                self.found_objects[name].is_collected = True
                self.found_objects[name].marker.color.r = 0.0 # make green
                self.object_marker_pub.publish(self.found_objects[name].marker)
                
                # remove from dictionary 
                self.found_objects.pop(name)
                self.publish_next_object()
            else:
                rospy.loginfo("Obj Tracker tried to collect %s, which does not exist\r\n", name)
        else:
            rospy.loginfo("Error: Object Tracker cannot collect object when not in RESCUE phase. Tried to collect {}\r\n", name)

    def publish_next_object(self):
        if self.project_phase == "RESCUE":
            print(list(self.found_objects.keys()))
            if len(list(self.found_objects.keys())) > 0:
                to_pub = self.found_objects[list(self.found_objects.keys())[0]]
            else:
                to_pub = self.home_found_object

            rospy.loginfo("Tracker publishing %s to /tracker/next_obj\r\n", to_pub.name)
            self.object_rescue_pub.publish(to_pub)
        else:
            rospy.loginfo("Error: Object Tracker cannot publish object when not in RESCUE phase\r\n")


    def loop(self):
        last_phase = self.project_phase
        self.project_phase = rospy.get_param("/project_phase", "EXPLORE")
        if (last_phase == 'EXPLORE') and (self.project_phase == 'RESCUE'):
            rospy.loginfo("Entering RESCUE from EXPLORE\r\n")
            # for key in self.found_objects.keys():
            #     print(key)
            #     if key not in self.rescue_list:
            #         self.found_objects.pop(key)
            #         #rospy.loginfo("Not collecting %s\r\n", key)
            rospy.loginfo("Will collect the following: \r\n")
            for key in self.found_objects.keys():
                rospy.loginfo("\t%s\r\n", key)
            self.publish_next_object()
        try:
            (translation, rotation) = self.trans_listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0)
            )
            self.pose.x = translation[0]
            self.pose.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.pose.theta = euler[2]
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
