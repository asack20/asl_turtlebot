#!/usr/bin/env python3

import rospy
import numpy as np
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from std_msgs.msg import String, Int16
# import numpy as np
import tf
from numpy import linalg
from enum import Enum, IntEnum
from asl_turtlebot.msg import FoundObject

class Mode(Enum):
    """State machine modes. Feel free to change."""
    EXPLORE = 0
    RESCUE = 1
    WAIT = 2

class NavMode(IntEnum):
    IDLE = 0
    ALIGN = 1
    TRACK = 2
    PARK = 3

class Rescuer:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_rescuer', anonymous=True)
        self.cmd_nav_pub = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.obj_collected_pub = rospy.Publisher('/rescuer/obj_collected_name', String, queue_size=10)
        self.task_completed_pub = rospy.Publisher('/rescuer/task_completed', String, queue_size=10)
        self.nav_mode_sub = rospy.Subscriber('/navigator/mode', Int16, self.nav_mode_callback) 
        self.next_obj_sub = rospy.Subscriber('/tracker/next_obj', FoundObject, self.next_obj_callback)
        # self.params = SupervisorParams(verbose=True)

        self.rob_obj_loc = Pose2D()
        self.obj_name = "initialized"

        self.task_complete = 'False'

        rospy.set_param("/project_phase", 'EXPLORE')

        self.phase = rospy.get_param("/project_phase")

        self.mode = Mode.EXPLORE
        self.prev_mode = None  # For printing purposes

        self.trans_listener = tf.TransformListener()

        self.rescue_started = False

    def next_obj_callback(self, msg):
        self.phase = rospy.get_param("/project_phase")
        if self.phase == 'EXPLORE':
            pass
        elif self.phase == 'RESCUE':
            if self.rescue_started == False:
                self.mode = Mode.RESCUE
                self.rescue_started = True
        else:
            print('No phase param set')
        
        rospy.loginfo("Next Obj Callback %s\r\n", msg.name)

        if self.mode == Mode.EXPLORE:
            rospy.loginfo("Rescuer received Obj Callback in EXPLORE. Ignoring \r\n")
        elif self.mode == Mode.RESCUE:
            self.rob_obj_loc = msg.robot_pose
            self.obj_name = msg.name
            #rospy.loginfo("Rescuer navigation to %s at x: {}, y: {}, th: {}\r\n", msg.name, msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta)
            self.cmd_nav_pub.publish(msg.robot_pose)
        elif self.mode == Mode.WAIT:
            rospy.loginfo("Rescuer received Obj Callback in WAIT. Ignoring \r\n")
        else:
            rospy.loginfo('No rescuer mode set\r\n')
    
    def nav_mode_callback(self, msg):
        self.nav_mode = msg.data

    def close_to_obj(self):
        dist_eps = 0.5
        dist = np.sqrt((self.x - self.rob_obj_loc.x)**2 + (self.y - self.rob_obj_loc.y)**2)
        # print(dist, '\r')
        # print(self.nav_mode, '\r')
        # print(self.nav_mode == NavMode.IDLE, '\r')
        # print(self.mode, '\r')
        # print(self.mode == Mode.RESCUE, '\r')
        #dist = linalg.norm(np.asarray([self.x, self.y]) - np.asarray([self.rob_obj_loc.x, self.rob_obj_loc.y]))
        return self.mode == Mode.RESCUE and \
               self.nav_mode == NavMode.IDLE and dist < dist_eps
    
    def init_wait(self):
        """ initiates waiting at rescued object """
        rospy.loginfo("Rescuer initing wait\r\n")
        self.wait_start = rospy.get_rostime()
        self.mode = Mode.WAIT

    def has_waited(self):
        """ checks if rescue maneuver is over """
        
        wait_time = 3.0
        return self.mode == Mode.WAIT and \
               ((rospy.get_rostime() - self.wait_start) > rospy.Duration.from_sec(wait_time))

    def switch_mode(self, new_mode):
        rospy.loginfo("Switching from %s -> %s\r\n", self.mode, new_mode)
        self.mode = new_mode

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        try:
            (translation, rotation) = self.trans_listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0)
                )
            self.x = translation[0]
            self.y = translation[1]
            # euler = tf.transformations.euler_from_quaternion(rotation)
            # self.theta = euler[2]
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.loginfo("Rescuer: waiting for state info\r\n")
            print(e)
            pass

        # Handle first switch to RESCUE mode when /project_phase is swapped
        self.phase = rospy.get_param("/project_phase")
        if self.phase == 'EXPLORE':
            pass
        elif self.phase == 'RESCUE':
            if self.rescue_started == False:
                self.mode = Mode.RESCUE
                self.rescue_started = True
            else:
                pass
        else:
            print('No phase param set\r\n')

        # logs the current mode
        if self.prev_mode != self.mode:
            rospy.loginfo("Current project mode: %s\r\n", self.mode)
            self.prev_mode = self.mode

        if self.mode == Mode.EXPLORE:
            pass
        elif self.mode == Mode.RESCUE:
            if self.close_to_obj():
                rospy.loginfo("RESCUER close to object. Init Wait\r\n")
                self.init_wait()
        elif self.mode == Mode.WAIT:
            # Wait (already does it? if not, maybe set control_mode to STOP)
            if self.has_waited():
                rospy.loginfo("RESCUER Done waiting\r\n")
                self.obj_collected_pub.publish(self.obj_name)
                if self.obj_name == 'home':
                    self.task_complete = 'True'
                else:
                    self.mode = Mode.RESCUE
        else:
            print('No rescuer mode set\r\n')
        

        self.task_completed_pub.publish(self.task_complete)


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    rescuer = Rescuer()
    rescuer.run()