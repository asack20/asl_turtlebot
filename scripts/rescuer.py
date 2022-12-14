#!/usr/bin/env python3

import rospy
import numpy as np
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from std_msgs.msg import String, Int16
import numpy as np
import tf
from numpy import linalg
from enum import Enum, IntEnum
from asl_turtlebot.msg import FoundObject

class Mode(Enum):
    """State machine modes. Feel free to change."""
    EXPLORE = 0
    RESCUE = 1

class NavMode(IntEnum):
    IDLE = 0
    ALIGN = 1
    TRACK = 2
    PARK = 3

class Rescuer:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_rescuer', anonymous=True)
        cmd_nav_pub = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        obj_collected_pub = rospy.Publisher('/rescuer/obj_collected_name', String, queue_size=10)
        task_completed_pub = rospy.Publisher('/rescuer/task_completed', String, queue_size=10)
        nav_mode_sub = rospy.Subscriber('/navigator/mode', Int16, self.nav_mode_callback) 
        next_obj_sub = rospy.Subscriber('/tracker/next_obj', FoundObject, self.next_obj_callback)
        # self.params = SupervisorParams(verbose=True)

        task_completed_pub.publish('False')

        phase = rospy.get_param("/project_phase", 'EXPLORE')

        self.mode = Mode.EXPLORE
        self.prev_mode = None  # For printing purposes

        self.trans_listener = tf.TransformListener()

        rescue_started = False

    def next_obj_callback(self, msg):
        rob_obj_loc = msg.robot_pose
        obj_name = msg.name
    
    def nav_mode_callback(self, msg):
        nav_mode = msg

    def close_to_obj(self):
        dist_eps = 1.0
        dist = linalg.norm(np.as_array([self.x, self.y]), np.as_array([rob_obj_loc.x, rob_obj_loc.y]))
        return self.mode == Mode.RESCUE and \
               nav_mode == NavMode.IDLE and dist < dist_eps
    
    def init_wait(self):
        """ initiates waiting at rescued object """

        self.wait_start = rospy.get_rostime()
        self.mode = Mode.WAIT

    def has_stopped(self):
        """ checks if rescue maneuver is over """
        
        wait_time = 3.0
        return self.mode == Mode.WAIT and \
               rospy.get_rostime() - self.wait_start > rospy.Duration.from_sec(wait_time)

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
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.loginfo("Rescuer: waiting for state info\r\n")
            print(e)
            pass

        phase = rospy.get_param("/project_phase", "EXPLORE")
        if phase == 'EXPLORE':
            pass
        elif phase == 'RESCUE' and rescue_started == False:
            self.mode = Mode.RESCUE
            rescue_started = True
        else:
            print('No phase param set')

        # logs the current mode
        if self.prev_mode != self.mode:
            rospy.loginfo("Current phase: %s\r\n", self.mode)
            self.prev_mode = self.mode


        if self.mode == Mode.EXPLORE:
            pass
        elif self.mode == Mode.RESCUE:
            cmd_nav_pub.publish(obj_loc)
            if self.close_to_obj():
                self.init_wait()
                if self.has_waited():
                    obj_collected_pub.publish(obj_name)
                    if obj_name == 'Home':
                        task_completed_pub.publish('True')
                    else:
                        self.mode = Mode.RESCUE

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    rescuer = Rescuer()
    rescuer.run()