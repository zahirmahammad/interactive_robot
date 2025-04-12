#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import random
from actionlib_msgs.msg import GoalStatusArray



class StrollMode:
    def __init__(self):
        rospy.init_node("stroll_mode")


        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber("/move_base/status/", GoalStatusArray, self.status_callback)
        self.status_dict = {
            0: "PENDING",
            1: "ACTIVE",
            2: "PREEMPTED",
            3: "SUCCEEDED",
            4: "ABORTED",
            5: "REJECTED",
            6: "PREEMPTING",
            7: "RECALLING",
            8: "RECALLED",
            9: "LOST"
        }
        rate = rospy.Rate(0.1)
        self.map_data = None
        self.status = None

        self.new_goal_status = False

    def status_callback(self, msg):
        self.status = msg.status_list[-1].status
    
    def map_callback(self, msg):
        self.map_data = msg

    def stroll(self):
        while not rospy.is_shutdown():
            if self.map_data:
                # Get map width and height
                width = self.map_data.info.width
                height = self.map_data.info.height

                resol = self.map_data.info.resolution

                origin = self.map_data.info.origin 

                curr_map = np.array(self.map_data.data).reshape((height, width))

                free_indices = np.transpose(np.where(curr_map == 0))
                
                while self.status is None or self.status != 1:
                    rospy.loginfo("Starting New Goal....")

                    rand_idx = random.choice(free_indices)

                    x_idx, y_idx = rand_idx

                    x = x_idx * resol + origin.position.x + resol/2
                    y = y_idx * resol + origin.position.y + resol/2

                    # rospy.loginfo(f"X : {x}")
                    # rospy.loginfo(f"Y : {y}")

                    # create a goal
                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.header.stamp = rospy.Time.now()

                    goal.pose.position.x = x
                    goal.pose.position.y = y
                    goal.pose.orientation.w = 1.0

                    self.goal_pub.publish(goal)
                    rospy.loginfo(f"Published goal: x={x:.2f}, y={y:.2f}")




if __name__ == '__main__':
    stroll = StrollMode()
    stroll.stroll()