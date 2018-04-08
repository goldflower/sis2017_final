#!/usr/bin/env/python
# -*- coding: utf-8 -*-

import sys

import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3

def get_empty_map():
    empty_map = OccupancyGrid()
    empty_map.info.resolution = 0.05 
    empty_map.info.width = 4000
    empty_map.info.height = 4000
    empty_map.info.origin.position.x = -100
    empty_map.info.origin.position.y = -100
    empty_map.info.origin.position.z = 0
    empty_map.info.origin.orientation.x = 0
    empty_map.info.origin.orientation.y = 0
    empty_map.info.origin.orientation.z = 0
    empty_map.info.origin.orientation.w = 0
    empty_map.data = [-1] * (empty_map.info.width * empty_map.info.height)

    return empty_map
    
if __name__ == '__main__':
    rospy.init_node('clear_map_node', anonymous=True)
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10, latch=True)
    empty_map = get_empty_map()
    map_pub.publish(empty_map)
