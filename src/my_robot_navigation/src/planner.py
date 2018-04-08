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


class WayPointsNavigation():

    def __init__(self, way_points = None):
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10, latch=True)
        self.vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10, latch=True)
        rospy.init_node('way_points_navigation')
        rospy.on_shutdown(self.shutdown)
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo('Waiting for the move_base action server...')
        
        rospy.loginfo('The move_base action server is up!')
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        if way_points is None:
            self.way_points = [[3, -2,-pi], [ 2.0, 2.0, 0.0 * pi], [ 1.0, 1.5, 0.5 * pi],[ -2.5, 2.5,  pi], [0.0, 0.0, 0.0 * pi], [None, None, None]]        
        else:
            self.way_points = way_points
            self.way_points.append([None, None, None])
        self.publish_way_points()
        

           
    def publish_way_points(self):
        #way_points = [[3, -2, -pi], [None, None, None]]
        way_points = self.way_points
        i = 0
        #empty_map = self.get_empty_map()
        #self.map_pub.publish(empty_map)
        while not rospy.is_shutdown():
            if self.way_points[i][0] is None:
                break        
            #self.publish_marker(self.way_points)
            self.goal.target_pose.pose.position.x = way_points[i][0]
            self.goal.target_pose.pose.position.y = way_points[i][1]
            q = tf.transformations.quaternion_from_euler(0, 0, way_points[i][2])
            self.goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            rospy.loginfo('Sending goal: {}: ({}, {}, {})'.format(i+1, way_points[i][0], way_points[i][1], way_points[i][2]))
            
            #self.ac.send_goal(self.goal)
            
            success = self.ac.send_goal_and_wait(self.goal, rospy.Duration(600))
            state = self.ac.get_state();
            
            if success:
                rospy.loginfo('Navigate to way point {} succeeded'.format(i+1))
            else:
                rospy.loginfo('Navigate to way point {} failed'.format(i+1))
            
            i += 1
            
    def get_empty_map(self):
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
        
        
    def publish_marker(self, way_points):
        points = []
        for point in way_points:
            points.append([point[0], point[1], 0])
        self.vis_pub.publish(self.createPointMarker2(points[:-1], 1, [0.6, 0.6, 0, 1]))            
            
    def createPointMarker2(self, points, marker_id, rgba = None, pose=[0,0,0,0,0,0,1], frame_id = '/world'):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.POINTS
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.id = marker_id

        n = len(points)
        sub = 1
    
        if rgba is not None:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = tuple(rgba)

        for i in xrange(0,n,sub):
            p = Point()
            p.x = points[i][0]
            p.y = points[i][1]
            p.z = points[i][2]
            marker.points.append(p)


        if rgba is None:
            for i in xrange(0,n,sub):
                p = ColorRGBA()
                p.r = points[i][3]
                p.g = points[i][4]
                p.b = points[i][5]
                p.a = points[i][6]
                marker.colors.append(p)

        # marker.pose = poselist2pose(pose)

        return marker
            
    def shutdown(self):
        rospy.loginfo('The robot was terminated')
        self.ac.cancel_goal()
        
if __name__ == '__main__':
    try:
        if len(sys.argv) == 1:
            WayPointsNavigation()
        else:
            WayPointsNavigation([[float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]], )
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Way points navigation finished')
               
        
