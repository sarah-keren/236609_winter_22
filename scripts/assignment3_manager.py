#!/usr/bin/env python

import rospy
import random
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import yaml
import os

from visualization_msgs.msg import Marker
from costmap_listen_and_update import CostmapUpdater

CUBE_EDGE = 0.5


def point_in_square(point, center):
    x, y = point
    cx, cy = center

    return cx - CUBE_EDGE <= x <= cx + CUBE_EDGE and cy - CUBE_EDGE <= y <= cy + CUBE_EDGE


class RvizPublisher:
    def __init__(self):
        topic = 'visualization_marker'
        self.publisher = rospy.Publisher(topic, Marker, queue_size=10)

    def update_and_publish_markers(self, color_r, color_g, color_b, p_x, p_y, id):
        marker = Marker()
        marker.id = id
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = CUBE_EDGE
        marker.scale.y = CUBE_EDGE
        marker.scale.z = 0.1
        marker.color.a = 0.5
        marker.color.r = color_r
        marker.color.g = color_g
        marker.color.b = color_b
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = p_x
        marker.pose.position.y = p_y
        marker.pose.position.z = 0

        self.publisher.publish(marker)


class AffordanceServ:

    def __init__(self, aff_cen):
        self.aff_cen = aff_cen
        rospy.Service('/affordance_service', Trigger, self.handle_request)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.update_status)
        self.rviz_pub = RvizPublisher()
        self.cost_pub = rospy.Publisher('current_cost', String, queue_size=10)
        self.curr_pose = None
        self.time = 0
        self.cmu = CostmapUpdater()
        for key, val in self.aff_cen.items():
            self.rviz_pub.update_and_publish_markers(1, 0, 0, val[0], val[1], int(key[2]))
        rospy.Timer(rospy.Duration(1), self.publish_cost)

    def publish_cost(self, event):
        if self.curr_pose is None:
            return
        self.time += 1
        idx_x, idx_y = self.cmu.position_to_map(self.curr_pose)
        c = self.cmu.cost_map[int(idx_x)][int(idx_y)]
        message = String()
        message.data = str(c + self.time)
        self.cost_pub.publish(message)

    def handle_request(self, req):
        # Do some processing here and return a response
        res = TriggerResponse()
        res.success = True
        res.message = ''
        for key, val in self.aff_cen.items():
            res.message += key + ' affordance center is at ' + str(val[0]) + ' ' + str(val[1]) + '\n'
        return res

    def update_status(self, msg):
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        self.curr_pose = np.array([pose_x, pose_y])
        point = (pose_x, pose_y)
        for key, val in self.aff_cen.items():
            if point_in_square(point, val):
                new_msg = 'turtlebot currently in workstation: ' + key
                print(new_msg)
                self.rviz_pub.update_and_publish_markers(0, 1., 0, val[0], val[1], int(key[2]))
            else:
                self.rviz_pub.update_and_publish_markers(1., 0, 0, val[0], val[1], int(key[2]))


if __name__ == '__main__':
    rospy.init_node('assignment3_manger')
    ws_file = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/config/workstations_config.yaml"
    locs = {}
    with open(ws_file, 'r') as f:
        data = yaml.load(f)
        num_ws = data['num_ws']
        for i in range(num_ws):
            locs['ws' + str(i)] = data['ws' + str(i)]['location']

    moves = [[-CUBE_EDGE, 0], [0, -CUBE_EDGE], [CUBE_EDGE, 0], [0, CUBE_EDGE]]
    aff_cen_list = {}
    for key, val in locs.items():
        move = random.choice(moves)
        aff_cen_list[key] = [val[0] + move[0], val[1] + move[1]]

    aff_pub = AffordanceServ(aff_cen_list)
    rospy.spin()
