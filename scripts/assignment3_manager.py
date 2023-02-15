#!/usr/bin/env python

import rospy
import random
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from MRS_236609.srv import ActionReq, ActionReqResponse
import yaml
import os
import dynamic_reconfigure.client

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

    def __init__(self, aff_cen, ws_actions, req_tasks):
        self.aff_cen = aff_cen
        self.possible_actions = ws_actions
        self.tasks = req_tasks
        self.mv_DWA_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS/")
        self.mv_DWA_client.update_configuration({"max_vel_x": 0.22})
        rospy.Service('/affordance_service', Trigger, self.handle_request)
        rospy.Service('/do_action', ActionReq, self.action_request)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.update_status)
        self.rviz_pub = RvizPublisher()
        self.cost_pub = rospy.Publisher('current_cost', String, queue_size=10)
        self.curr_pose = None
        self.curr_reward = 0
        self.last_action = None
        self.current_object = None
        self.busy = False
        self.time = 0
        self.done_actions = ''
        self.cmu = CostmapUpdater()
        for key, val in self.aff_cen.items():
            self.rviz_pub.update_and_publish_markers(1, 0, 0, val[0], val[1], int(key[2]))
        rospy.Timer(rospy.Duration(1), self.publish_cost)

    def check_and_update_reward(self):
        for key, val in self.tasks.items():
            curr_task = key.replace('->', '')
            if self.done_actions.endswith(curr_task):
                del self.tasks[key]
                self.curr_reward += val
                self.last_action = None
                self.busy = False
                self.done_actions = ''
                break

    def publish_cost(self, event):
        if self.curr_pose is None:
            return
        self.time += 1
        idx_x, idx_y = self.cmu.position_to_map(self.curr_pose)
        c = self.cmu.cost_map[int(idx_x)][int(idx_y)]
        message = String()
        message.data = 'cost: ' + str(c + self.time) + ', reward: ' + str(self.curr_reward)
        self.cost_pub.publish(message)

    def action_request(self, req):
        ws = req.workstation
        act = req.action
        res = ActionReqResponse()
        curr_ws = None
        for key, val in self.aff_cen.items():
            if point_in_square(self.curr_pose, val):
                curr_ws = key

        if curr_ws is None:
            res.success = False
            res.message = 'Not At a workstation'
            return res

        if curr_ws != ws:
            res.success = False
            res.message = "The current station is different"
            return res

        if act not in self.possible_actions[ws]:
            res.success = False
            res.message = 'Not a valid action for this station'
            return res

        if self.last_action is None:
            if curr_ws == ws:
                self.last_action = [ws, act]
                if act.startswith('ACT'):
                    self.done_actions += act
                if act.startswith('PU'):
                    self.mv_DWA_client.update_configuration({"max_vel_x": 0.15})
                    self.current_object = act[-1]
                res.success = True
                self.busy = True
                return res

        if act.startswith('PU') and self.current_object is not None:
            res.success = False
            res.message = "Trying picking another object while already loaded"
            return res

        if act.startswith('PL') and self.current_object is None:
            res.success = False
            res.message = 'Trying to place an object while nothing in your hold'
            return res

        if act.startswith('ACT') and self.current_object is not None:
            res.success = False
            res.message = 'Trying to act while holding an object'
            return res

        if self.busy and act.startswith('ACT') and not self.last_action[1].startswith('PL'):
            res.success = False
            res.message = 'Trying to cheat without moving an object'
            return res

        if self.last_action[0] == ws:
            if act.startswith('ACT') and self.last_action[1].startswith('ACT'):
                res.success = False
                res.message = 'Too many actions at the same workstation'
                return res
            if act.startswith('PL') and self.last_action[1].startswith('PU'):
                res.success = False
                res.message = 'Cannot place an object at the same station without doing some action with it'
                return res

            self.last_action = [ws, act]
            res.success = True
            if act.startswith('PU'):
                self.mv_DWA_client.update_configuration({"max_vel_x": 0.15})
                self.current_object = act[-1]
            if act.startswith('ACT'):
                self.done_actions += act
                self.check_and_update_reward()
            return res

        if act.startswith('PL') and self.last_action[1].startswith('PU'):
            ob = act[-1]
            if self.last_action[1].endswith(ob):
                self.last_action = [ws, act]
                res.success = True
                self.mv_DWA_client.update_configuration({"max_vel_x": 0.22})
                self.current_object = None
                return res
            res.success = False
            res.message = 'Trying to place an unmatched object'
            return res

        self.last_action = [ws, act]
        res.success = True
        if act.startswith('PU'):
            self.mv_DWA_client.update_configuration({"max_vel_x": 0.1})
        if act.startswith('ACT'):
            self.done_actions += act
        self.check_and_update_reward()
        return res

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
                # Uncomment for debugging
                # new_msg = 'turtlebot currently in workstation: ' + key
                # print(new_msg)
                self.rviz_pub.update_and_publish_markers(0, 1., 0, val[0], val[1], int(key[2]))
            else:
                self.rviz_pub.update_and_publish_markers(1., 0, 0, val[0], val[1], int(key[2]))


if __name__ == '__main__':
    rospy.init_node('assignment3_manger')

    gcm_client = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")
    gcm_client.update_configuration({"inflation_radius": 0.2})
    lcm_client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
    lcm_client.update_configuration({"inflation_radius": 0.2})

    ws_file = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/config/workstations_config.yaml"
    tasks_file = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/config/tasks_config.yaml"

    with open(tasks_file, 'r') as f:
        data = yaml.load(f)
        tasks = data['tasks']

    act = ActionReq()

    locs = {}
    actions = {}
    with open(ws_file, 'r') as f:
        data = yaml.load(f)
        num_ws = data['num_ws']
        for i in range(num_ws):
            locs['ws' + str(i)] = data['ws' + str(i)]['location']
            actions['ws' + str(i)] = data['ws' + str(i)]['tasks']

    moves = [[-CUBE_EDGE, 0], [0, -CUBE_EDGE], [CUBE_EDGE, 0], [0, CUBE_EDGE]]
    aff_cen_list = {}
    for key, val in locs.items():
        move = random.choice(moves)
        aff_cen_list[key] = [val[0] + move[0], val[1] + move[1]]

    aff_pub = AffordanceServ(aff_cen_list, actions, tasks)
    rospy.spin()
