#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


class CostmapUpdater:
    def __init__(self):
        self.number_of_update = 0
        self.cost_map = None
        self.initial_msg = None
        self.shape = None
        self.map_org = None
        self.resolution = None
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.init_costmap_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costmap_callback_update)

    def init_costmap_callback(self, msg):
        self.initial_msg = msg
        self.shape = msg.info.height, msg.info.width
        self.cost_map = np.array(msg.data).reshape(self.shape)
        self.map_org = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.resolution = msg.info.resolution

    def costmap_callback_update(self, msg):
        shape = msg.height, msg.width
        data = np.array(msg.data).reshape(shape)
        self.cost_map[msg.y:msg.y + shape[0], msg.x: msg.x + shape[1]] = data

    def show_map(self):
        if not self.cost_map is None:
            plt.imshow(self.cost_map)
            plt.show()

    def position_to_map(self, pos):
        return (pos - self.map_org) // self.resolution


if __name__ == '__main__':
    rospy.init_node('costmap_updater')
    cmu = CostmapUpdater()
    rospy.spin()
