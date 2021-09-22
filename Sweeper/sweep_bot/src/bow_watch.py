#!/usr/bin/env python

import rospy
from rospy.core import is_shutdown
from sensor_msgs.msg import LaserScan
from sweep_bot.msg import Bow, BowGrid
import numpy, math
import settings as CONFIG

class BowWatch :
    RANGE = CONFIG.RANGE
    grid = []

    def __init__(self) :
        rospy.loginfo('initiating bow watch')
        # self.scans = ()
        self.regions = {}
        # self.openspace_index = 0
        # self.publisher = self.subscriber = ''
        # self.index = 0

    def callbackScans(self, data) :
        self.scans = data.ranges
        self.regions = {
            # 'starboard_bow' : self.scans[408:543],
            # 'starboard_abeam_bow' : self.scans[272:407],
            # 'starboard_abeam_aft' : self.scans[136:271],
            # 'starboard_aft' : self.scans[0:135],
            # 'port_bow' : self.scans[544:679],
            # 'port_abeam_bow' : self.scans[679:814],
            # 'port_abeam_aft' : self.scans[814:949],
            # 'port_aft' : self.scans[949:1084],
            # 'starboard_abeam' : self.scans[136:407],
            # 'port_abeam' : self.scans[679:949],
            'bow' : self.scans[408:679],
            # 'starboard' : self.scans[0:543],
        }

    def publish(self) :
        # rospy.loginfo( self.regions['bow'] )
        self.grid = BowGrid()
        for index, data in enumerate(self.regions['bow']) :
            if index % 3 == 0 :
                bow = Bow()
                bow.test = index
            #     # bow.obstructed.insert(index+45, False if self.regions['bow'][index] == 8.0 else True)
                bow.angle = (index/3) - 45
                bow.obstructed = False if data == 8.0 else True

                self.grid.Grid.insert(index, bow)
        publisher.publish( self.grid )

if __name__ == '__main__' :
    rospy.init_node('sweeper_bow_watch')
    rate = rospy.Rate(10)
    bow = BowWatch()

    subscriber = rospy.Subscriber("scans", LaserScan, bow.callbackScans)
    rospy.sleep(1)
    publisher  = rospy.Publisher("sweepbot/bow_watch", BowGrid, queue_size=10)

    while not rospy.is_shutdown() :
        bow.publish()
        rate.sleep()