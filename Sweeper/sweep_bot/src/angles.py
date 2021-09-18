#!/usr/bin/env python

import rospy
from rospy.core import is_shutdown
from sensor_msgs.msg import LaserScan
from sweep_bot.msg import Angle
from sweep_bot.msg import AngleArray
from sweep_bot.msg import AnglesArray
import numpy, math
import settings as CONFIG

class Angles :
    RANGE = CONFIG.RANGE

    def __init__(self) :
        rospy.loginfo('initiating angles publishing')
        self.scans = ()
        self.regions = {}
        self.openspace_index = 0
        self.publisher = self.subscriber = ''
        self.index = 0

    def callbackScans(self, data) :
        self.scans = data.ranges
        self.regions = {
            'starboard_bow' : self.scans[408:543],
            'starboard_abeam_bow' : self.scans[272:407],
            'starboard_abeam_aft' : self.scans[136:271],
            'starboard_aft' : self.scans[0:135],
            'port_bow' : self.scans[544:679],
            'port_abeam_bow' : self.scans[679:814],
            'port_abeam_aft' : self.scans[814:949],
            'port_aft' : self.scans[949:1084],
            'starboard_abeam' : self.scans[136:407],
            'port_abeam' : self.scans[679:949],
            'bow' : self.scans[408:679],
            'starboard' : self.scans[0:543],
        }

    def publish(self) :
        anglesArray = AnglesArray()

        for region in self.regions :

            angleArray = AngleArray()
            angleArray.region = region
            
            length_1 = self.regions[region][0]
            length_2 = self.regions[region][-1]

            hypo = math.sqrt(length_1**2 + length_2**2)
            alpha = 0 if self.regions[region][0] == CONFIG.RANGE else math.asin(length_1/hypo) * 180/math.pi
            delta = 0 if self.regions[region][-1] == CONFIG.RANGE else math.asin(length_2/hypo) * 180/math.pi

            angleArray.angles.insert(0, alpha )
            angleArray.angles.insert(1, delta )
            # if region == 'starboard_abeam' or region == 'port_abeam' or 'bow':
            if self.isclose(alpha, delta, 0.1) and alpha != 0 and delta != 0 :
                angleArray.parallel = True

            angleArray.mid_distance = self.regions[region][len(self.regions[region])/2]

            angleArray.avg_distance = self.avrg( self.regions[region] )

            anglesArray.angles.append(angleArray)
            
        # rospy.loginfo('------------------------')
        # rospy.loginfo(anglesArray)
        # rospy.loginfo('------------------------')
        publisher.publish( anglesArray )

    def isclose(self, a, b, rel_tol=1e-09, abs_tol=0.0):
        return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

    def clean_set(self, set) :
        set_list = list(set)
        occurances = set_list.count(CONFIG.RANGE)
        # print('Occurances: %d' %(occurances))
        while set_list.count(CONFIG.RANGE) :
            set_list.remove(CONFIG.RANGE)
        return tuple(set_list)

    def avrg(self, values): # where values is a list of all values
        return sum(self.clean_set(values)) / len(values)
    
if __name__ == '__main__' :
    rospy.init_node('sweeper_regions_angles')
    rate = rospy.Rate(10)
    angles = Angles()

    subscriber = rospy.Subscriber("scans", LaserScan, angles.callbackScans)
    rospy.sleep(1)
    publisher  = rospy.Publisher("sweepbot/regions_angles", AnglesArray, queue_size=10)

    while not rospy.is_shutdown() :
        angles.publish()
        rate.sleep()