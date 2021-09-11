#!/usr/bin/env python

import rospy
from rospy.core import is_shutdown
from sensor_msgs.msg import LaserScan
from sweep_bot.msg import Angle
from sweep_bot.msg import AngleArray
from sweep_bot.msg import AnglesArray
import numpy, math

class Angles :

    def __init__(self) :
        rospy.loginfo('initiating angles publishing')
        # rospy.Subscriber("scans", LaserScan, self.callbackScans)
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
        }

    def publish(self) :
        anglesArray = AnglesArray()

        for region in self.regions :

            angleArray = AngleArray()
            angleArray.region = region
            
            if self.regions[region][0] == 26.0 and self.regions[region][-1] == 26.0 :
                alpha = delta = 0
            else :
                length_1 = self.regions[region][0]
                length_2 = self.regions[region][-1]

                # rospy.loginfo("length_1: %s", length_1)
                # rospy.loginfo("length_2: %s", length_2)

                hypo = math.sqrt(length_1**2 + length_2**2)
                alpha = math.asin(length_1/hypo) * 180/math.pi
                delta = math.asin(length_2/hypo) * 180/math.pi
            
            angleArray.angle.insert(0, Angle( alpha ) )
            angleArray.angle.insert(1, Angle( delta ) )

            anglesArray.angles.append(angleArray)
            
        rospy.loginfo('------------------------')
        rospy.loginfo(anglesArray)
        rospy.loginfo('------------------------')
        # publisher.publish( anglesArray )
        # exit()


    def getOpenAirPort(self) :
        # 'port_aft' : self.scans[949:1084],
        # 'port_abeam_aft' : self.scans[814:949],
        # 'port_abeam_bow' : self.scans[679:814],
        # 'port_bow' : self.scans[544:679],

        arr = self.scans[544:1084]
        for index, value in enumerate(arr) :
            if value == 26.0 :
                j = index
                space = []
                for j in range(0, 5) :
                    try :
                        if arr[index + j] == 26.0 :
                            space.append(True)
                        else :
                            space.append(False)
                            j += 1
                    except Exception as e :
                        rospy.logerr('Exception in j-level space checking - Region: ')
                if space.count(True) == 5 and False not in space :
                    return index
        return False
    
    def getOpenAirStarboard(self) :
        # 'starboard_aft' : self.scans[0:135],
        # 'starboard_abeam_aft' : self.scans[136:271],
        # 'starboard_abeam_bow' : self.scans[272:407],
        # 'starboard_bow' : self.scans[408:543],

        arr = self.scans[0:543]
        for index, value in enumerate(arr) :
            if value == 26.0 :
                j = index
                space = []
                for j in range(0, 5) :
                    try :
                        if arr[index + j] == 26.0 :
                            space.append(True)
                        else :
                            space.append(False)
                            j += 1
                    except Exception as e :
                        rospy.logerr('Exception in j-level space checking - Region: ')
                if space.count(True) == 5 and False not in space :
                    return index
        return False
    
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