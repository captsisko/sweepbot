#!/usr/bin/env python

import rospy
from rospy.core import is_shutdown
from sensor_msgs.msg import LaserScan
from sweep_bot.msg import Space
from sweep_bot.msg import SpaceArray

class FreeSpace :

    def __init__(self) :
        rospy.loginfo('initiating space publishing')
        # rospy.Subscriber("scans", LaserScan, self.callbackScans)
        self.scans = ()
        self.regions = {}
        self.openspace_index = 0
        self.publisher = self.subscriber = ''
        
    def callbackScans(self, data) :
        self.scans = data.ranges
        self.regions = {
            'starboard_aft' : self.scans[0:135],
            'starboard_abeam_aft' : self.scans[136:271],
            'starboard_abeam_bow' : self.scans[272:407],
            'starboard_bow' : self.scans[408:543],
            'port' : {
                'bow' : self.scans[544:679],
                'abeam_bow' : self.scans[679:814],
                'abeam_aft' : self.scans[814:949],
                'aft' : self.scans[949:1084],
            }
        }

    def publish(self) :
        spacesArray = SpaceArray()
        
        port = Space()
        port.region = 'port'
        port.gap = str(self.getOpenAirPort())
        spacesArray.spaces.append(port)

        starboard = Space()
        starboard.region = 'starboard'
        starboard.gap = str(self.getOpenAirStarboard())
        spacesArray.spaces.append(starboard)

        publisher.publish( spacesArray )

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
    rospy.init_node('space_identifier')
    rate = rospy.Rate(10)
    spacer = FreeSpace()

    subscriber = rospy.Subscriber("scans", LaserScan, spacer.callbackScans)
    rospy.sleep(1)
    publisher  = rospy.Publisher("scans_freespace", SpaceArray, queue_size=10)

    while not rospy.is_shutdown() :
        spacer.publish()
        rate.sleep()