#!/usr/bin/env python

import rospy
from rospy.core import is_shutdown
from sensor_msgs.msg import LaserScan
from sweep_bot.msg import Space
from sweep_bot.msg import SpaceArray

class FreeSpace :

    def __init__(self) :
        rospy.loginfo('starting')
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
            'port_aft' : self.scans[949:1084],
            'port_abeam_aft' : self.scans[814:949],
            'port_abeam_bow' : self.scans[679:814],
            'port_bow' : self.scans[544:679],
        }

    def publish(self) :
        spacesArray = SpaceArray()
        for region in self.regions :
            space = Space()
            space.region = region
            space.spaces = self.getOpenAir(region)[:-1]
            # space.space = str({region : self.getOpenAir(region)})
            spacesArray.spaces.append(space)
        publisher.publish( spacesArray )

    def getOpenAir(self, region) :
        # 'port_aft' : self.scans[949:1084],
        # 'port_abeam_aft' : self.scans[814:949],
        # 'port_abeam_bow' : self.scans[679:814],
        # 'port_bow' : self.scans[544:679],

        self.arr = self.regions[region]

        spaces = []
        spaces_list = ''

        for i in range( len(self.arr) ) :
            # rospy.loginfo("Comparing OpenSpace-Index: %s to Array Length: %s", i, len(self.arr)-1)
            # rospy.loginfo("%s ===>>> %s", i, self.arr[i])
            if self.arr[i] == 26.0 :
                match_index = i
                space = []
                for j in range(0, 5) :
                    try :
                        if self.arr[match_index + j] == 26.0 :
                            space.append(True)
                        else :
                            space.append(False)
                        j += 1
                    except IndexError :
                        # rospy.logerr('Out of range')
                        self.openspace_index = 0

                # rospy.loginfo(space)

                if space.count(True) == 5 and False not in space :
                    spaces_list += str(match_index) + ','

            self.openspace_index += 5

        # rospy.loginfo('Region ===>>> %s', region)
        # spaces_list = ''.join(spaces)

        return spaces_list
        
    
if __name__ == '__main__' :
    rospy.init_node('space_identifier')
    rate = rospy.Rate(10)
    spacer = FreeSpace()

    subscriber = rospy.Subscriber("scans", LaserScan, spacer.callbackScans)
    # rospy.wait_for_message('scans', LaserScan, )
    rospy.sleep(1)
    publisher  = rospy.Publisher("scans_freespace", SpaceArray, queue_size=10)

    while not rospy.is_shutdown() :
        # rospy.loginfo( spacer.regions )
        spacer.publish()
        rate.sleep()

    # try:
    #     spacer()
    # except rospy.ROSInterruptException:
    #     pass