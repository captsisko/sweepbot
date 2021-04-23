#!/usr/bin/env python

import rospy

def main():
    print "Testing!"

if __name__ == "__main__":
    rospy.init_node('test_test', anonymous=False)
    main()
    rospy.spin()

    # try:
    #     while not rospy.is_shutdown():
    #     # print "Testing ..."
    # except rospy.ROSInterruptException:
    #     pass