#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

class TimeShifter:
    def __init__(self):
        self.pub = rospy.Publisher('/depth_camera/depth/color/points_wrong_timing', PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber('/depth_camera/depth/color/points', PointCloud2, self.callback)

    def callback(self, data):
        rospy.loginfo('Received ' + str(data.header.stamp.secs) + '.' + str(data.header.stamp.nsecs))
        data.header.stamp = data.header.stamp + rospy.Duration.from_sec(-0.16)
        self.pub.publish(data)
        rospy.loginfo('Published ' + str(data.header.stamp.secs) + '.' + str(data.header.stamp.nsecs))

if __name__ == '__main__':
    try:
        rospy.loginfo('Started.')
        rospy.init_node('time_shifter', anonymous=True)
        time_shifter = TimeShifter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
