#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose


class HDTAnglerMarker(object):
    def __init__(self):
        rospy.init_node('hdt_marker_target_pub', anonymous=True)

        self.listener = tf.TransformListener()

        self.rate = rospy.Rate(20)
        self.target_pose = Pose()

        self.pub = rospy.Publisher('/target', Pose, queue_size=1)

    def hdt_target_pub(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/base_link', '/aruco_marker_frame', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("No aruco marker target!!!")
                rospy.sleep(1)
                continue
            self.target_pose.position.x = trans[0]
            self.target_pose.position.y = trans[1]
            self.target_pose.position.z = trans[2]
            self.target_pose.orientation.x = rot[0]
            self.target_pose.orientation.y = rot[1]
            self.target_pose.orientation.z = rot[2]
            self.target_pose.orientation.w = rot[3]
            self.pub.publish(self.target_pose)
            print("trans, rot: ", trans, rot)
            self.rate.sleep()


if __name__ == '__main__':
    hdt_target = HDTAnglerMarker()
    hdt_target.hdt_target_pub()

    listener = tf.TransformListener()