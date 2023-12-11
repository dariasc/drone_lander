#!/usr/bin/env python3
import rospy
from aruco_msgs.msg import MarkerArray
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from lander.msg import Mode

DISARM_Z_THRESHOLD = 0.10

class LanderNode(object):
    def ir_callback(self, data):
        self.z = data.range

    def aruco_callback(self, data):
        for marker in data.markers:
            if marker.id == 42:
                self.latest_pose = marker.pose.pose
        
    def start(self):
        self.state = "FLYING" # POSITIONING, LANDING
        self.latest_pose = None
        self.z = 0
        rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.aruco_callback)
        rospy.Subscriber('/pidrone/range', Range, self.ir_callback)
        self.twist_pub = rospy.Publisher('/pidrone/desired/twist/', Twist)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()
            print(self.state, self.z, self.latest_pose)
            if self.state == 'FLYING':
                continue
            elif self.state == 'LANDING':
                if self.z < DISARM_Z_THRESHOLD:
                    # disarm
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.z = 0
                else:
                    # send -z velocity
                    pass
            elif self.state == "POSITIONING":
                continue

            self.twist_pub.publish(twist)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('lander_node', anonymous=True)
    node = LanderNode()
    node.start()
