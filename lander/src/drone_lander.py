#!/usr/bin/env python3
import rospy
from aruco_msgs.msg import MarkerArray
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, Pose
from lander.msg import Mode

# Threshold for how many detections before landig
ARUCO_SEEN_THRESHOLD = 1
# Threshold for disarming when landing
DISARM_Z_THRESHOLD = 0.075
# Landing speed
LANDING_Z_SPEED = 0.05

class LanderNode(object):
    def ir_callback(self, data):
        # Callback function for the infrared range sensor
        self.z = data.range

    def aruco_callback(self, data):
        # Callback function for the ARUCO marker detection
        for marker in data.markers:
            if marker.id == 42:
                self.latest_pose = marker.pose.pose
                self.aruco_seen += 1
        
    def start(self):
        # Initialize node parameters and publishers/subscribers
        self.state = 'FLYING'  # Initial state is FLYING
        self.latest_pose = None  # Latest ARUCO marker pose
        self.aruco_seen = 0  # How many times the camera has seen the ARUCO marker
        self.z = 0  # Infrared range sensor reading

        rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.aruco_callback)
        rospy.Subscriber('/pidrone/range', Range, self.ir_callback)
        self.twist_pub = rospy.Publisher('/pidrone/desired/twist', Twist, queue_size=1)
        self.pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
        self.mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
        
        r = rospy.Rate(10)  # Set the loop rate to 10 Hz
        while not rospy.is_shutdown():
            print(self.state, self.z)  # Print current state and infrared range for debugging
            if self.latest_pose != None:
                print(self.latest_pose.position)

            # State machine logic
            if self.state == 'FLYING':
                if self.aruco_seen >= ARUCO_SEEN_THRESHOLD:
                    self.state = 'LANDING'  # Change state
                continue  # Continue flying without any specific actions
            elif self.state == 'LANDING':
                if self.z < DISARM_Z_THRESHOLD:
                    # Disarm the drone if the altitude is below the threshold
                    mode = Mode()
                    mode.mode = 'DISARMED'
                    self.mode_pub.publish(mode)

                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.z = 0
                    self.twist_pub.publish(twist)
                    break  # Break out of the loop and stop publishing twist commands
                else:
                    pose = Pose()
                    pose.position.x = 0
                    pose.position.y = 0
                    pose.position.z = -LANDING_Z_SPEED
                    self.pose_pub.publish(pose)

            r.sleep()

if __name__ == '__main__':
    # Initialize ROS node and start the LanderNode
    rospy.init_node('lander_node', anonymous=True)
    node = LanderNode()
    node.start()
