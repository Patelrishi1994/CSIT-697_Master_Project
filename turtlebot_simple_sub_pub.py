#! /usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Follow_wall(object):
    def __init__(self, pub):
        self._pub = pub
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        pub = self._pub
        rate = self.rate
        msg.ranges
        move = Twist()

        print(f"Minimulm value in is: {min(msg.ranges)}")
        print(f"On Index: {msg.ranges.index(min(msg.ranges))}")
        front_left_dis = msg.ranges[105]
        front_dis = msg.ranges[90]      # 75 - 105
        front_right_dis = msg.ranges[75]

        left_front_dis = msg.ranges[120]
        left_dis = msg.ranges[145]      # 120 - 150
        left_back_dis = msg.ranges[150]

        right_front_dis = msg.ranges[60]
        right_dis = msg.ranges[45]      # 30 - 60
        right_back_dis = msg.ranges[30]

        back_dis = msg.ranges[0]
        print(f"F - Left distance is: {front_left_dis}")
        print(f"Front distance is: {front_dis}")
        print(f"F - Right distance is: {front_right_dis}")

        print(f"R - Front distance is: {right_front_dis}")
        print(f"Right distance is: {right_dis}")
        print(f"R - Back distance is: {right_back_dis}")

        print(f"L - Front distance is: {left_front_dis}")
        print(f"Left distance is: {left_dis}")
        print(f"L - Back distance is: {left_back_dis}")

        print(f"Back distance is: {back_dis}")

        if (front_dis == float('inf') or front_left_dis == float('inf') or front_right_dis == float('inf') or right_dis == float('inf') or right_back_dis == float('inf') or right_front_dis == float('inf')) and (front_dis < 0.15 or front_left_dis < 0.15 or front_right_dis < 0.15 or right_dis < 0.15 or right_back_dis < 0.15 or right_front_dis < 0.15):
            rospy.loginfo("REVERSING")
            move.angular.z = 0.0
            move.linear.x = -0.09

        elif front_dis < 0.4 or front_left_dis < 0.4 or front_right_dis < 0.4:
            rospy.loginfo("TURNING LEFT.")
            move.angular.z = 0.5
            move.linear.x = 0

        elif right_dis < 0.2 or right_back_dis < 0.2 or right_front_dis < 0.2:
            rospy.loginfo("TURNING LEFT - FORWARD.")
            move.angular.z = 0.3
            move.linear.x = 0.08

        elif (right_dis > 0.2 or right_back_dis > 0.2 or right_front_dis > 0.2) and (right_dis < 0.3 or right_back_dis < 0.3 or right_front_dis < 0.3) and ((front_dis > 0.7 or front_right_dis > 0.5 or front_left_dis > 0.5) and (front_dis != float('inf') and front_left_dis != float('inf') and front_right_dis != float('inf'))):
            rospy.loginfo("MOVING FORWARD")
            move.angular.z = 0.0
            move.linear.x = 0.09

        elif right_dis > 0.3 or right_back_dis > 0.3 or right_front_dis > 0.3:
            rospy.loginfo("TURNING RIGHT - FORWARD.")
            move.angular.z = -0.3
            move.linear.x = 0.08

        pub.publish(move)


def main():
    # Initiating Node
    rospy.init_node("wall_follow_node", anonymous=True)

    # Initializing publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    # Creating class object
    fw = Follow_wall(pub)

    # Initiating subscriber
    rospy.Subscriber('/scan', LaserScan, fw.callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
