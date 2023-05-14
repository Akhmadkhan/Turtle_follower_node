#!/usr/bin/env python3

import rospy
from math import atan2
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleFollowerNode:
    def __init__(self, follower_name, leader_name, follower_speed):
        self.follower_name = follower_name
        self.leader_name = leader_name
        self.follower_speed = follower_speed

        self.follower_pose = Pose()
        self.leader_pose = Pose()

        rospy.init_node('turtle_follower')

        # Подписка на топики с позициями черепашек
        rospy.Subscriber('/{}/pose'.format(follower_name), Pose, self.follower_pose_callback)
        rospy.Subscriber('/{}/pose'.format(leader_name), Pose, self.leader_pose_callback)

        # Публикация команд на движение для второй черепашки
        self.cmd_vel_pub = rospy.Publisher('/{}/cmd_vel'.format(follower_name), Twist, queue_size=10)

    def follower_pose_callback(self, msg):
        self.follower_pose = msg

    def leader_pose_callback(self, msg):
        self.leader_pose = msg

    def compute_velocity(self):
        # Вычисление команды на движение для второй черепашки
        cmd_vel = Twist()
        cmd_vel.linear.x = self.follower_speed

        # Расчет трансформации между позициями черепашек
        dx = self.leader_pose.x - self.follower_pose.x
        dy = self.leader_pose.y - self.follower_pose.y

        # Направление движения второй черепашки к первой
        cmd_vel.angular.z = -1.0 * (self.follower_pose.theta - atan2(dy, dx))

        return cmd_vel

    def run(self):
        rate = rospy.Rate(10)  # Частота публикации команд

        while not rospy.is_shutdown():
            cmd_vel = self.compute_velocity()
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()


if __name__ == '__main__':
    follower_name = 'turtle2'
    leader_name = 'turtle1'
    follower_speed = rospy.get_param('~follower_speed', 1.0)  # Чтение скорости преследователя из параметров

    turtle_follower = TurtleFollowerNode(follower_name, leader_name, follower_speed)
    turtle_follower.run()
