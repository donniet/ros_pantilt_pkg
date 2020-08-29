#!/usr/bin/env python3

import rospy
from adafruit_servokit import ServoKit
from functools import partial
from geometry_msgs.msg import Pose, Quaternion, Vector3
from math import asin, atan2, pi

def ToEulerAngles(q):
    pitch = 0
    yaw = 0
    roll = 0

    sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = pi / 2
        if sinp < 0:
            pitch = -pitch
    else:
        pitch = asin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def handlePose(kit, pose):
    roll, pitch, yaw = ToEulerAngles(pose.Quaterion)

    print('roll, pitch, yaw = {} {} {}'.format(roll, pitch, yaw))

def handleVector3(kit, pub, vec3):
    print('roll, pitch, yaw = {} {} {}'.format(vec3.x, vec3.y, vec3.z))

    if vec3.y < 0 or vec3.y > kit.servo[1].actuation_range:
        print('pitch value of {} less than 0 or greater than {}'.format(vec3.y, kit.servo[0].actuation_range))
        return

    if vec3.z < 0 or vec3.z > kit.servo[0].actuation_range:
        print('pitch value of {} less than 0 or greater than {}'.format(vec3.z, kit.servo[1].actuation_range))
        return

    kit.servo[0].angle = vec3.z
    kit.servo[1].angle = vec3.y

def listener():
    kit = ServoKit(channels=16)
    #kit.servo[0].angle = 0
    #kit.servo[1].angle = 0

    rospy.init_node('pantilt', anonymous=True)

    rate = rospy.Rate(30)

    pub = rospy.Publisher('pantilt', Vector3, queue_size=10)
    rospy.Subscriber('pantiltPose', Vector3, partial(handleVector3, kit, pub))

    while not rospy.is_shutdown():
        pub.publish(Vector3(0, kit.servo[1].angle, kit.servo[0].angle))
        rate.sleep()

if __name__ == "__main__":
    listener()
