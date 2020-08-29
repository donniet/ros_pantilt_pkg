#!/usr/bin/env python3

import rospy
from adafruit_servokit import ServoKit
from functools import partial
from geometry_msgs.msg import Pose, Quaternion, Vector3
from math import asin, atan2, pi
from pantilt_pkg.msg import Pose

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


def handleQuaterniaon(kit, pose):
    roll, pitch, yaw = ToEulerAngles(pose.Quaterion)

    print('roll, pitch, yaw = {} {} {}'.format(roll, pitch, yaw))

def handlePose(kit, pub, pose):
    if pose.yaw >= 0 and pose.yaw < kit.servo[0].actuation_range:
        kit.servo[0].angle = pose.yaw

    if pose.pitch >= 0 and pose.pitch < kit.servo[1].actuation_range: 
        kit.servo[1].angle = pose.pitch

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

    rate = rospy.Rate(10)

    pub = rospy.Publisher('pantilt', Pose, queue_size=10)
    rospy.Subscriber('pantiltPose', Pose, partial(handlePose, kit, pub))

    while not rospy.is_shutdown():
        pose = Pose()
        pose.stamp = rospy.Time.now()
        pose.yaw = kit.servo[0].angle
        pose.pitch = kit.servo[1].angle

        pub.publish(pose)
        rate.sleep()

if __name__ == "__main__":
    listener()
