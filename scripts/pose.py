#!/usr/bin/env python3

import rospy
from adafruit_servokit import ServoKit
from functools import partial
from geometry_msgs.msg import Pose, Quaternion, Vector3, TransformStamped
from math import asin, atan2, pi
from pantilt_pkg.msg import Pose
import tf2_ros
import numpy
import math

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



# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion from Euler angles and axis sequence.
    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple
    >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
    >>> numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
    True
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    quaternion = numpy.empty((4, ), dtype=numpy.float64)
    if repetition:
        quaternion[i] = cj*(cs + sc)
        quaternion[j] = sj*(cc + ss)
        quaternion[k] = sj*(cs - sc)
        quaternion[3] = cj*(cc - ss)
    else:
        quaternion[i] = cj*sc - sj*cs
        quaternion[j] = cj*ss + sj*cc
        quaternion[k] = cj*cs - sj*sc
        quaternion[3] = cj*cc + sj*ss
    if parity:
        quaternion[j] *= -1

    return quaternion


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

    rate = rospy.Rate(5)

    pub = rospy.Publisher('pantilt', Pose, queue_size=1)
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('pantiltPose', Pose, partial(handlePose, kit, pub))

    while not rospy.is_shutdown():
        pose = Pose()
        pose.stamp = rospy.Time.now()
        pose.yaw = kit.servo[0].angle
        pose.pitch = kit.servo[1].angle

        pub.publish(pose)

        yaw = kit.servo[0].angle * pi / 180.
        pitch = kit.servo[1].angle * pi / 180.

        trans = TransformStamped()
        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = "world"
        trans.child_frame_id = "pantilt"

        trans.transform.translation.x = 0
        trans.transform.translation.y = 0
        trans.transform.translation.z = 0

        quat = quaternion_from_euler(0, pitch, yaw)
        trans.transform.rotation.x = quat[0]
        trans.transform.rotation.y = quat[1]
        trans.transform.rotation.z = quat[2]
        trans.transform.rotation.w = quat[3]

        br.sendTransform(trans)

        rate.sleep()

if __name__ == "__main__":
    listener()
