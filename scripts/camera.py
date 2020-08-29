#!/usr/bin/env python3

import rospy
import picamera
import argparse

from sensor_msgs.msg import Image

class Processor(object):
    def __init__(self, width, height, fps, publisher):
        self.publisher = publisher
        self.fps = fps
        self.width = width
        self.height = height

    def write(self, buf):
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.now()        
        img_msg.height = self.height
        img_msg.width = self.width
        img_msg.encoding = "rgb8" # "bgr8"?
        img_msg.is_bigendian = False # not sure about this either

        img_msg.data = buf
        img_msg.step = 3 * self.width

        #print("publishing image message")

        self.publisher.publish(img_msg)


def raw_frames(width, height, fps=30):
    pub = rospy.Publisher('rawFrames', Image, queue_size=10)
    rospy.init_node('camera', anonymous=True)

    rate = rospy.Rate(fps)
    proc = Processor(width, height, fps, pub)

    with picamera.PiCamera(resolution=(width, height), framerate=fps) as camera:
        camera.start_recording(proc, format='rgb')

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=320)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    try:
        raw_frames(args.width, args.height, args.fps)
    except rospy.ROSInterruptException:
        pass


