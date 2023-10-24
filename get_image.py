#coding:utf-8

import roslib
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import argparse
import os

class GetImage:

    def __init__(self, args):
        self.bridge = CvBridge()
        path = args.input_path
        with rosbag.Bag(path, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == args.topic_of_image:
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        
                        # Generate Image Natimestrme from ROS Timestamp
                        timestamp = msg.header.stamp
                        timestamp_str = str(timestamp)
                        formatted_timestamp = timestamp_str[:-9] + '.' + timestamp_str[-9:]
                        image_name = formatted_timestamp + '.png'
                        
                        # Save the Image
                        if not os.path.exists(args.output_path):
                            os.makedirs(args.output_path)
                            print(f"'{args.output_path}' has been created!")
                        cv2.imwrite(args.output_path + image_name, cv_image)

                    except CvBridgeError as e:
                        print("Error converting ROS Image to OpenCV Image:", e)

            
# deal with optional arguments

parser = argparse.ArgumentParser(
    description="get images from bag file")
parser.add_argument(
    "-pi","--input_path",
    help="Path to the input bag file",
    default='/home/tom/ROS_twizy/bagdata/2023-9-13/',
    type=str)
parser.add_argument(
    "-po","--output_path",
    help="Path to the output folder",
    default='/home/tom/ROS_twizy/bagdata/2023-9-13/output/img_output',
    type=str)
parser.add_argument(
    "-t","--topic_of_image",
    help="The topic of image data",
    default='/theta_s_uvc/image_raw',
    type=str)

args = parser.parse_args()

if __name__== '__main__':
    #rospy.init_node(PKG)

    try:
        get_image = GetImage(args)
    except rospy.ROSInterruptException:
        pass