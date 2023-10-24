#coding:utf-8
import rosbag
import json
from sensor_msgs.msg import NavSatFix
import argparse
import os
import rospy

class GetGPS:
    def __init__(self, args):
        path = args.input_path

        # Define the topic name containing sensor_msgs/NavSatFix messages
        navsatfix_topic = args.topic_of_GPS
        # Create a list for saving data
        gps_data_list = []
        # Open the bag file in read mode
        with rosbag.Bag(path, 'r') as bag:
            # Iterate through messages in the specified topic and extract NavSatFix messages
            for topic, msg, t in bag.read_messages(topics=[navsatfix_topic]):
                if topic == navsatfix_topic:
                    # Access the NavSatFix message
                    navsatfix_message = msg
                    timestamp = navsatfix_message.header.stamp
                    timestamp = navsatfix_message.header.stamp
                    timestamp_str = str(timestamp)
                    formatted_timestamp = timestamp_str[:-9] + '.' + timestamp_str[-9:]
                    # Extract relevant information from the NavSatFix message
                    latitude = navsatfix_message.latitude
                    longitude = navsatfix_message.longitude
                    altitude = navsatfix_message.altitude
                    
                    gps_data = {
                        "latitude": latitude,
                        "longitude": longitude,
                        "altitude": altitude,
                        "header": {
                            "timestamp": formatted_timestamp
                        }
                    }
                    gps_data_list.append(gps_data)
        # Close the bag file
        bag.close()
        # Write GPS data to JSON file
        with open(args.output_json_file, 'w') as json_file:
            json.dump(gps_data_list, json_file, indent=4)

        print(f"GPS data has been extracted from '{path}' and saved to '{args.output_json_file}'.")

# deal with optional arguments

parser = argparse.ArgumentParser(
    description="get images from bag file")
parser.add_argument(
    "-pi","--input_path",
    help="Path to the input bag file",
    default='/media/tom/SSD1/Simple_BEV/bag_data/durhamCamp_2023-10-03-13-42-34_0.bag',
    type=str)
parser.add_argument(
    "-po","--output_json_file",
    help="Path to the output file",
    default='output_gps_data.json',
    type=str)
parser.add_argument(
    "-t","--topic_of_GPS",
    help="The topic of GPS data",
    default='/gps/fix',
    type=str)

args = parser.parse_args()

if __name__== '__main__':
    #rospy.init_node(PKG)
    try:
        get_image = GetGPS(args)
    except rospy.ROSInterruptException:
        pass