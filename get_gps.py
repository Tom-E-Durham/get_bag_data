#coding:utf-8
import rosbag
import json
from sensor_msgs.msg import NavSatFix
import argparse
import os
import rospy
import tf.transformations

class GetGPS:
    def __init__(self, args):
        path = args.input_path

        # Define the topic name containing sensor_msgs/NavSatFix messages
        navsatfix_topic = '/gps/fix'
        odom_topic = '/gps/odom'
        # Create a list for saving data
        gps_data_list = []
        odom_data_list = []
        # Open the bag file in read mode
        with rosbag.Bag(path, 'r') as bag:
            # Iterate through messages in the specified topic and extract NavSatFix messages
            for topic, msg, t in bag.read_messages(topics=[navsatfix_topic, odom_topic]):
                if topic == navsatfix_topic:
                    # Access the NavSatFix message
                    navsatfix_message = msg
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
                if topic == odom_topic:
                    odom_message = msg
                    timestamp = odom_message.header.stamp
                    timestamp_str = str(timestamp)
                    formatted_timestamp = timestamp_str[:-9] + '.' + timestamp_str[-9:]
                    # Extract relevant information from the Odometry message
                    odom_orien = odom_message.pose.pose.orientation
                    x = odom_orien.x
                    y = odom_orien.y
                    z = odom_orien.z
                    w = odom_orien.w
                    quaternion = (x, y, z, w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    roll = euler[0]
                    pitch = euler[1]
                    yaw = euler[2]
                    odom_data = {
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                        "header": {
                            "timestamp": formatted_timestamp
                        }
                    }
                    odom_data_list.append(odom_data)
        # Close the bag file
        bag.close()

        output_folder_path = os.path.join(args.output_path, 'gps_output')
        if not os.path.exists(output_folder_path):
            os.makedirs(output_folder_path)
            print(f"'{output_folder_path}' has been created!" )
        gps_json_file = os.path.join(output_folder_path, 'gps_data.json')
        # Write GPS data to JSON file
        with open(gps_json_file, 'w') as json_file:
            json.dump(gps_data_list, json_file, indent=4)

        print(f"GPS data has been extracted from '{path}' and saved to '{gps_json_file}'.")

        odom_json_file = os.path.join(output_folder_path, 'odom_data.json')
        # Write GPS data to JSON file
        with open(odom_json_file, 'w') as json_file:
            json.dump(odom_data_list, json_file, indent=4)

        print(f"Odometry data has been extracted from '{path}' and saved to '{odom_json_file}'.")

# deal with optional arguments

parser = argparse.ArgumentParser(
    description="get images from bag file")
parser.add_argument(
    "-pi","--input_path",
    help="Path to the input bag file",
    default='./bag_data',
    type=str)
parser.add_argument(
    "-po","--output_path",
    help="Path to the output file",
    default='./',
    type=str)


args = parser.parse_args()

if __name__== '__main__':
    #rospy.init_node(PKG)
    try:
        get_image = GetGPS(args)
    except rospy.ROSInterruptException:
        pass
