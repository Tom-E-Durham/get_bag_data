#coding:utf-8
import rosbag
import json
from sensor_msgs.msg import NavSatFix
import argparse
import os
import rospy

class GetIMU:
    def __init__(self, args):
        path = args.input_path

        # Define the topic name containing sensor_msgs/NavSatFix messages
        navsatfix_topic = args.topic_of_IMU
        # Create a list for saving data
        imu_data_list = []
        # Open the bag file in read mode
        with rosbag.Bag(path, 'r') as bag:
            # Iterate through messages in the specified topic and extract NavSatFix messages
            for topic, msg, t in bag.read_messages(topics=[navsatfix_topic]):
                if topic == navsatfix_topic:
                    # Access the NavSatFix message
                    navsatfix_message = msg
                    timestamp = navsatfix_message.header.stamp
                    timestamp_str = str(timestamp)
                    formatted_timestamp = timestamp_str[:-9] + '.' + timestamp_str[-9:]
                    # Extract relevant information from the NavSatFix message
                    orientation = navsatfix_message.orientation
                    orien_covariance = navsatfix_message.orientation_covariance
                    angular_velocity = navsatfix_message.angular_velocity
                    a_v_covariance = navsatfix_message.angular_velocity_covariance
                    linear_acceleration = navsatfix_message.linear_acceleration
                    l_a_covariance = navsatfix_message.linear_acceleration_covariance
                    
                    imu_data = {
                        "header": {
                            "timestamp": formatted_timestamp
                        },
                        "orientation": {
                            "x": orientation.x,
                            "y": orientation.y,
                            "z": orientation.z,
                            "w": orientation.w
                        },
                        "orientation_covariance": orien_covariance,
                        "angular_velocity": {
                            "x": angular_velocity.x,
                            "y": angular_velocity.y,
                            "z": angular_velocity.z
                        },
                        "angular_velocity_covariance": a_v_covariance,
                        "linear_acceleration": {
                            "x": linear_acceleration.x,
                            "y": linear_acceleration.y,
                            "z": linear_acceleration.z
                        },
                        "linear_acceleration_covariance": l_a_covariance,
                    }
                    imu_data_list.append(imu_data)
        # Close the bag file
        bag.close()

        output_folder_path = os.path.join(args.output_path, 'imu_output')
        if not os.path.exists(output_folder_path):
            os.makedirs(output_folder_path)
            print(f"'{output_folder_path}' has been created!" )
        output_json_file = os.path.join(output_folder_path, 'imu_data.json')
        # Write IMU data to JSON file
        with open(output_json_file, 'w') as json_file:
            json.dump(imu_data_list, json_file, indent=7)

        print(f"IMU data has been extracted from '{path}' and saved to '{output_json_file}'.")

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
parser.add_argument(
    "-t","--topic_of_IMU",
    help="The topic of IMU data",
    default='/imu/data',
    type=str)

args = parser.parse_args()

if __name__== '__main__':
    #rospy.init_node(PKG)
    try:
        get_image = GetIMU(args)
    except rospy.ROSInterruptException:
        pass
