#!/usr/bin/env bash

# Check if the script is provided with two path parameters
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <input_bag_path> <output_folder_path>"
    exit 1
fi

# Get the input and output paths from the parameters
input_path="$1"
output_path="$2"

# Use the input and output paths in your commands
echo "Input path: $input_path"
echo "Output path: $output_path"

source ~/catkin_ws/devel/setup.bash
source ~/ROS_twizy/rospy/bin/activate

# For image
python3 get_image.py -pi "$input_path" -po "$output_path" -t /theta_s_uvc/image_raw

# For pcd
# Format: rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>
#rosrun pcl_ros bag_to_pcd $input_path /os_cloud_node/points $output_path/pcd_output

# For gps
#python3 get_gps.py -pi "$input_path" -po "$output_path" -t /gps/fix