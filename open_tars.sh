#!/bin/bash

# SSH into tarsv5-005 and run the rostopic pub command
ssh tarsv5-005 << EOF
source /opt/ros/melodic/setup.bash
rostopic pub /arduino_msg_buffer std_msgs/UInt16MultiArray "{layout: {dim: [], data_offset: 0}, data: [1, 99, 99]}" -1
EOF
