#/bin/bash
#
# Instructions:
#
# On nuc run from directory of network.sh
# source network.sh
#
# On control computer run
# source network.sh $IP_OF_NUC
# 

ipv4_address=`ifconfig | grep -E 'inet addr:10|inet addr:192' | ~/NRMC2018/src/util/parse_ifconfig.py`
export ROS_IP=$ipv4_address
export ROS_MASTER_URI=http://$ipv4_address:1234
if [ "$1" != "" ]
  then 
    export ROS_MASTER_URI=http://$1:1234
fi
echo ROS_IP:         $ROS_IP
echo ROS_MASTER_URI: $ROS_MASTER_URI
