#!/bin/bash
#
# Instructions:
#
# On nuc run from directory of network.sh
# source network.sh
#
# On control computer run
# source network.sh $IP_OF_NUC
#

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"


ipv4_address=`ifconfig | grep -E 'inet addr:192' | $DIR/parse_ifconfig.py`
export ROS_IP=$ipv4_address
export ROS_MASTER_URI=http://$ipv4_address:1234
if [ "$1" != "" ]
  then
    export ROS_MASTER_URI=http://$1:1234
fi
echo ROS_IP:         $ROS_IP
echo ROS_MASTER_URI: $ROS_MASTER_URI