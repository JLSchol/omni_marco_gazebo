#!/bin/bash

# rosbag2csvs.sh

# use in the directory where rosbag2csvs.sh is located:
# ./rosbag2csvs.sh relative_directory bagDuration Topic1 Topic2 etc...
# topics:


# This script extracts multiple csv files accourding to the topic that 
# are found from a single rosbag 


topics=""
for var in "$@"
do
	if [ "$var" = "$1" ] || [ "$var" = "$2" ]; then 
		continue
	else
		topics+=" ${var}"
	fi
done

datum=`date +'%Y%m%d%H%M'`
Wlength=`rosparam get /stiffness_commanding/window_length`
Lmin=`rosparam get /stiffness_commanding/lambda_min`
Lmax=`rosparam get /stiffness_commanding/lambda_max`
Smin=`rosparam get /stiffness_commanding/stiffness_min`
Smax=`rosparam get /stiffness_commanding/stiffness_max`


dir_name="${datum}_D${2}_W${Wlength}_L${Lmin}_${Lmax}_S${Smin}_${Smax}"
bag_name="${dir_name}_bag"
par_name="${dir_name}_par"

echo "path (s1): " $1"/"$dir_name
echo "duration: " $2
echo "topics: " $topics

mkdir -p $1"/"$dir_name

rosbag record -O $1"/"$dir_name"/"$bag_name.bag --duration=$2 $topics
rosparam dump $1"/"$dir_name"/"$par_name.yaml # can add /namespace to find specific params


for topic in $topics
do 
	filename="${topic///}"
	rostopic echo -b $PWD"/"$1"/"$dir_name"/"$bag_name.bag -p $topic > ${filename}.csv
	mv "${filename}".csv $1"/"$dir_name
	echo $filename".csv created"
done


