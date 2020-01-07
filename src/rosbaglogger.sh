#!/bin/bash
# rosbaglogger.sh


# You could pass all arguments to another program like this
# myProgram "$@"

#absolute_path=$PWD"/"$1
echo "path (s1): " $1
# echo "abspath is:" $absolute_path
echo "duration: " $2

topics=""
for var in "$@"
do
	if [ "$var" = "$1" ] || [ "$var" = "$2" ]; then 
		continue
	else
		topics+=" ${var}"
	fi
done
echo "topics: " $topics

#dir_name="yearmonthday_Dduriation_Wmwindowindow_Lwiggle_Sstiffness"

echo "bagname: " $bag_name
echo "parname: " $par_name

echo "s1bagname " $1"/"$bag_name

datum= date +'%Y%m%d%H%M'
Wlength=`rosparam get /stiffness_learning/window_length`
Lmin=`rosparam get /stiffness_learning/lambda_min`
Lmax=`rosparam get /stiffness_learning/lambda_max`
Smin=`rosparam get /stiffness_learning/stiffness_min`
Smax=`rosparam get /stiffness_learning/stiffness_max`

echo "yearmonthdaymin_Dduriation_Wmwindowlength_Lwigglelim_Sstiffnesslim"
dir_name="$datum_D${2}_W${Wlength}_L${Lmin}_${Lmax}_S${Smin}_${Smax}"
echo $dir_name
bag_name="${dir_name}_bag"
par_name="${dir_name}_par"

echo "directory name: " $dir_name

mkdir $1 #relative or absolute path

rosbag record -O $1"/"$bag_name.bag --duration=$2 $topics
rosparam dump $1"/"$par_name.yaml # can add /namespace to find specific params
