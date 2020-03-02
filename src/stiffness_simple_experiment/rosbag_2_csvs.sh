#!/bin/bash
# rosbag_2_csvs.sh

# use in the directory where rosbag2csvs.sh is located:
# ./rosbag2csvs.sh 	path2file 	outputDir 
# $0 				$1			$2 			
# topics:


# path2file="/home/jasper/omni_marco_gazebo/src/data_processing/data/202001140841_D30_W100_L0.01_0.45_S100_1000/202001140841_D30_W100_L0.01_0.45_S100_1000_bag.bag"
# outputdir="/home/jasper/omni_marco_gazebo/src/data_processing/data"

# echo `getRosbagInfo "$1" topic:`

topics=`rosbag info -y $1 | grep topic: | awk '{print $3}'`
echo "topics: " $topics

for topic in $topics
do 
	filename="${topic///}"
	echo $filename
	rostopic echo -b $1 -p $topic > $2"/"${filename}.csv
	echo $filename".csv created"
done

