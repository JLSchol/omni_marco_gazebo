#!/bin/bash
# rosbag_2_csvs.sh

# use in the directory where rosbag2csvs.sh is located:
# ./rosbag2csvs.sh 	path2file 	outputDir 	topic1 	topic2 	etc..
# $0 				$1			$2 			$3		$4 		etc..
# topics:

# if $3,4 etc is not specified, all topics will be extracted

# path2file="/home/jasper/omni_marco_gazebo/src/data_processing/data/202001140841_D30_W100_L0.01_0.45_S100_1000/202001140841_D30_W100_L0.01_0.45_S100_1000_bag.bag"
# outputdir="/home/jasper/omni_marco_gazebo/src/data_processing/data"

# echo `getRosbagInfo "$1" topic:`

function list_include_item {
  local list="$1"
  local item="$2"
  if [[ $list =~ (^|[[:space:]])"$item"($|[[:space:]]) ]] ; then
    # yes, list include item
    result=0
  else
    result=1
  fi
  return $result
}

# check if topic is in list


all_topics=`rosbag info -y $1 | grep topic: | awk '{print $3}'`
topics=""
if [ -z "$3" ]; then
	topics=$all_topics
else
	for var in "$@" # Skips $0!!!! only $1 >
	do
		if [ "$var" = "$1" ] || [ "$var" = "$2" ]; then 
			continue
		else
			if `list_include_item "$all_topics" "$var"`; then
				topics+=" ${var}"
			else
				echo $var "is not an existing topic and will not be extracted"
			fi
			
		fi
	done
fi

echo "all topics in rosbag: " $all_topics
echo "topics that will be extracted from bag: " $topics

for topic in $topics
do 
	filename="${topic///}"
	echo $filename
	rostopic echo -b $1 -p $topic > $2"/"${filename}.csv
	echo $filename".csv created"
done

