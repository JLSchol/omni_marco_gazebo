#!/bin/bash

# Usage:
# ./rosbag_record 	saveDir bagName logNodeName topic1 	topic2 	etc
# $0 				$1		$2 		$3 			$3		$4 		etc

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

requisted_record_topics=""
for var in "$@" # Skips $0!!!! only $1 >
do
	if [ "$var" = "$1" ] || [ "$var" = "$2" ] || [ "$var" = "$3" ]; then 
		continue
	else

		requisted_record_topics+=" /${var}"
	fi
done

active_topics=`rostopic list`
record_list=""
for topic in $requisted_record_topics
do
	if `list_include_item "$active_topics" "$topic"`; then
		record_list+=" ${topic}"
	else
		echo $topic "is not an active topic and will not be recorded"
	fi
done

rosbag record -O $1"/"$2.bag $record_list __name:=$3 &
rosparam dump $1"/"$2.yaml