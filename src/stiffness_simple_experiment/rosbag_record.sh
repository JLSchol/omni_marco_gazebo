#!/bin/bash

# Usage:
# ./rosbag_record topic1 topic2 etc

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
for var in "$@"
do
	if [ "var" = "$1" ]; then 
		continue
	else
		requisted_record_topics+=" /${var}"
	fi
done

active_topics=`rostopic list`
topic_list=""
for topic in $requisted_record_topics
do
	if `list_include_item "$active_topics" "$topic"`; then
		topic_list+=" ${topic}"
	else
		echo $topic "is not an active topic"
	fi
done





#rosbag record -o /file/name /topic __name:=my_bag