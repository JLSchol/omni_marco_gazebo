#!/bin/bash

# Usage:
# ./kill_nodes 	Node1 	Node2 	etc
# $0			$1		$2		etc

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

to_kill_nodes=""
for var in "$@" 	# Skips $0!!!! only $1 >
do
	to_kill_nodes+=" /${var}"
done

active_nodes=`rosnode list`

for to_kill_node in $to_kill_nodes
do
	if `list_include_item "$active_nodes" "$to_kill_node"`; then
		rosnode kill $to_kill_node
	else
		echo $to_kill_node "is not an active node"
	fi
done
