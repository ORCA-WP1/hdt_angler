#!/bin/bash

sleep 2
while true; do
  returnstring=$(rosnode list)
  #echo $returnstring
  if [[ $returnstring =~ "controller_manager" ]]
  then
    echo "delay_script: waiting for controller_manager to finish"
  else
    echo "delay_script: controller_manager done"
    break
  fi
  sleep 1
done

# start second part of bring up script
roslaunch hdt_angler_bringup angler_a2_bringup.launch 

