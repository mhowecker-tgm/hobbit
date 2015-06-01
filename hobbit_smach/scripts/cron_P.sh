#!/bin/bash

source /opt/ros/hobbit_hydro/devel/setup.sh


rostopic pub -1 /Event hobbit_msgs/Event "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
event: 'E_PATROL'
sessionID: ''
confidence: 0.0
params:
- {name: '', value: ''}"

exit 0
