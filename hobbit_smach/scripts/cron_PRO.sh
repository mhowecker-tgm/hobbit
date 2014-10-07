#!/bin/bash

source /localhome/demo/.bashrc

rostopic pub -1 /Event hobbit_msgs/Event "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
event: 'E_PROACTIVE'
sessionID: ''
confidence: 0.0
params:
- {name: '', value: ''}"
