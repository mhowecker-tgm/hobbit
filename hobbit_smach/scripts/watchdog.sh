#!/bin/bash

# This is the watchdog for Hobbit PT2

if [ -f /opt/ros/hobbit_hydro/src/hobbit_launch/scripts/.bash_hobbit_config ]; then
    . /opt/ros/hobbit_hydro/src/hobbit_launch/scripts/.bash_hobbit_config
fi

function send_event {
rostopic pub -1 /Event hobbit_msgs/Event "header:
  seq: 0
  stamp: {secs: $1, nsecs: 0}
  frame_id: ''
event: '$2'
sessionID: ''
confidence: 0.0
params:
- {name: '', value: ''}"
}

function get_line_numbers {
local __resultvar=$1
local myresult=`wc -l < $2`
eval $__resultvar="'$myresult'"
}

function get_file_name {
local __resultvar=$1
local __name="/puppetmaster-32.log"
local myresult=`roscd log && pwd`
eval $__resultvar="'$myresult${__name}'"
}

function restart_master {
pid=`ps aux|grep puppetmaster|grep -v grep|grep -v less|awk {'print$2'}`
for i in ${pid}
do
  echo ${i}
  kill -9 ${i}&
done
roslaunch hobbit_smach puppetmaster.launch&
send_event ${timestamp} 'E_MASTER_RESTART'
}

function check_file {
diff=$(($2-$1))
if [[ "${diff}" -eq 0 ]]; then
  echo -e "\e[31m Puppetmaster is not receiving the watchdog events. We have to restart it.\e[0m";
  echo "";
  restart_master
else
  extended=$((${diff}+100))
  stamps=`tail -n ${extended} $3 |grep secs`
  if echo "${stamps}" | grep -q "$4"; then
    echo -e "\e[32m Puppetmaster is still receiving Events. Do not restart\e[0m";
  else
    echo -e "\e[31m Puppetmaster is not receiving the watchdog events. We have to restart it.\e[0m";
    restart_master
  fi
fi
}

function check_uptime {
local __resultvar=$1
uptime=$(</proc/uptime)
uptime=${uptime%%.*}
local minutes=$(( uptime/60%60 ))
eval $__resultvar="'$minutes'"
}

timestamp=`date +%s|bc`
date
FILE='/home/bajo/.ros/log/puppetmaster.log'
check_uptime TIME
if [[ TIME -lt 10 ]]; then
  exit
fi

get_file_name FILE 
if [[ -e ${FILE} ]]; then
  get_line_numbers before ${FILE}
  send_event ${timestamp} 'E_WATCHDOG'
  get_line_numbers after ${FILE}
  check_file ${before} ${after} ${FILE} ${timestamp}
else
  echo "logfile does not exist in `roscd log && pwd`"
  echo "puppetmaster may not be running or was not started via a launch file"
  restart_master
fi
