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
- {name: '', value: ''}" >/dev/null
}

function get_line_numbers {
local __resultvar=$1
local myresult=`wc -l < $2`
eval $__resultvar="'$myresult'"
}

function get_file_name {
local __resultvar=$1
local __name="/puppetmaster-1.log"
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
  echo 1
else
  extended=$((${diff}+100))
  result=`tail -n ${extended} $3|grep 'E_WATCHDOG'`
  echo $?
  #stamps=`tail -n ${extended} $3 |grep secs`
  #if echo "${stamps}" | grep -q "$4"; then
  #  echo 0
  #else
  #  echo 1
  #  #restart_master
  #fi
fi
}

function check_file_content {
  get_line_numbers before ${1}
  send_event ${timestamp} 'E_WATCHDOG'
  get_line_numbers after ${1}
  count=$(check_file ${before} ${after} ${1} ${timestamp})
  echo $count
}
function loop_files {
  count=0
  files=0
  path=`roscd log && pwd`
  for i in `ls ${path}/puppetmaster-*[0-9].log`; do
    echo $i
    files=$((files + 1))
    if [[ -e ${i} ]]; then
      tmp=$(check_file_content ${i})
      count=$((count + tmp))
    else
      echo "logfile does not exist in `roscd log && pwd`"
      echo "puppetmaster may not be running or was not started via a launch file"
    fi
  done
  echo $files
  echo $count
  if [[ $count -ge $files ]]; then
    echo -e "\e[31m Puppetmaster is not receiving the watchdog events. We have to restart it.\e[0m";
    restart_master
  else
    echo -e "\e[32m Puppetmaster is alive and well.\e[0m";
  fi
}

function check_uptime {
local __resultvar=$1
uptime=$(</proc/uptime)
uptime=${uptime%%.*}
local minutes=$(( uptime/60 ))
eval $__resultvar="'$minutes'"
}

timestamp=`date +%s|bc`
date
FILE='/home/bajo/.ros/log/puppetmaster.log'
check_uptime TIME
if [[ TIME -lt 10 ]]; then
  echo -e "\e[32m Uptime limit not reached.\e[0m";
  exit
fi

loop_files
