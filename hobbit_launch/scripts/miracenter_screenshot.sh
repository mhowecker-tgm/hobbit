#!/bin/bash
window_id=`xwininfo -name "MIRA Center" | awk '{print $4}' | grep -i 0x`
now_=$(date "+%d.%m.%Y-%H.%M.%S")
file_name="/localhome/demo/screenshots/miracenter_$now_.png"
import -window $window_id $file_name
