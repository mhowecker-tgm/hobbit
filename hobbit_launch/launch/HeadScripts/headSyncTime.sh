#!/bin/bash

sudo /etc/init.d/chrony stop
sudo ntpdate 192.168.2.122
sudo /etc/init.d/chrony start

exit 0
