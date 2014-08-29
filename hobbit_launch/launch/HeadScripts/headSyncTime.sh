#!/bin/bash

#we need to perform some super-user commands from a non root user , so we use oure systemCommandsPi suid binary to do those things 

./systemCommandsPi chronyStop
./systemCommandsPi ntpdate
./systemCommandsPi chronyStart

exit 0
