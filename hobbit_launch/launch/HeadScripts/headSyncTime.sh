#!/bin/bash

#we need to perform some super-user commands from a non root user , so we use oure systemCommandsPi suid binary to do those things 

./systemCommandsPi chronyStart
./systemCommandsPi ntpdate
./systemCommandsPi chronyStop

exit 0
