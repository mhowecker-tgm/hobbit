#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

mkdir dataset
rm dataset/*
mv *.png dataset/

THEDATETAG=`date +"%y-%m-%d_%H-%M-%S"`
tar cvfjh "hobbitRecord_$THEDATETAG.tar.bz2" dataset/


mkdir ../../web_interface/bin/recordings/
mv "hobbitRecord_$THEDATETAG.tar.bz2" ../../web_interface/bin/recordings/

rm dataset/*

exit 0
