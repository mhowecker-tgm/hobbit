#!/bin/bash


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR" 

cd ..
./run_it.sh 2> out.txt

cat out.txt | cut -d ' ' -f 3 > scripts/bboxx.txt
cat out.txt | cut -d ' ' -f 5 > scripts/bboxy.txt
cat out.txt | cut -d ' ' -f 7 > scripts/bboxz.txt

cd scripts
./generateGraphs.sh

cd ..

exit 0
