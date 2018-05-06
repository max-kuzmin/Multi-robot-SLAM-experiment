#!/bin/bash

PNAME=$1
PID=$(pgrep $PNAME)
PID=$(echo $PID | grep -o "^\S*")
echo $PID

while true
do
ps -p $PID -o rss= >> output/mem.log
ps -p $PID -o %cpu= >> output/cpu.log
sleep 1
done
