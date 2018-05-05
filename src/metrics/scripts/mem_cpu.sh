#!/bin/bash

PNAME=$1
PID=$(pgrep $PNAME)


while true
do
ps -C gedit -o rss= >> output/mem.log
ps -C gedit -o %cpu= >> output/cpu.log
sleep 1
done
