TOPIC_BW=$1

(rostopic bw -w 5 $TOPIC_BW) >> output/topic_bw.txt
