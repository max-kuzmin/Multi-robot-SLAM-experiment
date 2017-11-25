sudo ip link set wlan0 down 
sudo iwconfig wlan0 mode ad-hoc 
sudo iwconfig wlan0 channel 4 
sudo iwconfig wlan0 essid 'ROS1' 
sudo iwconfig wlan0 key 1234267895
sudo ip link set wlan0 up 
sudo ip addr add 169.254.34.2/16 dev wlan0
sudo route add -net 224.0.0.0 netmask 224.0.0.0 wlan0

export ROS_IP=169.254.34.2
export ROS_MASTER_URI=http://169.254.34.2:11112
