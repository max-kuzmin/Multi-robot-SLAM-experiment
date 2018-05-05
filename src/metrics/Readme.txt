1. Add to launchfile:

    <group ns="robot0">
        <param name="tf_prefix" value="robot0"/>
        <node name="metrics_node" pkg="metrics" type="metrics_node" output="log"/>
    </group>

2. Also add range_ctrl node to get connection times

3. Run scripts in console simultaneously:

./mem_cpu.sh ohm_tsd_slam
./bw.sh /robot0/scan_inrange_r1

4. Output in files:

output/cpu.log 				- % of cpu time
output/mem.log 				- amount of ram in kb
output/topic_bw.log			- bandwith of topic
/home/<user>/.ros/robot0_rmse.csv	- rmse of estimated and real pose: [x_deviation; y_deviation; rmse]
/home/<user>/.ros/robot0_times.csv	- time of connection between robots: [run_time; connection_time]

Values in files given with interval 1 second
