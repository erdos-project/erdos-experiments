#!/bin/bash

hzs=(5)
detectors=(efficientdet-d7 efficientdet-d6 efficientdet-d5 efficientdet-d4 efficientdet-d3 efficientdet-d2)
target_speeds=(8 10 12)
file_prefix_name=traffic_jam
scenario=ERDOSTrafficJam

cd ${PYLOT_HOME}
dir_name=results_${file_prefix_name}
mkdir -p ${dir_name}

for hz in ${hzs[@]}; do
    for run in `seq 1 10`; do
        for target_speed in ${target_speeds[@]}; do
            file_base=${dir_name}/${file_prefix_name}_dynamic_deadlines_target_speed_${target_speed}_Hz_${hz}_run_${run}
            if [ ! -f "${PYLOT_HOME}/${file_base}.csv" ]; then
                echo "[x] Running the experiment with dynamic deadlines, target speed $target_speed"
                cd ${PYLOT_HOME}/scripts ; ./run_simulator.sh &
                sleep 10
                cd $PYLOT_HOME ; python3 pylot.py --flagfile=configs/scenarios/traffic_jam_static_deadlines.conf --target_speed=$target_speed --log_file_name=$file_base.log --csv_log_file_name=$file_base.csv --profile_file_name=$file_base.json --deadline_enforcement=dynamic --obstacle_detection_model_paths=${PYLOT_HOME}/dependencies/models/obstacle_detection/efficientdet/efficientdet-d1/efficientdet-d1_frozen.pb,${PYLOT_HOME}/dependencies/models/obstacle_detection/efficientdet/efficientdet-d2/efficientdet-d2_frozen.pb,${PYLOT_HOME}/dependencies/models/obstacle_detection/efficientdet/efficientdet-d3/efficientdet-d3_frozen.pb,${PYLOT_HOME}/dependencies/models/obstacle_detection/efficientdet/efficientdet-d4/efficientdet-d4_frozen.pb,${PYLOT_HOME}/dependencies/models/obstacle_detection/efficientdet/efficientdet-d5/efficientdet-d5_frozen.pb,${PYLOT_HOME}/dependencies/models/obstacle_detection/efficientdet/efficientdet-d6/efficientdet-d6_frozen.pb,${PYLOT_HOME}/dependencies/models/obstacle_detection/efficientdet/efficientdet-d7/efficientdet-d7_frozen.pb --obstacle_detection_model_names=efficientdet-d1,efficientdet-d2,efficientdet-d3,efficientdet-d4,efficientdet-d5,efficientdet-d6,efficientdet-d7 --simulator_camera_frequency=$hz --simulator_imu_frequency=$hz --simulator_lidar_frequency=$hz --simulator_localization_frequency=$hz &
                cd $ROOT_SCENARIO_RUNNER ; python3 scenario_runner.py --scenario $scenario --reloadWorld --timeout 600
                echo "[x] Scenario runner finished. Killing Pylot..."
                pkill --signal 9 -f scenario_runner.py
                # Kill the simulator
                `ps aux | grep CarlaUE4 | awk '{print $2}' | head -n -1 | xargs kill -9`
                `ps aux | grep CarlaUE4 | awk '{print $2}' | tail -n -1 | xargs kill -9`
                 sleep 5
             else
                 echo "$file_base exists"
             fi
        done
    done
done
