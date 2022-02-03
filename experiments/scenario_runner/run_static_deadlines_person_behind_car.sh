#!/bin/bash

hzs=(5)
detection_runtimes=(20 50 80 150 200 270)
detectors=(efficientdet-d2 efficientdet-d3 efficientdet-d4 efficientdet-d5 efficientdet-d6 efficientdet-d7)
planning_runtimes=(309 208 148 67 40 30)
target_speeds=(11 12 13)
file_prefix_name=person_behind_car
scenario=ERDOSPedestrianBehindCar

cd ${PYLOT_HOME}
dir_name=results_${file_prefix_name}
mkdir -p ${dir_name}

for run in `seq 1 10`; do
    for hz in ${hzs[@]}; do
        for target_speed in ${target_speeds[@]}; do
            for d_index in ${!detection_runtimes[@]}; do
                for planning in ${planning_runtimes[@]}; do
                    file_base=${dir_name}/${file_prefix_name}_detection_${detection_runtimes[$d_index]}_planning_${planning}_target_speed_${target_speed}_Hz_${hz}_run_${run}
                    if [ ! -f "${PYLOT_HOME}/${file_base}.csv" ]; then
                        echo "[x] Running the experiment with detection deadline ${detection_runtimes[$d_index]} , planning deadline $planning , target speed $target_speed"
                        cd ${PYLOT_HOME}/scripts ; ./run_simulator.sh &
                        sleep 10
                        cd $PYLOT_HOME ; python3 scenario_runner.py --flagfile=configs/scenarios/person_avoidance_static_deadlines.conf --target_speed=$target_speed --log_file_name=$file_base.log --csv_log_file_name=$file_base.csv --profile_file_name=$file_base.json --deadline_enforcement=static --detection_deadline=${detection_runtimes[$d_index]} --planning_deadline=$planning --obstacle_detection_model_paths=efficientdet/${detectors[$d_index]} --obstacle_detection_model_names=${detectors[$d_index]} --simulator_camera_frequency=$hz --simulator_imu_frequency=$hz --simulator_lidar_frequency=$hz --simulator_localization_frequency=$hz &
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
    done
done
