#!/bin/sh

# Figure 2a: EfficientDet Heatmap
python3 plot_edet_heatmap.py
mv scenario_edet_matrix.pdf graphs/

# Figure 2b: Object Tracking runtimes with number of obstacles.
python3 plot_tracker_runtime_vs_num_targets.py --log_file=../experiments/scenario_runner/tracker_evaluation/runtime_vs_num_targets/all_runtime_logs.csv --file_format=pdf --mode=small_paper --file_name=tracker_runtime_vs_num_targets
mv tracker_runtime_vs_num_targets.pdf graphs/

# Figure 2c: Prediction runtime with prediction horizon.
python3 plot_prediction_runtimes.py --base_dir=../experiments/scenario_runner/prediction/ --file_format=pdf --small_paper_mode --file_name=prediction-runtime-horizon
mv prediction-runtime-horizon.pdf graphs/

# Figure 2d: Jerk vs. Planning runtime.
csv_files=../experiments/scenario_runner/planning/person_behind_car_frenet/person_behind_car_frenet_maxt_5.0_dt_0.3_droadw_0.7_target_speed_18.csv,../experiments/scenario_runner/planning/person_behind_car_frenet/person_behind_car_frenet_maxt_5.0_dt_0.2_droadw_0.5_target_speed_18.csv,../experiments/scenario_runner/planning/person_behind_car_frenet/person_behind_car_frenet_maxt_5.0_dt_0.1_droadw_0.3_target_speed_18.csv
log_files=../experiments/scenario_runner/planning/person_behind_car_frenet/person_behind_car_frenet_maxt_5.0_dt_0.3_droadw_0.7_target_speed_18.log,../experiments/scenario_runner/planning/person_behind_car_frenet/person_behind_car_frenet_maxt_5.0_dt_0.2_droadw_0.5_target_speed_18.log,../experiments/scenario_runner/planning/person_behind_car_frenet/person_behind_car_frenet_maxt_5.0_dt_0.1_droadw_0.3_target_speed_18.log
json_files=../experiments/scenario_runner/planning/person_behind_car_frenet/person_behind_car_frenet_maxt_5.0_dt_0.3_droadw_0.7_target_speed_18.json,../experiments/scenario_runner/planning/person_behind_car_frenet/person_behind_car_frenet_maxt_5.0_dt_0.2_droadw_0.5_target_speed_18.json,../experiments/scenario_runner/planning/person_behind_car_frenet/person_behind_car_frenet_maxt_5.0_dt_0.1_droadw_0.3_target_speed_18.json
python3 plot_pylot_graphs.py --csv_file_names=${csv_files} --profile_file_names=${json_files} --log_file_names=${log_files} --lateral_jerk --min_x_val=0 --max_x_val=1601 --min_y_val=-150 --max_y_val=150 --y_ticks_increment=50  --x_ticks_increment=2000  --ignore_first_sim_time_ms=12400 --labels="125,200,500" --file_name=synchronous_lateral_jerk --stretched --file_format=pdf
mv planning-absolute-jerk.pdf graphs/

# Figure 3: Apollo Perception and Prediction timeline.
python3 plot_apollo_rosbag_analysis.py --files=../experiments/apollo/sensor_perception_prediction_timeline_apollo_1.5.csv --file_name=apollo_timeline --file_format=pdf --paper_mode
mv apollo_timeline.pdf graphs/

HOST="erdos3"
# Figure 8a: Latency for different message sizes.
python3 plot_msg_size_latency.py \
    "../eurosys_systems_experiments/$HOST/erdos/msg-size-latency-intra-process.csv" \
    "../eurosys_systems_experiments/$HOST/flink/msg-size-latency-intra-process.csv" \
    "../eurosys_systems_experiments/$HOST/erdos/msg-size-latency-inter-process.csv" \
    "../eurosys_systems_experiments/$HOST/flink/msg-size-latency-inter-process.csv" \
    "../eurosys_systems_experiments/$HOST/ros/msg-size-latency-inter-process.csv"

# Figure 8b: Latency for broadcast communication patterns.
python3 plot_broadcast_latency.py \
    "../eurosys_systems_experiments/$HOST/erdos/broadcast-latency-intra-process.csv" \
    "../eurosys_systems_experiments/$HOST/flink/broadcast-latency-intra-process.csv" \
    "../eurosys_systems_experiments/$HOST/erdos/broadcast-latency-inter-process.csv" \
    "../eurosys_systems_experiments/$HOST/flink/broadcast-latency-inter-process.csv" \
    "../eurosys_systems_experiments/$HOST/ros/broadcast-latency-inter-process.csv"

# Figure 8c: Latency for a synthetic pipeline at different scales.
python3 plot_synthetic_pipeline.py \
    "../eurosys_systems_experiments/$HOST/erdos/synthetic-pipeline-intra-process.csv" \
    "../eurosys_systems_experiments/$HOST/erdos/synthetic-pipeline-inter-process.csv" \
    "../eurosys_systems_experiments/$HOST/ros/synthetic-pipeline-inter-process.csv" 
# Figure 9: Mode changes for detection and planning.
python3 plot_mode_changes.py --base_dir=../experiments/challenge/mode_changes/ --file_format=pdf --paper_mode
mv pylot-mode-changes.pdf graphs/

# Generate box plot of response time without and with policy. 
python3 plot_challenge_policy.py --base_dir=../experiments/challenge/comparison_w_and_wo_policy/ --file_format=pdf --small_paper_mode --plot_runtimes --towns=1 --start_route=1 --end_route=9 --num_reps=5 --verbose

# Figure 10: Box and whiskers of runtime with and without deadline handlers.
python3 plot_challenge_deadline_paper.py --base_dir=../experiments/challenge/deadlines_comparison_with_accurate_lidar/ --file_format=png --paper_mode --towns=1 --start_route=1 --end_route=9 --num_reps=7 --file_format=pdf --small_paper_mode
mv deadline-bbox.pdf graphs/

# Figure 11: Barchart of collisions.
python3 generate_collision_stats.py --base_dir=../experiments/challenge/deadlines_comparison_with_accurate_lidar/ --filter_carla_cola=True --towns=1 --start_route=1 --end_route=9 --num_reps=7
python3 plot_challenge_collisions_barchart.py
mv pylot_collisions.pdf graphs/

# Figure 12: Histogram of end-to-end response time static vs. dynamic deadlines.
python3 plot_challenge_e2e_response_time.py --base_dir=../experiments/challenge/deadlines_comparison_with_accurate_lidar/ --end_route=9 --num_reps=7 --file_format=pdf --small_paper_mode --plot_histogram
mv configurations-e2e-runtime-hist.pdf graphs/

# Figure 13: Heatmap of collission for different target speeds and deadline configurations.
python3 plot_scenario_collision_heatmap.py
mv speed-deadline-dd-heatmap.pdf graphs/

# Figure 14: Timeline of end-to-end response time across a scenario.
python3 plot_scenario_response_time_timeline.py
mv dynamic_deadline_timeline.pdf graphs/
