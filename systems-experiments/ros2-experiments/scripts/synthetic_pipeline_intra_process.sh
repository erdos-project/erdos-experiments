!#!/bin/bash

FREQUENCY="30.0"

NUM_WARMUP_MSGS=100
NUM_MSGS=10000

CAMERA_MSG_SIZE=921600
LIDAR_MSG_SIZE=600000
LOCALIZATION_MSG_SIZE=1000
DETECTION_MSG_SIZE=4433
TRAFFIC_LIGHT_MSG_SIZE=246
LANE_DETECTION_MSG_SIZE=358
TRACKER_MSG_SIZE=123
PREDICTION_MSG_SIZE=600
PLANNING_MSG_SIZE=13099
CONTROL_MSG_SIZE=140
FUSION_MSG_SIZE=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

TOTAL_MSGS=$(($NUM_WARMUP_MSGS + $NUM_MSGS))

colcon build
. install/setup.bash

csv_filename="$OUT_DIR/synthetic-pipeline-intra-process.csv"    
tmp_files=()
args=(
    # Binary Source
    -p BinarySource:topic:=source
    -p BinarySource:msg_size:=0
    -p BinarySource:num_msgs:=$TOTAL_MSGS
    -p BinarySource:frequency:=$FREQUENCY
    )
for pipeline_copies in {1..5}
do
    filename="$TMP_DIR/synthetic-pipeline-$pipeline_copies-intra-process.csv"    
    tmp_files+=("$filename")
    i=$(($pipeline_copies - 1))

    args+=(
        # Camera 1
        -p "pipeline_${i}_camera_1:input_topic:=source"
        -p "pipeline_${i}_camera_1:output_topic:=pipeline_${i}_camera_1"
        -p "pipeline_${i}_camera_1:is_sensor:=true"
        -p "pipeline_${i}_camera_1:msg_size:=$CAMERA_MSG_SIZE"
        # Camera 2
        -p "pipeline_${i}_camera_2:input_topic:=source"
        -p "pipeline_${i}_camera_2:output_topic:=pipeline_${i}_camera_2"
        -p "pipeline_${i}_camera_2:is_sensor:=true"
        -p "pipeline_${i}_camera_2:msg_size:=$CAMERA_MSG_SIZE"
        # LIDAR
        -p "pipeline_${i}_lidar:input_topic:=source"
        -p "pipeline_${i}_lidar:output_topic:=pipeline_${i}_lidar"
        -p "pipeline_${i}_lidar:is_sensor:=true"
        -p "pipeline_${i}_lidar:msg_size:=$LIDAR_MSG_SIZE"
        # Localization
        -p "pipeline_${i}_localization:input_topic:=source"
        -p "pipeline_${i}_localization:output_topic:=pipeline_${i}_localization"
        -p "pipeline_${i}_localization:is_sensor:=true"
        -p "pipeline_${i}_localization:msg_size:=$LOCALIZATION_MSG_SIZE"
        # Segmentation
        -p "pipeline_${i}_segmentation:input_topic:=pipeline_${i}_camera_1"
        -p "pipeline_${i}_segmentation:output_topic:=pipeline_${i}_segmentation"
        -p "pipeline_${i}_segmentation:is_sensor:=false"
        -p "pipeline_${i}_segmentation:msg_size:=$CAMERA_MSG_SIZE"
        # Obstacle Detection
        -p "pipeline_${i}_obstacle_detection:input_topic:=pipeline_${i}_camera_1"
        -p "pipeline_${i}_obstacle_detection:output_topic:=pipeline_${i}_obstacle_detection"
        -p "pipeline_${i}_obstacle_detection:is_sensor:=false"
        -p "pipeline_${i}_obstacle_detection:msg_size:=$DETECTION_MSG_SIZE"
        # Traffic Light Detection
        -p "pipeline_${i}_traffic_light_detection:input_topic:=pipeline_${i}_camera_2"
        -p "pipeline_${i}_traffic_light_detection:output_topic:=pipeline_${i}_traffic_light_detection"
        -p "pipeline_${i}_traffic_light_detection:is_sensor:=false"
        -p "pipeline_${i}_traffic_light_detection:msg_size:=$TRAFFIC_LIGHT_MSG_SIZE"
        # Lane Detection
        -p "pipeline_${i}_lane_detection:input_topic:=pipeline_${i}_camera_2"
        -p "pipeline_${i}_lane_detection:output_topic:=pipeline_${i}_lane_detection"
        -p "pipeline_${i}_lane_detection:is_sensor:=false"
        -p "pipeline_${i}_lane_detection:msg_size:=$LANE_DETECTION_MSG_SIZE"
        # Tracker
        -p "pipeline_${i}_tracker:left_topic:=pipeline_${i}_camera_1"
        -p "pipeline_${i}_tracker:right_topic:=pipeline_${i}_obstacle_detection"
        -p "pipeline_${i}_tracker:output_topic:=pipeline_${i}_tracker"
        -p "pipeline_${i}_tracker:msg_size:=$TRACKER_MSG_SIZE"
        # LIDAR Traffic Light Fusion
        -p "pipeline_${i}_lidar_traffic_light_fusion:left_topic:=pipeline_${i}_lidar"
        -p "pipeline_${i}_lidar_traffic_light_fusion:right_topic:=pipeline_${i}_traffic_light_detection"
        -p "pipeline_${i}_lidar_traffic_light_fusion:output_topic:=pipeline_${i}_lidar_traffic_light_fusion"
        -p "pipeline_${i}_lidar_traffic_light_fusion:msg_size:=$FUSION_MSG_SIZE"
        # Tmp Fusion
        -p "pipeline_${i}_tmp_fusion:left_topic:=pipeline_${i}_tracker"
        -p "pipeline_${i}_tmp_fusion:right_topic:=pipeline_${i}_lidar"
        -p "pipeline_${i}_tmp_fusion:output_topic:=pipeline_${i}_tmp_fusion"
        -p "pipeline_${i}_tmp_fusion:msg_size:=$FUSION_MSG_SIZE"
        # Fusion
        -p "pipeline_${i}_fusion:left_topic:=pipeline_${i}_tmp_fusion"
        -p "pipeline_${i}_fusion:right_topic:=pipeline_${i}_localization"
        -p "pipeline_${i}_fusion:output_topic:=pipeline_${i}_fusion"
        -p "pipeline_${i}_fusion:msg_size:=$FUSION_MSG_SIZE"
        # Prediction
        -p "pipeline_${i}_prediction:left_topic:=pipeline_${i}_fusion"
        -p "pipeline_${i}_prediction:right_topic:=pipeline_${i}_localization"
        -p "pipeline_${i}_prediction:output_topic:=pipeline_${i}_prediction"
        -p "pipeline_${i}_prediction:msg_size:=$PREDICTION_MSG_SIZE"
        # Tmp Planning
        -p "pipeline_${i}_tmp_planning:left_topic:=pipeline_${i}_prediction"
        -p "pipeline_${i}_tmp_planning:right_topic:=pipeline_${i}_lane_detection"
        -p "pipeline_${i}_tmp_planning:output_topic:=pipeline_${i}_tmp_planning"
        -p "pipeline_${i}_tmp_planning:msg_size:=$PLANNING_MSG_SIZE"
        # Planning
        -p "pipeline_${i}_planning:left_topic:=pipeline_${i}_tmp_planning"
        -p "pipeline_${i}_planning:right_topic:=pipeline_${i}_lidar_traffic_light_fusion"
        -p "pipeline_${i}_planning:output_topic:=pipeline_${i}_planning"
        -p "pipeline_${i}_planning:msg_size:=$PLANNING_MSG_SIZE"
        # Control
        -p "pipeline_${i}_control:left_topic:=pipeline_${i}_planning"
        -p "pipeline_${i}_control:right_topic:=pipeline_${i}_localization"
        -p "pipeline_${i}_control:output_topic:=pipeline_${i}_control"
        -p "pipeline_${i}_control:msg_size:=$CONTROL_MSG_SIZE"
        # Receiver
        -p "pipeline_${i}_receiver:recv_topic:=pipeline_${i}_control"
        -p "pipeline_${i}_receiver:is_inter_process:=false"
    )

    ros2 run latency synthetic_pipeline $pipeline_copies \
        --ros-args \
        -p CSVSink:warmup_samples:=$NUM_WARMUP_MSGS \
        -p CSVSink:samples:=$NUM_MSGS \
        -p CSVSink:is_inter_process:=false \
        -p CSVSink:filename:=$filename \
        "${args[@]}"

    python3 ../scripts/add_column_to_csv.py \
            -i $filename -o $filename -c "pipeline_copies" -v $pipeline_copies
done

# Merge files    
cat ${tmp_files[0]} > $csv_filename    
for filename in ${tmp_files[@]:1}    
do    
    tail -n +2 $filename >> $csv_filename    
done