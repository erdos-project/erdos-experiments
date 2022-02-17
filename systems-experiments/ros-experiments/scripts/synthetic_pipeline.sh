#!/bin/bash
set -x

FREQUENCY=30

NUM_WARMUP_MSGS=100
NUM_MSGS=10000
TIMEOUT="$((($NUM_WARMUP_MSGS + $NUM_MSGS) / $FREQUENCY + 10))s"

CAMERA_MSG_SIZE=921600    # 6222274
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

csv_filename="$OUT_DIR/synthetic-pipeline-inter-process.csv"
tmp_files=()
for num_copies in {1..5}
do
    filename="$TMP_DIR/synthetic-pipeline-$num_copies-copies-inter-process.csv"
    tmp_files+=("$filename")
    if [ "inter-process" == "inter-process" ]; then
        roscore &
        sleep 3
        rosrun latency csv_sink csv_sink samples $filename $NUM_WARMUP_MSGS $NUM_MSGS &
        pid=$!
        for i in $(seq 1 $num_copies)
        do
            rosrun latency receiver "receiver_$i" "control_$i" "samples" &
            rosrun latency join_two "control_$i" "planning_$i" "localization_$i" "control_$i" $CONTROL_MSG_SIZE &
            rosrun latency join_three "planning_$i" "prediction_$i" "lane_detection_$i" \
                "lidar_traffic_light_fusion_$i" "planning_$i" $PLANNING_MSG_SIZE &
            rosrun latency join_two "prediction_$i" "fusion_$i" "localization_$i" "prediction_$i" $PREDICTION_MSG_SIZE &
            rosrun latency join_three "fusion_$i" "tracker_$i" "lidar_$i" "localization_$i" "fusion_$i" $FUSION_MSG_SIZE &
            rosrun latency join_two "lidar_traffic_light_fusion_$i" "lidar_$i" "traffic_light_detection_$i" \
                "lidar_traffic_light_fusion_$i" $FUSION_MSG_SIZE &
            rosrun latency join_two "tracker_$i" "camera_1_$i" "detection_$i" "tracker_$i" $TRACKER_MSG_SIZE &
            rosrun latency map "lane_detection_$i" "camera_1_$i" "lane_detection_$i" $LANE_DETECTION_MSG_SIZE &
            rosrun latency map "traffic_light_detection_$i" "camera_2_$i" "traffic_light_detection_$i" $TRAFFIC_LIGHT_MSG_SIZE &
            rosrun latency map "detection_$i" "camera_1_$i" "detection_$i" $DETECTION_MSG_SIZE &
            rosrun latency map "segmentation_$i" "camera_1_$i" "segmentation_$i" $CAMERA_MSG_SIZE &

            # Sensors
            rosrun latency map "localization_$i" "source" "localization_$i" $LOCALIZATION_MSG_SIZE &
            rosrun latency map "lidar_$i" "source" "lidar_$i" $LIDAR_MSG_SIZE &
            rosrun latency map "camera_2_$i" "source" "camera_2_$i" $CAMERA_MSG_SIZE &
            rosrun latency map "camera_1_$i" "source" "camera_1_$i" $CAMERA_MSG_SIZE &
        done

        # Spawn sources at the end.
        rosrun latency binary_source "source" "source" 0 \
            $NUM_WARMUP_MSGS $NUM_MSGS $FREQUENCY

        # Wait everything to shut down.
        sleep 10

        python3 ../scripts/add_column_to_csv.py \
            -i $filename -o $filename -c "pipeline_copies" -v $num_copies

        pkill -9 "source"
        pkill -9 "receiver"
        pkill -9 "map"
        pkill -9 "join"
        pkill -9 "binary_source"
        pkill -9 "roscore"
        pkill -9 "rosmaster"
        pkill -9 "rosout"
    fi

    python3 ../scripts/add_column_to_csv.py \
        -i $filename -o $filename -c "pipeline_copies" -v $num_copies
done

# Merge files
cat ${tmp_files[0]} > $csv_filename
for filename in ${tmp_files[@]:1}
do
    tail -n +2 $filename >> $csv_filename
done
