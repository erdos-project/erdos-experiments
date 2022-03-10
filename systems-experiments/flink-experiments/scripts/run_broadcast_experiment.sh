#!/usr/bin/env bash
# Runs experiment and copies result to the local filesystem.
# Run this from the `flink-experiments` directory.

# Parse args.
MSG_SIZE=$1
NUM_RECEIVERS=$2
SAMPLES=$3
WARMUP_SAMPLES=$4
FREQUENCY=$5
CSV_FILENAME=$6
ANALYZE_PERFORMANCE=$7

# Compile
mvn package

# Launch cluster
docker-compose up -d

# Set up head.
JOBMANAGER_CONTAINER=$(docker ps --filter name=jobmanager --format={{.ID}})
docker cp ./target/flink-experiments-0.1.jar "$JOBMANAGER_CONTAINER":/job.jar

# Set up workers.
TASK_MANAGER_CONTAINERS=$(docker ps --filter name=taskmanager --format={{.ID}})
for NODE in $TASK_MANAGER_CONTAINERS
do
    docker exec -t -i "$NODE" mkdir -p /tmp/experiments/
    docker exec -t -i "$NODE" chmod a+rw /tmp/experiments/
done

if [ -z ${ANALYZE_PERFORMANCE+x} ];
then
docker exec -t -i "$JOBMANAGER_CONTAINER" flink run -c "flink_experiments.BroadcastExperiment" \
    /job.jar --msg-size=$MSG_SIZE --num-receivers=$NUM_RECEIVERS --samples=$SAMPLES \
    --warmup-samples=$WARMUP_SAMPLES --frequency=$FREQUENCY \
    --output="/tmp/experiments/out.csv" --inter-process
else
docker exec -t -i "$JOBMANAGER_CONTAINER" flink run -c "flink_experiments.BroadcastExperiment" \
    /job.jar --msg-size=$MSG_SIZE --num-receivers=$NUM_RECEIVERS --samples=$SAMPLES \
    --warmup-samples=$WARMUP_SAMPLES --frequency=$FREQUENCY \
    --output="/tmp/experiments/out.csv" --inter-process --analyze-performance
fi

# Get result CSV
for NODE in $TASK_MANAGER_CONTAINERS
do
    FILES=$(docker exec -t -i "$NODE" ls -A /tmp/experiments/ | tr -d '\r')
    if [ -z "$FILES" ]
    then
        :
    else
        docker cp "$NODE:/tmp/experiments/out.csv" "$CSV_FILENAME"
    fi
    docker exec "$NODE" rm -rf /tmp/experiments
done

# Shut down cluster
docker-compose down
