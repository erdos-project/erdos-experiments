#!/bin/bash
docker run -it -h `hostname` --mount type=bind,src="$(pwd)/../",dst=/experiments \
    ros:galactic /bin/bash -c "cd /experiments/ros2-experiments;
    . scripts/setup_container.sh;
    . scripts/broadcast_latency_intra_process.sh"
