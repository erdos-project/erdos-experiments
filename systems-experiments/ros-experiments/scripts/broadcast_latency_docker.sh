#!/bin/bash
docker run -it -h `hostname` --mount type=bind,src="$(pwd)/../",dst=/experiments \
    ros:melodic /bin/bash -c "cd /experiments/ros-experiments;
    . scripts/setup_container.sh;
    . scripts/broadcast_latency.sh"
