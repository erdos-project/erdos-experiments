#!/bin/bash
docker run -it -h `hostname` --mount type=bind,src="$(pwd)/../",dst=/experiments \
    ros:noetic /bin/bash -c "cd /experiments/ros-experiments;
    . scripts/setup_container.sh;
    . scripts/msg_size_latency.sh"
