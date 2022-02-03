This repository contains scripts to download our experiment data, to reproduce
our figures, and instructions on how to reproduce our experiments.

# Downloading log data of experiments and generating graphs

```console
./download_data.sh
# Verify that the experiments directory contains log data of our experiments.
# Install dependencies required to run the plotting scripts.
pip3 install -r requirements.txt
cd plotting_scripts
./generate_plots.sh
# The graphs will be generated in plotting_scripts/graphs.
```

# Executing experiments

## Executing system experiments

## Executing autonomous driving experiments

### Configuring ERDOS, Pylot, and the CARLA simulator

In order to evaluate the efficacy of ERDOS, we deveploed Pylot, an autonomous
vehicle that runs both in simulation and on real-vehicles. The easiest way to
get Pylot running is to use our Docker image. Please ensure you have
`nvidia-docker` on your machine before you start installing Pylot.
In case you do not have `nvidia-docker` please download
[install-nvidia-docker.sh](https://github.com/erdos-project/pylot/blob/master/scripts/install-nvidia-docker.sh)
and execute it.

We provide a Docker image with ERDOS, Pylot and CARLA simulator already setup.

```console
docker pull erdosproject/pylot
nvidia-docker run -itd --name pylot -p 20022:22 erdosproject/pylot /bin/bash
```
To test if the image runs on your hardware, please start the simulator in the
container:

```console
nvidia-docker exec -i -t pylot /home/erdos/workspace/pylot/scripts/run_simulator.sh
```

In another terminal, start Pylot using the container:

```console
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
# Execute a version of Pylot that uses a Faster-RCNN object detector.
python3 pylot.py --flagfile=configs/detection.conf
```

Following, you can inspect `~/workspace/pylot/pylot.log` to see if the simulator
and Pylot are progressing. Moreover, you can visualize the outputs of different
components (e.g., bounding boxes for detected objects) by forwarding X from the
container. In order to do so, you have to first add your public ssh key to the
`~/.ssh/authorized_keys` in the container:

```console
nvidia-docker cp ~/.ssh/id_rsa.pub pylot:/home/erdos/.ssh/authorized_keys
nvidia-docker exec -i -t pylot sudo chown erdos /home/erdos/.ssh/authorized_keys
nvidia-docker exec -i -t pylot sudo service ssh start
```
Finally, ssh into the container with X forwarding:
```console
ssh -p 20022 -X erdos@localhost
cd /home/erdos/workspace/pylot/
python3 pylot.py --flagfile=configs/detection.conf --visualize_detected_obstacles
```

If everything worked ok, you should be able to see a visualization like
the one below:

![Pylot obstacle detection](https://github.com/erdos-project/pylot/raw/master/doc/source/images/pylot-obstacle-detection.png)

## Executing experiments using complex driving scenarios

Hardware requirements: all experiments were performed on a machine
having 2 x Xeon Gold 6226 CPUs, 128GB of RAM, and 2 x 24GB Titan-RTX GPUs,
running Linux Kernel 5.3.0.

**Important note:** executing the experiments on different hardware would
could significantly affect the results as any differences in end-to-end response
would cause the simulation experiments to diverse from the simulations we
executed (e.g., an increased response time might cause an early collision,
but avoid later collissions due to divergence between simulations).

We highlight that due to limitations in the simulator's performance,
executing 1 second of simulation takes approximately 10-30 wallclock time
seconds. Thus, in order to address the issue that executing our experiments
takes approximately a month, we provide the logs of all our experiments together
with scripts to analyze them and re-generate all our figures:

```console
./download_data.sh
# Verify that the experiments directory contains log data of our experiments.
# Install dependencies required to run the plotting scripts.
pip3 install -r requirements.txt
cd plotting_scripts
./generate_plots.sh
# The graphs will be generated in plotting_scripts/graphs.
```

### Measuring the adaptability of the detection and planning components ([Figure 9](plotting_scripts/graphs/pylot-mode-changes.pdf))

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/

git checkout -b mode_changes origin/mode_changes

# Change directory the CARLA leaderboard repository, which generates complex driving scenarios.
cd ~/workspace/
# Runs Pylot without a policy that randomly adjusts the deadlines of the detection and planning components.
./leaderboard/scripts/run_mode_change_experiment.sh
```

The experiment will generates logs in `~/workspace/logs_mode_change_town_1_route_2_timely_True_run_1`, which can be plotted by running the following commands outside the container:

```console
mkdir -p ${ERDOS_EXPERIMENTS_HOME}/new_experiments/mode_changes
docker cp pylot:/home/erdos/workspace/logs_mode_change_town_1_route_2_timely_True_run_1 ${ERDOS_EXPERIMENTS_HOME}/new_experiments/mode_changes
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
python3 plot_mode_changes.py --base_dir=../new_experiments/mode_changes --file_format=pdf --paper_mode
```

###  Measuring the overhead introduced by a no-op policy

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/
# Runs Pylot without a policy for adjusting deadlines.
./leaderboard/scripts/run_nopolicy_experiments.sh
# Runs Pylot with a no-op policy.
./leaderboard/scripts/run_policy_experiments.sh dedicated
mkdir -p ~/workspace/new_experiments/challenge/comparison_w_and_wo_policy
mv ~/workspace/logs_*policy* ~/workspace/new_experiments/challenge/comparison_w_and_wo_policy/
```

Next, generate the graphs and response time statistics by running the
following commands:

```console
# Copy the logs from the container.
mkdir -p ${ERDOS_EXPERIMENTS_HOME}/new_experiments/challenge/comparison_w_and_wo_policy/
docker cp pylot:/home/erdos/workspace/new_experiments/challenge/comparison_w_and_wo_policy ${ERDOS_EXPERIMENTS_HOME}/new_experiments/challenge/
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
python3 plot_challenge_policy.py --base_dir=../new_experiments/challenge/comparison_w_and_wo_policy/ --file_format=pdf --small_paper_mode --plot_runtimes --towns=1 --start_route=1 --end_route=9 --num_reps=5 --verbose
```

###  Measuring the effect of deadline exception handlers on response time ([Figure 10](plotting_scripts/graphs/deadline-bbox.pdf))

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b deadlines origin/deadlines
# Change directory the CARLA leaderboard repository, which generates complex driving scenarios.
cd ~/workspace/
# Runs Pylot with static deadlines.
./leaderboard/scripts/run_deadline_experiments.sh
mkdir -p ~/workspace/new_experiments/challenge/deadlines_comparison_with_accurate_lidar
mv ~/workspace/logs_deadlines_* ~/workspace/new_experiments/challenge/deadlines_comparison_with_accurate_lidar
```

Next, generate the graph by executing the following commands outside of the container:

```console
# Copy the logs from the container.
mkdir -p ${ERDOS_EXPERIMENTS_HOME}/new_experiments/challenge/deadlines_comparison_with_accurate_lidar
docker cp pylot:/home/erdos/workspace/new_experiments/challenge/deadlines_comparison_with_accurate_lidar ${ERDOS_EXPERIMENTS_HOME}/new_experiments/challenge/
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
python3 plot_challenge_deadline_paper.py --base_dir=../experiments/challenge/deadlines_comparison_with_accurate_lidar/ --file_format=png --paper_mode --towns=1 --start_route=1 --end_route=9 --num_reps=7 --file_format=pdf --small_paper_mode
```

###  Measuring collisions while using different execution models ([Figure 11](plotting_scripts/graphs/pylot_collisions.pdf))

Comparing periodic and data-driven execution models with the executions
enforcing static deadlines and dynamic deadlines.

**Data-driven execution model**:
```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b no-deadlines origin/no-deadlines
# Change directory the CARLA leaderboard repository, which generates complex driving scenarios.
cd ~/workspace/
./leaderboard/scripts/run_timely_experiments.sh
```

**Periodic execution model**:
```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b frequency-execution origin/frequency-execution
# Change directory the CARLA leaderboard repository, which generates complex driving scenarios.
cd ~/workspace/
./leaderboard/scripts/
```

The logs for the execution model using **static deadlines** were already
generated by the [experiment](#measuring-the-effect-of-deadline-exception-handlers-on-response-time-figure-10)
measuring the effect of deadline exception handlers. Similarly, the logs for
the execution using a simple policy that **dynamically adapts deadlines** were
already generated by the [experiment](#measuring-the-overhead-introduced-by-a-no-op-policy)
measuring the response time overhead introduced by the policy.

### Generating the response time histograph from logged data ([Figure 12](plotting_scripts/graphs/configurations-e2e-runtime-hist.pdf))

```console
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
python3 plot_challenge_e2e_response_time.py --base_dir=../new_experiments/challenge/deadlines_comparison/ --end_route=9 --num_reps=7 --file_format=pdf --small_paper_mode --plot_histogram
```

### Measuring collision speed across across different scenarios ([Figure 13](plotting_scripts/graphs/speed-deadline-dd-heatmap.pdf))

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout master
cd ~/workspace/scenario_runner/
```

### Generating timelines of the response time during a drive in a scenario ([Figure 14](plotting_scripts/graphs/dynamic_deadline_timeline.pdf))

```console
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
python3 plot_scenario_response_time_timeline.py --static_log_base=../new_experiments/scenario_runner/person_behind_car_response_time/person_behind_car_detection_200_planning_309_target_speed_12_Hz_5 --dynamic_log_base=../new_experiments/scenario_runner/person_behind_car_response_time/person_behind_car_dynamic_deadlines_target_speed_12_Hz_5
```
<!-- ```console -->
<!-- cd ????? -->
<!-- git checkout -b no-ttd origin/no-ttd -->
<!-- ``` -->
