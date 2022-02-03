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

If you want to replicate our experiments, please ensure that you have updated
Pylot and are using the deadlines branch in the ERDOS repository. You can do
this by running the following commands:

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/erdos
pip3 uninstall erdos
git pull
git checkout -b deadlines origin/deadlines
python3 python/setup.py install --user
```

### Measuring the adaptability of the detection and planning components ([Figure 9](plotting_scripts/graphs/pylot-mode-changes.pdf))
In order to measure how well the detection and planning components adapt to
frequent deadline changes you have to execute the following commands:

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
# Checkout the branch, which contains the code to conduct frequent mode changes.
git checkout -b mode_changes origin/mode_changes
cd ~/workspace/
# Run Pylot without a policy that randomly adjusts the deadlines of the detection and planning components.
./leaderboard/scripts/run_mode_change_experiment.sh
```

The experiment will generates logs in `~/workspace/logs_mode_change_town_1_route_2_timely_True_run_1`,
which can be plotted by running the following commands outside the container:

```console
mkdir -p ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/mode_changes
docker cp pylot:/home/erdos/workspace/logs_mode_change_town_1_route_2_timely_True_run_1 ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/mode_changes
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
python3 plot_mode_changes.py --base_dir=../reproduced_experiments/mode_changes --file_format=pdf --paper_mode
```

###  Measuring the overhead introduced by a no-op policy
This experiment compares the response time of Pylot configured without a
deadline policy to the response time of Pylot with a no-op policy. The goal of
the experiment is to measure the overhead introduced by the mechanism for
implementing policies. In order to conduct the experiment, you have to run:

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot
# Checkout the branch, which contains the Pylot code without a policy.
git checkout -b no-ttd origin/no-ttd
cd ~/workspace/
# Run Pylot without a policy for adjusting deadlines.
./leaderboard/scripts/run_nopolicy_experiments.sh
cd ~/workspace/pylot
# Checkout the branch, which contains the Pylot code with a no-op policy.
git checkout -b deadlines origin/deadlines
# Run Pylot with a no-op policy.
./leaderboard/scripts/run_policy_experiments.sh dedicated
mkdir -p ~/workspace/reproduced_experiments/challenge/comparison_w_and_wo_policy
mv ~/workspace/logs_*policy* ~/workspace/reproduced_experiments/challenge/comparison_w_and_wo_policy/
```

Next, generate the graphs and response time statistics by running the
following commands:

```console
# In another terminal, copy the logs from the container.
mkdir -p ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/challenge/comparison_w_and_wo_policy/
docker cp pylot:/home/erdos/workspace/reproduced_experiments/challenge/comparison_w_and_wo_policy ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/challenge/
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
# Execute script that will analyze the log and print out the distribution of the end-to-end response time.
python3 plot_challenge_policy.py --base_dir=../reproduced_experiments/challenge/comparison_w_and_wo_policy/ --file_format=pdf --small_paper_mode --plot_runtimes --towns=1 --start_route=1 --end_route=9 --num_reps=5 --verbose
```

###  Measuring the effect of deadline exception handlers on response time ([Figure 10](plotting_scripts/graphs/deadline-bbox.pdf))
In this experiment, the self-driving car drives in a 70km route with static
deadlines per-component. The goal of the experiment is to show that the deadline
exception handlers execute quickly, and thus ensure that the components always
send output shortly after their deadline. Please run the following commands in
order to execute the experiment:

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b deadlines origin/deadlines
cd ~/workspace/
# Run Pylot with static deadlines.
./leaderboard/scripts/run_deadline_experiments.sh dedicated
mkdir -p ~/workspace/reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar
mv ~/workspace/logs_deadlines_* ~/workspace/reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar
```

Following, execute Pylot using the **data-driven execution model**, which does
not enforce deadlines:

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b no-deadlines origin/no-deadlines
cd ~/workspace/
# Run Pylot in data-driven execution model (i.e., without deadlines).
./leaderboard/scripts/run_timely_experiments.sh
mv ~/workspace/logs_town_* ~/workspace/reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar
```

Next, generate the graph by executing the following commands outside of the
container:

```console
# Copy the logs from the container.
mkdir -p ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar
docker cp pylot:/home/erdos/workspace/reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/challenge/
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
python3 plot_challenge_deadline_paper.py --base_dir=../experiments/challenge/deadlines_comparison_with_accurate_lidar/ --file_format=png --paper_mode --towns=1 --start_route=1 --end_route=9 --num_reps=7 --file_format=pdf --small_paper_mode
```

###  Measuring collisions while using different execution models ([Figure 11](plotting_scripts/graphs/pylot_collisions.pdf))
This experiment compares the periodic and data-driven execution models with
the executions enforcing static deadlines and dynamic deadlines. The logs for
the **data-driven** and **static deadlines** execution models were already
generated in the experiment measuring the effect of deadline exception
handlers (see [here](#measuring-the-effect-of-deadline-exception-handlers-on-response-time-figure-10)
in case you need to run the experiment).

Please run the following commands in order to generate the logs of Pylot running
atop of the **periodic** execution model:

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b frequency-execution origin/frequency-execution
cd ~/workspace/
# Run Pylot using a frequency-driven execution model.
./leaderboard/scripts/run_frequency_experiments.sh
mv ~/workspace/logs_frequency_* ~/workspace/reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar
```

Following, please run the following commands in order to obtain the logs
generated by Pylot executing using a simple policy that
**dynamically adapts deadlines**:

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b deadlines-ttd origin/deadlines-ttd
cd ~/workspace/
# Run Pylot with a simple deadline allocation policy.
./leaderboard/scripts/run_policy_experiments.sh dedicated
mv ~/workspace/logs_policy_* ~/workspace/reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar
```

Next, generate collision statistics and graphs by running the following
commands:

```console
# Copy the logs from the container.
mkdir -p ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar
docker cp pylot:/home/erdos/workspace/reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/challenge/
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
# Generate collision statistics.
python3 generate_collision_stats.py --base_dir=../reproduced_experiments/challenge/deadlines_comparison_with_accurate_lidar/ --filter_carla_cola=True --towns=1 --start_route=1 --end_route=9 --num_reps=7
python3 plot_challenge_collisions_barchart.py
```

### Generating the response time histograph from logged data ([Figure 12](plotting_scripts/graphs/configurations-e2e-runtime-hist.pdf))
Please run the following commands in order to generate the histogram comparing
the end-to-end response time of Pylot's executions with static and dynamic
deadlines:

```console
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
python3 plot_challenge_e2e_response_time.py --base_dir=../reproduced_experiments/challenge/deadlines_comparison/ --end_route=9 --num_reps=7 --file_format=pdf --small_paper_mode --plot_histogram
```

### Measuring collision speed across across different scenarios ([Figure 13](plotting_scripts/graphs/speed-deadline-dd-heatmap.pdf))
Please run the following commands in order to reproduce the experiment:

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout 1518c0a14ff1dc66a3c2cfd38862ca468084ec5c
cd ~/workspace/
git clone https://github.com/erdos-project/erdos-experiments.git
cd ~/workspace/erdos-experiments/experiments/scenario_runner
# Run the Traffic Jam and Person Behind Truck scenarios using different Pylot configurations with static deadlines.
./run_static_deadlines_traffic_jam.sh
./run_static_deadlines_person_behind_car.sh
cd ~/workspace/pylot/
git checkout -b deadlines-ttd origin/deadlines-ttd
cd ~/workspace/erdos-experiments/experiments/scenario_runner
# Run the Traffic Jam and Person Behind Truck scenarios using different Pylot configurations with dynamic deadlines.
./run_dynamic_deadlines_traffic_jam.sh
./run_dynamic_deadlines_person_behind_car.sh
```

In another terminal, copy the log data from the container in to the experiments
directory:

```console
mkdir -p ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/scenario_runner/scenarios_collision/
docker cp pylot:/home/erdos/workspace/pylot/traffic_jam* ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/scenario_runner/scenarios_collision/
docker cp pylot:/home/erdos/workspace/pylot/person_behind_car* ${ERDOS_EXPERIMENTS_HOME}/reproduced_experiments/scenario_runner/scenarios_collision/
```

### Generating timelines of the response time during a drive in a scenario ([Figure 14](plotting_scripts/graphs/dynamic_deadline_timeline.pdf))
Please execute the following script in order to reproduce the timelines
comparing the end-to-end response time of the executions with static and
dynamic deadlines:

```console
cd ${ERDOS_EXPERIMENTS_HOME}/plotting_scripts
python3 plot_scenario_response_time_timeline.py --static_log_base=../reproduced_experiments/scenario_runner/person_behind_car_response_time/person_behind_car_detection_200_planning_309_target_speed_12_Hz_5 --dynamic_log_base=../reproduced_experiments/scenario_runner/person_behind_car_response_time/person_behind_car_dynamic_deadlines_target_speed_12_Hz_5
```
