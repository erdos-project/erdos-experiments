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
cd ${ERDOS_EXPERIMENTS_HOME}/new_experiments/mode_changes
docker cp pylot:/home/erdos/workspace/logs_mode_change_town_1_route_2_timely_True_run_1 ./
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
```

###  Measuring the effect of deadline exception handlers on response time ([Figure 10](plotting_scripts/graphs/deadline-bbox.pdf))

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b deadlines origin/deadlines
# Change directory the CARLA leaderboard repository, which generates complex driving scenarios.
cd ~/workspace/
./leaderboard/scripts/
```

###  Measuring collisions while using different execution models ([Figure 11](plotting_scripts/graphs/pylot_collisions.pdf))

Comparing periodic and data-driven execution models with the executions
enforcing static deadlines and dynamic deadlines.

Data-driven execution model:
```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b no-deadlines origin/no-deadlines
# Change directory the CARLA leaderboard repository, which generates complex driving scenarios.
cd ~/workspace/
./leaderboard/scripts/
```

Periodic execution model:
```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b frequency-execution origin/frequency-execution
# Change directory the CARLA leaderboard repository, which generates complex driving scenarios.
cd ~/workspace/
./leaderboard/scripts/
```

Execution using static deadlines:
```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b deadlines origin/deadlines
# Change directory the CARLA leaderboard repository, which generates complex driving scenarios.
cd ~/workspace/
./leaderboard/scripts/
```

Execution using a simple policy that dynamically adapts deadlines:
```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout -b deadlines-ttd origin/deadlines-ttd
# Change directory the CARLA leaderboard repository, which generates complex driving scenarios.
cd ~/workspace/
./leaderboard/scripts/
```

### Generating the response time histograph from logged data ([Figure 12](plotting_scripts/graphs/configurations-e2e-runtime-hist.pdf))


### Measuring collision speed across across different scenarios ([Figure 13](plotting_scripts/graphs/speed-deadline-dd-heatmap.pdf))

```console
# Connect to the Pylot container.
nvidia-docker exec -i -t pylot /bin/bash
cd ~/workspace/pylot/
git checkout master
cd ~/workspace/scenario_runner/
```

### Generating timelines of the response time during a drive in a scenario ([Figure 14](plotting_scripts/graphs/dynamic_deadline_timeline.pdf))

<!-- ```console -->
<!-- cd ????? -->
<!-- git checkout -b no-ttd origin/no-ttd -->
<!-- ``` -->
