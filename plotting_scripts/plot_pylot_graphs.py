# Copyright (c) 2018, Ionel Gog
import csv

from absl import app, flags
from matplotlib.patches import Patch

import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt

import numpy as np
import pandas as pd
import seaborn as sns

from utils import plot_box_and_whiskers, plot_cdf, plot_timeline, generate_ticks, set_paper_rcs
from pylot_utils import ProfileEvent, ProfileEvents, read_end_to_end_runtimes, \
    converter, get_timestamps_with_obstacles

FLAGS = flags.FLAGS
flags.DEFINE_enum('graph_type', 'cdf', ['cdf', 'box_and_whiskers', 'timeline'],
                  'Type of graph to plot')
flags.DEFINE_list('events', [], 'Events to select')
flags.DEFINE_bool('time_to_decision', False, 'True to plot time to decision')
flags.DEFINE_bool('lateral_acceleration', False,
                  'True to plot lateral acceleration')
flags.DEFINE_bool('longitudinal_acceleration', False,
                  'True to plot longitudinal acceleration')
flags.DEFINE_bool('lateral_jerk', False, 'True to plot lateral jerk')
flags.DEFINE_bool('longitudinal_jerk', False, 'True to plot longitudinal jerk')
flags.DEFINE_bool('obstacle_distance', False,
                  'True to plot distance to obstacle')
flags.DEFINE_bool('lane_deviation', False,
                  'True to plot cumulative deviation from lane center')
flags.DEFINE_bool('lane_invasion', False,
                  'True to plot cdf of time in lane invasion')
flags.DEFINE_bool('crosstrack_error', False, 'True to plot crossrack error')
flags.DEFINE_bool('heading_error', False, 'True to plot heading error')
flags.DEFINE_bool('small_paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('stretched', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('slide_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('poster_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_string('file_name', 'pylot', 'File to write plot to.')
flags.DEFINE_string('file_format', 'png', 'File type of the output plot.')
flags.DEFINE_string('xlabel', '', 'The xlabel to give the graph.')
flags.DEFINE_string('ylabel', '', 'The ylabel to give the graph.')
flags.DEFINE_list('labels', [], 'Labels of plotted data.')
flags.DEFINE_list('log_file_names', '', 'Pylot log files')
flags.DEFINE_list('csv_file_names', '', 'Pylot csv files')
flags.DEFINE_list('profile_file_names', '', 'Pylot profile files')
flags.DEFINE_integer('ignore_first_sim_time_ms', 1000,
                     'Ignore data from the first simulation time ms.')
flags.DEFINE_float('min_x_val', None, 'x axis starts from.')
flags.DEFINE_float('max_x_val', None, 'x axis goes up to.')
flags.DEFINE_float('min_y_val', None, 'y axis starts from.')
flags.DEFINE_float('max_y_val', None, 'y axis goes up to.')
flags.DEFINE_integer('x_ticks_increment', None,
                     'x axis increment. None if default should be used')
flags.DEFINE_integer('y_ticks_increment', None,
                     'y axis increment. None if default should be used')
flags.DEFINE_bool('legend_outside_graph', False,
                  'True to put legend outside box')
flags.DEFINE_bool('filter_only_timestamps_with_obstacles', False,
                  "True to only plot data for timestamps with obstacles")


def read_jerk_stats(csv_file_path):
    first_sim_time = None
    csv_file = open(csv_file_path)
    csv_reader = csv.reader(csv_file)
    lateral_jerk = []
    lateral_jerk_times = []
    longitudinal_jerk = []
    longitudinal_jerk_times = []
    for row in csv_reader:
        sim_time = int(row[1])
        if first_sim_time is None:
            first_sim_time = sim_time
        sim_time -= first_sim_time + FLAGS.ignore_first_sim_time_ms
        if (row[2] == 'jerk' and sim_time >= 0):
            jerk_type = row[3]
            if jerk_type == 'lateral':
                jerk = float(row[4])
                lateral_jerk.append(jerk)
                lateral_jerk_times.append(sim_time)
            elif jerk_type == 'longitudinal':
                jerk = float(row[4])
                longitudinal_jerk.append(jerk)
                longitudinal_jerk_times.append(sim_time)
            else:
                raise ValueError('Unexpected value in {}'.format(row))
    return (lateral_jerk, lateral_jerk_times, longitudinal_jerk,
            longitudinal_jerk_times)


def read_acceleration_stats(csv_file_path):
    first_sim_time = None
    csv_file = open(csv_file_path)
    csv_reader = csv.reader(csv_file)
    lateral_acceleration = []
    lateral_acc_times = []
    longitudinal_acceleration = []
    longitudinal_acc_times = []
    for row in csv_reader:
        sim_time = int(row[1])
        if not first_sim_time:
            first_sim_time = sim_time
        sim_time -= first_sim_time + FLAGS.ignore_first_sim_time_ms
        if (row[2] == 'acceleration' and sim_time >= 0):
            acc_type = row[3]
            if acc_type == 'lateral':
                acceleration = float(row[4])
                lateral_acceleration.append(acceleration)
                lateral_acc_times.append(sim_time)
            elif acc_type == 'longitudinal':
                acceleration = float(row[4])
                longitudinal_acceleration.append(acceleration)
                longitudinal_acc_times.append(sim_time)
            else:
                raise ValueError('Unexpected value in {}'.format(row))
    return (lateral_acceleration, lateral_acc_times, longitudinal_acceleration,
            longitudinal_acc_times)


def read_infraction_stats(csv_file_path):
    first_sim_time = None
    csv_file = open(csv_file_path)
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        proc_time = int(row[0])
        sim_time = int(row[1])
        if not first_sim_time:
            first_sim_time = sim_time
        sim_time -= first_sim_time + FLAGS.ignore_first_sim_time_ms
        event_name = row[2]
        if event_name == 'collision':
            actor_type = row[3]
            intensity = float(row[4])
            pass
        elif event_name == 'invasion':
            # red_light
            # sidewalk
            # lane
            invasion_type = row[3]
    # TODO: Implement.


def read_control_stats(csv_file_path):
    first_sim_time = None
    csv_file = open(csv_file_path)
    csv_reader = csv.reader(csv_file)
    sim_times = []
    crosstrack_err = []
    heading_err = []
    for row in csv_reader:
        sim_time = int(row[1])
        if not first_sim_time:
            first_sim_time = sim_time
        sim_time -= first_sim_time + FLAGS.ignore_first_sim_time_ms
        if (row[2] == 'control' and sim_time >= 0):
            sim_times.append(sim_time)
            crosstrack_err.append(float(row[3]))
            heading_err.append(float(row[4]))
    return (sim_times, crosstrack_err, heading_err)


def read_prediction_stats(csv_file_path):
    first_sim_time = None
    csv_file = open(csv_file_path)
    csv_reader = csv.reader(csv_file)
    sim_times = []
    msd = []
    ade = []
    fde = []
    person_msd = []
    person_ade = []
    person_fde = []
    vehicle_msd = []
    vehicle_ade = []
    vehicle_fde = []
    for row in csv_reader:
        sim_time = int(row[1])
        if not first_sim_time:
            first_sim_time = sim_time
        sim_time -= first_sim_time + FLAGS.ignore_first_sim_time_ms
        if (row[2] == 'prediction' and sim_time >= 0):
            sim_times.append(sim_time)
            if row[3] == 'MSD':
                msd.append(float(row[4]))
            elif row[3] == 'ADE':
                ade.append(float(row[4]))
            elif row[3] == 'FDE':
                fde.append(float(row[4]))
            elif row[3] == 'person-MSD':
                person_msd.append(float(row[4]))
            elif row[3] == 'person-ADE':
                person_ade.append(float(row[4]))
            elif row[3] == 'person-FDE':
                person_fde.append(float(row[4]))
            elif row[3] == 'vehicle-MSD':
                vehicle_msd.append(float(row[4]))
            elif row[3] == 'vehicle-ADE':
                vehicle_ade.append(float(row[4]))
            elif row[3] == 'vehicle-FDE':
                vehicle_fde.append(float(row[4]))
            else:
                raise ValueError('Unexpected row {}'.format(row))
    return (sim_times, msd, ade, fde, person_msd, person_ade, person_fde,
            vehicle_msd, vehicle_ade, vehicle_fde)


def get_obstacle_distances(filename, ignore_first_sim_time):
    df = pd.read_csv(
        filename,
        names=["timestamp", "ms", "log_label", "label_info", "label_value"])
    df = df.dropna()
    df['label_value'] = df['label_value'].str.replace(" ", ", ")
    df['label_value'] = df['label_value'].apply(converter)

    obstacles = df[df['log_label'] == 'obstacle']
    obstacles = obstacles.set_index('ms')
    pose = df[df['log_label'] == 'pose']

    timestamps = []
    distances = []
    first_timestamp = df["ms"].min()
    for t, p in pose[["ms", "label_value"]].values:
        adjusted_t = t - first_timestamp
        if t not in obstacles.index or adjusted_t <= ignore_first_sim_time:
            continue
        obs = obstacles.loc[t]['label_value']
        if isinstance(obs, list):
            obs = [obs]
        else:
            obs = obs.values
        min_dist = float('inf')
        for o in obs:
            dist = np.linalg.norm(np.array(p) - np.array(o))
            if dist > 0:
                min_dist = min(min_dist, dist)
        if min_dist != float('inf'):
            timestamps.append(adjusted_t - ignore_first_sim_time)
            distances.append(min_dist)

    return timestamps, distances


def get_lane_deviation(filename, lane_center, ignore_first_sim_time):
    df = pd.read_csv(
        filename,
        names=["timestamp", "ms", "log_label", "label_info", "label_value"])
    df = df.dropna()
    df['label_value'] = df['label_value'].str.replace(" ", ", ")
    df['label_value'] = df['label_value'].apply(converter)
    first_timestamp = df["ms"].min()
    df["ms"] = df["ms"] - first_timestamp
    df = df[df["ms"] > ignore_first_sim_time]
    first_timestamp = df["ms"].min()

    poses = df[df['log_label'] == 'pose']
    timestamps = (poses["ms"] - first_timestamp).to_numpy()
    poses = poses['label_value'].to_numpy()
    poses = np.array([np.array(x) for x in poses])
    y = poses[:-2, 1]
    deviation = np.cumsum(np.abs(y - lane_center))
    return timestamps[:-2], deviation


def get_lane_invasions(filename, lane_boundary, ignore_first_sim_time):
    df = pd.read_csv(
        filename,
        names=["timestamp", "ms", "log_label", "label_info", "label_value"])
    df = df.dropna()
    collision = len(df[df["log_label"] == "collision"]) > 0
    df['label_value'] = df['label_value'].str.replace(" ", ", ")
    df['label_value'] = df['label_value'].apply(converter)
    first_timestamp = df["ms"].min()
    df["ms"] = df["ms"] - first_timestamp
    df = df[df["ms"] > ignore_first_sim_time]
    first_timestamp = df["ms"].min()

    poses = df[df['log_label'] == 'pose']
    p = poses['label_value'].to_numpy()
    p = np.array([np.array(x) for x in p])
    y = p[:-2, 1]

    timestamps = poses.iloc[:-2][y > lane_boundary]["ms"]
    time_spent = 25
    if len(timestamps) > 1:
        time_spent = timestamps.iloc[-1] - timestamps.iloc[0]

    if collision:
        color = 'r'
    else:
        color = 'g'

    return time_spent, color


def main(argv):
    all_profile_events = [
        ProfileEvents(profile_file_name)
        for profile_file_name in FLAGS.profile_file_names
    ]
    if FLAGS.time_to_decision:
        all_ts_w_obstacles = []
        for csv_file_name in FLAGS.csv_file_names:
            ts = get_timestamps_with_obstacles(csv_file_name,
                                               obstacle_distance_threshold=20)
            all_ts_w_obstacles.append(ts)
        all_sim_times = []
        all_end_to_end_runtimes = []
        for ts, csv_file_name in zip(all_ts_w_obstacles, FLAGS.csv_file_names):
            if not FLAGS.filter_only_timestamps_with_obstacles:
                ts = None
            (sim_times, end_to_end_runtimes) = read_end_to_end_runtimes(
                csv_file_name, unit='ms', timestamps_with_obstacles=ts)
            all_sim_times.append(sim_times)
            all_end_to_end_runtimes.append(end_to_end_runtimes)
        if FLAGS.graph_type == 'cdf':
            plot_cdf(FLAGS,
                     FLAGS.file_name,
                     all_end_to_end_runtimes,
                     r'End-to-end runtime [$ms$]',
                     'CDF of end-to-end runtime',
                     FLAGS.labels,
                     show_legend=True,
                     legend_outside_box=FLAGS.legend_outside_graph,
                     x_ticks_increment=FLAGS.x_ticks_increment)
        elif FLAGS.graph_type == 'timeline':
            plot_timeline(FLAGS,
                          FLAGS.file_name,
                          all_sim_times,
                          all_end_to_end_runtimes,
                          r'Simulation Time [$s$]',
                          'End-to-end runtime',
                          FLAGS.labels,
                          show_legend=True,
                          legend_outside_box=FLAGS.legend_outside_graph,
                          cut_x_min=FLAGS.min_x_val,
                          cut_x_max=FLAGS.max_x_val,
                          x_ticks_increment=FLAGS.x_ticks_increment,
                          y_ticks_increment=FLAGS.y_ticks_increment,
                          cut_y_min=FLAGS.min_y_val,
                          cut_y_max=FLAGS.max_y_val,
                          scale_down_x_ticks=1000)

    if FLAGS.lateral_acceleration or FLAGS.longitudinal_acceleration:
        all_lateral_acc = []
        all_lateral_acc_times = []
        all_longitudinal_acc = []
        all_longitudinal_acc_times = []
        for csv_file_name in FLAGS.csv_file_names:
            (lateral_acc, lateral_acc_times, longitudinal_acc,
             longitudinal_acc_times) = read_acceleration_stats(csv_file_name)
            all_lateral_acc.append(lateral_acc)
            all_lateral_acc_times.append(lateral_acc_times)
            all_longitudinal_acc.append(longitudinal_acc)
            all_longitudinal_acc_times.append(longitudinal_acc_times)
        if FLAGS.lateral_acceleration:
            plot_timeline(FLAGS,
                          FLAGS.file_name,
                          all_lateral_acc_times,
                          all_lateral_acc,
                          r'Simulation Time [$s$]',
                          r'Lateral acceleration [$\frac{m}{s^2}$]',
                          FLAGS.labels,
                          show_legend=True,
                          legend_outside_box=FLAGS.legend_outside_graph,
                          cut_x_min=FLAGS.min_x_val,
                          cut_x_max=FLAGS.max_x_val,
                          x_ticks_increment=FLAGS.x_ticks_increment,
                          y_ticks_increment=FLAGS.y_ticks_increment,
                          cut_y_min=FLAGS.min_y_val,
                          cut_y_max=FLAGS.max_y_val,
                          line_width=1.0,
                          scale_down_x_ticks=1000)
        if FLAGS.longitudinal_acceleration:
            plot_timeline(FLAGS,
                          FLAGS.file_name,
                          all_longitudinal_acc_times,
                          all_longitudinal_acc,
                          r'Simulation Time [$s$]',
                          r'Longitudinal acceleration [$\frac{m}{s^2}$]',
                          FLAGS.labels,
                          show_legend=True,
                          legend_outside_box=FLAGS.legend_outside_graph,
                          cut_x_min=FLAGS.min_x_val,
                          cut_x_max=FLAGS.max_x_val,
                          x_ticks_increment=FLAGS.x_ticks_increment,
                          y_ticks_increment=FLAGS.y_ticks_increment,
                          cut_y_min=FLAGS.min_y_val,
                          cut_y_max=FLAGS.max_y_val,
                          line_width=1.0,
                          scale_down_x_ticks=1000)

    if FLAGS.lateral_jerk or FLAGS.longitudinal_jerk:
        all_lateral_jerk = []
        all_lateral_jerk_times = []
        all_longitudinal_jerk = []
        all_longitudinal_jerk_times = []
        for csv_file_name in FLAGS.csv_file_names:
            (lateral_jerk, lateral_jerk_times, longitudinal_jerk,
             longitudinal_jerk_times) = read_jerk_stats(csv_file_name)
            all_lateral_jerk.append(lateral_jerk)
            all_lateral_jerk_times.append(lateral_jerk_times)
            all_longitudinal_jerk.append(longitudinal_jerk)
            all_longitudinal_jerk_times.append(longitudinal_jerk_times)

        if FLAGS.lateral_jerk:
            import brewer2mpl
            bmap = brewer2mpl.get_map('Set2', 'qualitative', 7)
            colors = bmap.mpl_colors
            jerk_data = []
            set_paper_rcs()
            plt.figure(figsize=(2.4, 1.66))
            print(all_lateral_jerk)
            for i in range(3):
                jerk_data.append(
                    pd.DataFrame({
                        'jerk': [abs(x) for x in all_lateral_jerk[i]],
                        'deadline': FLAGS.labels[i]
                    }))
            data = pd.concat(jerk_data)
            ax = sns.boxplot(
                y='jerk',
                x='deadline',
                data=data,
                width=0.7,
                showfliers=True,
                color=colors[6],
                fliersize=0.9,
                whis=(5, 95),
                saturation=1,
            )
            plt.ylim(0, 141)
            plt.yticks([x for x in range(0, 121, 40)],
                       [str(x) for x in range(0, 121, 40)])
            plt.ylabel("Comfort\nAbs. Lateral Jerk [$m/s^3$]")
            plt.xlabel(r"Planning runtime [ms]")

            #showfliers=True,
            #fliersize=0.7)
            plt.savefig('planning-absolute-jerk.{}'.format(FLAGS.file_format),
                        bbox_inches='tight')

            plot_timeline(FLAGS,
                          FLAGS.file_name,
                          all_lateral_jerk_times,
                          all_lateral_jerk,
                          r'Simulation Time [$s$]',
                          r'Lateral jerk [$m/s^3$]',
                          FLAGS.labels,
                          show_legend=True,
                          legend_outside_box=FLAGS.legend_outside_graph,
                          cut_x_min=FLAGS.min_x_val,
                          cut_x_max=FLAGS.max_x_val,
                          colors=colors,
                          x_ticks_increment=FLAGS.x_ticks_increment,
                          y_ticks_increment=FLAGS.y_ticks_increment,
                          cut_y_min=FLAGS.min_y_val,
                          cut_y_max=FLAGS.max_y_val,
                          line_width=0.5,
                          scale_down_x_ticks=1000)
        if FLAGS.longitudinal_jerk:
            plot_timeline(FLAGS,
                          FLAGS.file_name,
                          all_longitudinal_jerk_times,
                          all_longitudinal_jerk,
                          r'Simulation Time [$s$]',
                          r'Longitudinal jerk [$\frac{m}{s^3}$]',
                          FLAGS.labels,
                          show_legend=True,
                          legend_outside_box=FLAGS.legend_outside_graph,
                          cut_x_min=FLAGS.min_x_val,
                          cut_x_max=FLAGS.max_x_val,
                          x_ticks_increment=FLAGS.x_ticks_increment,
                          y_ticks_increment=FLAGS.y_ticks_increment,
                          cut_y_min=FLAGS.min_y_val,
                          cut_y_max=FLAGS.max_y_val,
                          line_width=1.0,
                          scale_down_x_ticks=1000)

    if FLAGS.obstacle_distance:
        all_ts = []
        all_dists = []
        for csv_file_name in FLAGS.csv_file_names:
            ts, dist = get_obstacle_distances(csv_file_name,
                                              FLAGS.ignore_first_sim_time_ms)
            all_ts.append(ts)
            all_dists.append(dist)
        plot_timeline(FLAGS,
                      FLAGS.file_name,
                      all_ts,
                      all_dists,
                      r'Simulation Time [$s$]',
                      r'Distance to closest obstacle [$m$]',
                      FLAGS.labels,
                      show_legend=True,
                      legend_outside_box=FLAGS.legend_outside_graph,
                      cut_x_min=FLAGS.min_x_val,
                      cut_x_max=FLAGS.max_x_val,
                      x_ticks_increment=FLAGS.x_ticks_increment,
                      y_ticks_increment=FLAGS.y_ticks_increment,
                      cut_y_min=FLAGS.min_y_val,
                      cut_y_max=FLAGS.max_y_val,
                      line_width=1.0,
                      scale_down_x_ticks=1000)

    if FLAGS.lane_deviation:
        all_ts = []
        all_deviation = []
        for csv_file_name in FLAGS.csv_file_names:
            ts, deviation = get_lane_deviation(csv_file_name, 326.61,
                                               FLAGS.ignore_first_sim_time_ms)
            all_ts.append(ts)
            all_deviation.append(deviation)
        plot_timeline(FLAGS,
                      FLAGS.file_name,
                      all_ts,
                      all_deviation,
                      r'Simulation Time [$s$]',
                      r'Cumulative Deviation from Lane Center [$m$]',
                      FLAGS.labels,
                      show_legend=True,
                      legend_outside_box=FLAGS.legend_outside_graph,
                      cut_x_min=FLAGS.min_x_val,
                      cut_x_max=FLAGS.max_x_val,
                      x_ticks_increment=FLAGS.x_ticks_increment,
                      y_ticks_increment=FLAGS.y_ticks_increment,
                      cut_y_min=FLAGS.min_y_val,
                      cut_y_max=FLAGS.max_y_val,
                      line_width=1.0,
                      scale_down_x_ticks=1000)

    if FLAGS.lane_invasion:
        all_invasion = []
        all_collisions = []
        for csv_file_name in FLAGS.csv_file_names:
            invasion, collision = get_lane_invasions(
                csv_file_name, 327, FLAGS.ignore_first_sim_time_ms)
            all_invasion.append(invasion)
            all_collisions.append(collision)
        plt.barh(y=[i for i in range(len(all_invasion))],
                 width=all_invasion,
                 height=0.5,
                 color=all_collisions,
                 tick_label=FLAGS.labels)
        plt.xlabel(r"Total Time in Oncoming Lane [$s$]")
        plt.xlim(FLAGS.min_x_val, FLAGS.max_x_val)
        ts, tls = generate_ticks(FLAGS.min_x_val, FLAGS.max_x_val,
                                 FLAGS.x_ticks_increment, 1000)
        plt.xticks(ts, tls)
        ax = plt.gca()
        legend_elements = [
            Patch(facecolor='red', edgecolor='r', label='Failure'),
            Patch(facecolor='green', edgecolor='g', label='Success')
        ]
        ax.legend(handles=legend_elements)
        plt.savefig("{}.{}".format(FLAGS.file_name, FLAGS.file_format),
                    format=FLAGS.file_format,
                    bbox_inches="tight")

    if FLAGS.crosstrack_error or FLAGS.heading_error:
        for csv_file_name in FLAGS.csv_file_names:
            (sim_times, crosstrack_err,
             heading_err) = read_control_stats(csv_file_name)
        raise NotImplementedError

    if FLAGS.graph_type == 'cdf' and len(FLAGS.events) > 0:
        # assert len(FLAGS.labels) == len(FLAGS.events) * len(
        #     all_profile_events), 'All events must be labelled'
        all_ts_w_obstacles = []
        for csv_file_name in FLAGS.csv_file_names:
            ts = get_timestamps_with_obstacles(csv_file_name,
                                               obstacle_distance_threshold=20)
            all_ts_w_obstacles.append(ts)
        all_runtimes = []
        for ts, profile_events in zip(all_ts_w_obstacles, all_profile_events):
            if not FLAGS.filter_only_timestamps_with_obstacles:
                ts = None
            for event_name in FLAGS.events:
                runtimes = profile_events.get_runtimes(
                    event_name, unit='us', timestamps_with_obstacles=ts)
                all_runtimes.append(runtimes)
        plot_cdf(FLAGS,
                 FLAGS.file_name,
                 all_runtimes,
                 FLAGS.xlabel,
                 FLAGS.ylabel,
                 FLAGS.labels,
                 show_legend=True,
                 legend_outside_box=FLAGS.legend_outside_graph,
                 x_ticks_increment=FLAGS.x_ticks_increment,
                 scale_x_axis=1000)
    elif FLAGS.graph_type == 'timeline' and len(FLAGS.events) > 0:
        assert len(FLAGS.labels) == len(FLAGS.events) * len(
            all_profile_events), 'All events must be labelled'
        all_sim_times = []
        all_runtimes = []
        for profile_events in all_profile_events:
            for event_name in FLAGS.events:
                sim_times, runtimes = profile_events.get_timeline(event_name,
                                                                  unit='ms')
            all_sim_times.append(sim_times)
            all_runtimes.append(runtimes)
        plot_timeline(FLAGS,
                      FLAGS.file_name,
                      all_sim_times,
                      all_runtimes,
                      FLAGS.xlabel,
                      FLAGS.ylabel,
                      FLAGS.labels,
                      show_legend=True,
                      legend_outside_box=FLAGS.legend_outside_graph,
                      cut_x_min=FLAGS.min_x_val,
                      cut_x_max=FLAGS.max_x_val,
                      x_ticks_increment=FLAGS.x_ticks_increment,
                      cut_y_min=FLAGS.min_y_val,
                      cut_y_max=FLAGS.max_y_val)


if __name__ == '__main__':
    app.run(main)
