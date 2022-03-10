import csv
import math

from absl import app, flags

import matplotlib

matplotlib.use("agg")
import matplotlib.pyplot as plt

import numpy as np

from pylot_utils import ProfileEvent, ProfileEvents, fix_pylot_profile, \
    read_challenge_runtimes, read_challenge_deadline_misses

from utils import setup_plot

import pandas as pd

import seaborn as sns

from utils import *

from pylot_color_utils import get_colors

FLAGS = flags.FLAGS
flags.DEFINE_string('base_dir', '', 'Path to the base dir where the logs are')
flags.DEFINE_bool('small_paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('stretched', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('slide_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('poster_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_string('file_format', 'png', 'File type of the output plot.')
flags.DEFINE_integer('ignore_first_sim_time_ms', 0,
                     'Ignore data from the first simulation time ms.')
flags.DEFINE_integer('start_route', 1, 'Id of the first completed route.')
flags.DEFINE_integer('end_route', 9, 'Id of the last completed route.')
flags.DEFINE_integer('num_reps', 5, 'Number of experiment repetitions.')
flags.DEFINE_list('towns', ['1'], 'List of towns.')
flags.DEFINE_integer('e2e_deadline', 269, 'End-to-end deadline.')
flags.DEFINE_integer('detection_deadline', 110, 'Detection deadline.')
flags.DEFINE_integer('tracker_deadline', 15, 'Tracker deadline.')
flags.DEFINE_integer('loc_finder_deadline', 54, 'Location finder deadline.')
flags.DEFINE_integer('prediction_deadline', 25, 'Prediction deadline.')
flags.DEFINE_integer('planning_deadline', 65, 'Planning deadline.')


def read_challenge_results(log_dir_base, town, route, detector, num_reps,
                           with_deadlines):
    e2e_runtimes = []
    detector_runtimes = []
    loc_finder_runtimes = []
    tracker_runtimes = []
    prediction_runtimes = []
    planning_runtimes = []
    num_detection_misses = num_tracker_misses = num_loc_finder_misses = 0
    num_prediction_misses = num_planning_misses = 0
    for run in range(1, num_reps + 1):
        log_dir = '{}_run_{}/'.format(log_dir_base, run)
        result_file = log_dir + 'results.json'
        csv_file = log_dir + 'challenge.csv'
        profile_file = log_dir + 'challenge.json'
        log_file = log_dir + 'challenge.log'
        # Get the runtimes
        fix_pylot_profile(profile_file)
        profile_events = ProfileEvents(profile_file, no_offset=True)
        # profile_events.check_if_timestamps_overlapped()

        # Get the end-to-end runtimes.
        (sim_times, run_e2e, run_e2e_w_sensor,
         _) = read_challenge_runtimes(csv_file)
        e2e_runtimes = e2e_runtimes + run_e2e

        detection_miss = tracker_miss = loc_finder_miss = prediction_miss = planning_miss = None
        if with_deadlines:
            (detection_miss, tracker_miss, loc_finder_miss, prediction_miss,
             planning_miss) = read_challenge_deadline_misses(log_file)
            num_detection_misses += len(detection_miss)
            num_tracker_misses += len(tracker_miss)
            num_loc_finder_misses += len(loc_finder_miss)
            num_prediction_misses += len(prediction_miss)
            num_planning_misses += len(planning_miss)

        run_detector_runtimes = profile_events.get_filtered_runtimes(
            'efficientdet_operator.on_watermark',
            timestamps_to_ban=detection_miss)
        run_loc_finder_runtimes = profile_events.get_filtered_runtimes(
            'center_camera_location_finder_history_operator.on_watermark',
            timestamps_to_ban=loc_finder_miss)
        run_tracker_runtimes = profile_events.get_filtered_runtimes(
            'tracker_sort.on_watermark', timestamps_to_ban=tracker_miss)
        run_prediction_runtimes = profile_events.get_filtered_runtimes(
            'linear_prediction_operator.generate_predicted_trajectories',
            timestamps_to_ban=prediction_miss)
        if len(run_prediction_runtimes) == 0:
            run_prediction_runtimes = profile_events.get_filtered_runtimes(
                'linear_prediction_operator.on_watermark',
                timestamps_to_ban=prediction_miss)
        run_planning_runtimes = profile_events.get_filtered_runtimes(
            'planning_operator.on_watermark', timestamps_to_ban=planning_miss)

        detector_runtimes = detector_runtimes + run_detector_runtimes
        loc_finder_runtimes = loc_finder_runtimes + run_loc_finder_runtimes
        tracker_runtimes = tracker_runtimes + run_tracker_runtimes
        prediction_runtimes = prediction_runtimes + run_prediction_runtimes
        planning_runtimes = planning_runtimes + run_planning_runtimes

    entries = len(e2e_runtimes)
    num_e2e_misses = len([x for x in e2e_runtimes if x > FLAGS.e2e_deadline])
    if not with_deadlines:
        num_detection_misses = len(
            [x for x in detector_runtimes if x > FLAGS.detection_deadline])
        num_tracker_misses = len(
            [x for x in tracker_runtimes if x > FLAGS.tracker_deadline])
        num_loc_finder_misses = len(
            [x for x in loc_finder_runtimes if x > FLAGS.loc_finder_deadline])
        num_prediction_misses = len(
            [x for x in prediction_runtimes if x > FLAGS.prediction_deadline])
        num_lanning_misses = len(
            [x for x in planning_runtimes if x > FLAGS.planning_deadline])
    deadline_stats = {
        'detection_misses': num_detection_misses,
        'tracker_misses': num_tracker_misses,
        'loc_finder_misses': num_loc_finder_misses,
        'prediction_misses': num_prediction_misses,
        'planning_misses': num_planning_misses,
        'e2e_misses': num_e2e_misses,
        'num_deadlines': entries,
    }
    runtimes_df = pd.DataFrame({
        'town': [town] * entries,
        'route': [route] * entries,
        'runtimes': e2e_runtimes,
        'setup': ['e2e'] * entries,
        'deadline': with_deadlines,
    })
    det_runtimes_df = pd.DataFrame({
        'town': [town] * entries,
        'route': [route] * entries,
        'runtimes': detector_runtimes,
        'setup': ['detection'] * entries,
        'deadline': with_deadlines,
    })
    track_runtimes_df = pd.DataFrame({
        'town': [town] * entries,
        'route': [route] * entries,
        'runtimes': tracker_runtimes,
        'setup': ['tracking'] * entries,
        'deadline': with_deadlines,
    })
    loc_finder_runtimes_df = pd.DataFrame({
        'town': [town] * entries,
        'route': [route] * entries,
        'runtimes': loc_finder_runtimes,
        'setup': ['loc_finder'] * entries,
        'deadline': with_deadlines,
    })
    pred_runtimes_df = pd.DataFrame({
        'town': [town] * entries,
        'route': [route] * entries,
        'runtimes': prediction_runtimes,
        'setup': ['prediction'] * entries,
        'deadline': with_deadlines,
    })
    plan_runtimes_df = pd.DataFrame({
        'town': [town] * entries,
        'route': [route] * entries,
        'runtimes': planning_runtimes,
        'setup': ['planning'] * entries,
        'deadline': with_deadlines,
    })
    return runtimes_df, det_runtimes_df, track_runtimes_df, loc_finder_runtimes_df, pred_runtimes_df, plan_runtimes_df, deadline_stats


def annotate_deadlines(ax, detection):
    if detection:
        ax.axhline(FLAGS.detection_deadline, 0, 0.33, ls='--', color='r')
        ax.axhline(FLAGS.prediction_deadline, 0.33, 0.66, ls='--', color='r')
        ax.axhline(FLAGS.planning_deadline, 0.66, 0.99, ls='--', color='r')
        #ax.axhline(FLAGS.e2e_deadline, 0.75, 1.0, ls='--', color='r')
        ax.text(-0.15,
                FLAGS.detection_deadline + 5,
                'deadline',
                color='r',
                fontsize='small')
        # ax.text(0.95,
        #         FLAGS.prediction_deadline + 5,
        #         'deadline',
        #         color='r',
        #         fontsize='small')
        # ax.text(1.95,
        #         FLAGS.planning_deadline + 5,
        #         'deadline',
        #         color='r',
        #         fontsize='small')
        # ax.text(2.93,
        #         FLAGS.e2e_deadline + 5,
        #         'deadline',
        #         color='r',
        #         fontsize='small')
    else:
        ax.axhline(FLAGS.prediction_deadline, 0, 0.5, ls='--', color='r')
        ax.axhline(FLAGS.planning_deadline, 0.5, 1.0, ls='--', color='r')
        #ax.axhline(FLAGS.e2e_deadline, 0.66, 0.99, ls='--', color='r')
        ax.text(0.05,
                FLAGS.prediction_deadline + 5,
                'deadline',
                color='r',
                fontsize='small')
        ax.text(1.05,
                FLAGS.planning_deadline + 5,
                'deadline',
                color='r',
                fontsize='small')
        # ax.text(2.05,
        #         FLAGS.e2e_deadline + 5,
        #         'deadline',
        #         color='r',
        #         fontsize='small')


def main(argv):
    setup_plot(FLAGS)
    matplotlib.rc('font', family='serif', size=20)
    matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('legend', fontsize=20)
    matplotlib.rc('figure', figsize=(6, 2.75))
    matplotlib.rc('axes', linewidth=1.5)
    matplotlib.rc('lines', linewidth=1.5)

    colors = get_colors(['No-Constraints', 'Deadlines'])

    towns = [int(town) for town in FLAGS.towns]
    deadline_setups = [True, False]
    detector = 4
    runtimes_dfs = []
    with_detection = True
    deadline_stats_w_handlers = {
        'detection_misses': 0,
        'tracker_misses': 0,
        'loc_finder_misses': 0,
        'prediction_misses': 0,
        'planning_misses': 0,
        'e2e_misses': 0,
        'num_deadlines': 0,
    }
    deadline_stats_wo_handlers = {
        'detection_misses': 0,
        'tracker_misses': 0,
        'loc_finder_misses': 0,
        'prediction_misses': 0,
        'planning_misses': 0,
        'e2e_misses': 0,
        'num_deadlines': 0,
    }

    for town in towns:
        for route in range(FLAGS.start_route, FLAGS.end_route + 1):
            for with_deadline in deadline_setups:
                if with_deadline:
                    log_dir_base = FLAGS.base_dir + \
                        'logs_deadlines_town_{}_route_{}_timely_True_edet_{}'.format(
                            town, route, detector)
                else:
                    log_dir_base = FLAGS.base_dir + \
                        'logs_town_{}_route_{}_timely_True_edet_{}'.format(
                            town, route, detector)
                (runtimes_df, det_runtimes_df, track_runtimes_df,
                 loc_finder_runtimes_df, pred_runtimes_df, plan_runtimes_df,
                 deadline_stats) = \
                    read_challenge_results(
                        log_dir_base, town, route, detector, FLAGS.num_reps,
                        with_deadline)
                if with_deadline:
                    for key, value in deadline_stats.items():
                        deadline_stats_w_handlers[key] += value
                else:
                    for key, value in deadline_stats.items():
                        deadline_stats_wo_handlers[key] += value
                if with_detection:
                    runtimes_dfs.append(det_runtimes_df)
                runtimes_dfs.append(pred_runtimes_df)
                runtimes_dfs.append(plan_runtimes_df)
                runtimes_dfs.append(runtimes_df)
    runtime_data = pd.concat(runtimes_dfs)

    fig, axs = plt.subplots(ncols=2,
                            nrows=1,
                            gridspec_kw={
                                'width_ratios': [3.3, 1],
                                'hspace': 0.1,
                                'wspace': 0.05
                            })

    ax = sns.boxplot(x='setup',
                     y='runtimes',
                     hue='deadline',
                     data=runtime_data[runtime_data.setup != 'e2e'],
                     width=0.7,
                     whis=(5, 95),
                     saturation=1,
                     showfliers=True,
                     fliersize=0.7,
                     palette=colors,
                     ax=axs[0])
    ax.set_ylim(0, 201)
    ax.set_yticks([0, 50, 100, 150, 200])
    ax.set_yticklabels(['0', '50', '100', '150', '200'])
    ax.set_ylabel('Response time [ms]')
    ax.set_xlabel("")
    if with_detection:
        ax.set_xticks([0, 1, 2])
        ax.set_xticklabels(['Detection', 'Prediction', 'Planning'])
    else:
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['Prediction', 'Planning'])

    handles, _ = ax.get_legend_handles_labels()
    # ax.legend(handles=handles[0:2],
    #           labels=['W/o deadlines', 'W/ deadlines'],
    #           framealpha=0,
    #           loc="upper left",
    #           ncol=1,
    #           handlelength=0.5,
    #           handletextpad=0.3)
    ax.get_legend().remove()
    ax.legend(handles=handles[0:2],
              labels=['Without $D_{EH}$', 'With $D_{EH}$'],
              framealpha=0,
              loc="upper left",
              ncol=1,
              columnspacing=0.1,
              labelspacing=0.15,
              bbox_to_anchor=(-0.05, 1.08),
              handlelength=0.5,
              handletextpad=0.3)

    annotate_deadlines(ax, with_detection)

    ax = sns.boxplot(x='setup',
                     y='runtimes',
                     hue='deadline',
                     data=runtime_data[runtime_data.setup == 'e2e'],
                     width=0.7,
                     saturation=1,
                     whis=(5, 95),
                     showfliers=True,
                     fliersize=0.7,
                     palette=colors,
                     ax=axs[1])
    ax.axhline(FLAGS.e2e_deadline, 0.0, 1.00, ls='--', color='r')
    # ax.text(-0.12,
    #         FLAGS.e2e_deadline + 5,
    #         'deadline',
    #         color='r',
    #         fontsize='small')

    ax.get_legend().remove()

    max_runtime = runtime_data['runtimes'].max()
    y_max = int(math.ceil(max_runtime / 100) * 100)
    ax.set_ylabel("")
    ax.yaxis.tick_right()
    ax.set_yticks([y for y in range(0, y_max + 1, 100)])
    ax.set_yticklabels([str(y) for y in range(0, y_max + 1, 100)])
    ax.set_ylim(0, y_max + 1)
    ax.set_xticks([0])
    ax.set_xticklabels(['End-to-end'])
    plt.xlabel('')

    plt.savefig('deadline-bbox.{}'.format(FLAGS.file_format),
                bbox_inches='tight')


if __name__ == '__main__':
    app.run(main)
