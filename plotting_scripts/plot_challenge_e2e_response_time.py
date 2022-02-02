import csv
import math

from absl import app, flags

import matplotlib
matplotlib.use("agg")
import matplotlib.pyplot as plt

import numpy as np

from pylot_utils import ProfileEvent, ProfileEvents, fix_pylot_profile

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
flags.DEFINE_bool('verbose', False, 'Enables verbose logging.')
flags.DEFINE_integer('start_route', 1, 'Id of the first completed route.')
flags.DEFINE_integer('end_route', 9, 'Id of the last completed route.')
flags.DEFINE_integer('num_reps', 5, 'Number of experiment repetitions.')
flags.DEFINE_list('towns', ['1'], 'List of towns.')
flags.DEFINE_bool('plot_histogram', False,
                  'Plot a single histogram of runtimes.')
flags.DEFINE_bool('plot_multi_histograms', False,
                  'Plot configs in different subplots.')
flags.DEFINE_bool('plot_violin', False,
                  'Plot config runtimes as violion plots.')


def read_challenge_runtimes(csv_file_path):
    csv_file = open(csv_file_path)
    csv_reader = csv.reader(csv_file)
    sensor_send_runtime = {}
    print("WARNING: End-to-end runtime includes sensor send time")
    sim_times = []
    e2e_runtimes = []
    for row in csv_reader:
        sim_time = int(row[1])
        event_name = row[2]
        if event_name == 'e2e_runtime':
            e2e_runtime = float(row[3])
            if e2e_runtime > 600:
                # Ignorning outlier entries because the policy experiments
                # didn't have a deadline exception handler for detection.
                # Therefore each run has 1-2 outlier runtimes when a new
                # detection model is loaded.
                print("WARNING: Ignoring entry {}".format(row))
                continue
            e2e_runtimes.append(e2e_runtime - sensor_send_runtime[sim_time])
            #e2e_runtimes.append(e2e_runtime)
            sim_times.append(sim_time)
        elif event_name == 'sensor_send_runtime':
            sensor_send_runtime[sim_time] = float(row[3])
    return sim_times, e2e_runtimes


def read_data(log_dir_base, town, route, detector, num_reps, segmentation_name,
              segmentation_value):
    e2e_runtimes = []
    for run in range(1, num_reps + 1):
        log_dir = '{}_run_{}/'.format(log_dir_base, run)
        csv_file = log_dir + 'challenge.csv'
        # Get the end-to-end runtimes.
        (sim_times, run_e2e) = read_challenge_runtimes(csv_file)
        e2e_runtimes = e2e_runtimes + run_e2e
    entries = len(e2e_runtimes)
    runtimes_df = pd.DataFrame({
        'town': [town] * entries,
        'route': [route] * entries,
        'detector': [detector] * entries,
        segmentation_name: [segmentation_value] * entries,
        'e2eruntime': e2e_runtimes,
    })
    return runtimes_df


def main(argv):
    matplotlib.rc('font', family='serif', size=7)
    matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('legend', fontsize=6)
    matplotlib.rc('figure', figsize=(3.33, 2.22))
    #  matplotlib.rc('figure.subplot', left=0.10, top=0.90, bottom=0.12, right=0.95)
    matplotlib.rc('axes', linewidth=0.5)
    matplotlib.rc('lines', linewidth=0.5)
    plt.figure(figsize=(3.3, 0.85))

    colors = get_colors(['No-Constraints', 'Deadlines', 'Policy'])

    towns = [int(town) for town in FLAGS.towns]
    detector = 4
    runtimes_dfs = []
    for town in towns:
        for route in range(FLAGS.start_route, FLAGS.end_route + 1):
            # log_dir_base = FLAGS.base_dir + \
            #     'logs_town_{}_route_{}_timely_True_edet_{}'.format(
            #         town, route, detector)
            # runtimes_df = read_data(log_dir_base, town, route, detector,
            #                         FLAGS.num_reps, 'configuration',
            #                         'no-deadlines')
            # runtimes_dfs.append(runtimes_df)

            log_dir_base = FLAGS.base_dir + \
                'logs_deadlines_town_{}_route_{}_timely_True_edet_{}'.format(
                    town, route, detector)
            runtimes_df = read_data(log_dir_base, town, route, detector,
                                    FLAGS.num_reps, 'configuration',
                                    'deadlines')
            runtimes_dfs.append(runtimes_df)

            # log_dir_base = FLAGS.base_dir + \
            #     'logs_nopolicy_town_{}_route_{}_timely_True_edet_{}'.format(
            #         town, route, detector)
            # runtimes_df = read_data(
            #     log_dir_base, town, route, detector, FLAGS.num_reps,
            #     'configuration', 'no-policy')
            # runtimes_dfs.append(runtimes_df)

            log_dir_base = FLAGS.base_dir + \
                'logs_policy_town_{}_route_{}_timely_True'.format(
                    town, route)
            runtimes_df = read_data(log_dir_base, town, route, detector,
                                    FLAGS.num_reps, 'configuration', 'policy')
            runtimes_dfs.append(runtimes_df)

    runtime_data = pd.concat(runtimes_dfs)

    if FLAGS.plot_histogram:
        ax = sns.histplot(data=runtime_data,
                          x='e2eruntime',
                          hue='configuration',
                          hue_order=['policy', 'deadlines'],
                          palette=[colors[2], colors[1]],
                          binwidth=1,
                          element='step')
        #stat='density')
        ax.legend(  #handles=handles[0:],
            labels=['Static deadlines', 'Dynamic deadlines'],
            framealpha=0,
            loc=(0.01, 0.8),
            ncol=2,
            columnspacing=14,
            handlelength=0.5,
            handletextpad=0.3)
        #            bbox_to_anchor=(0.1, 1.25))
        plt.xlim(0)
        plt.yticks([x for x in range(0, 10001, 2000)],
                   [str(x) for x in range(0, 10001, 2000)])
        plt.xlabel('Response time [ms]')
        # ax.axvline(209, 0.0, 1.0, ls='--', color='r')
        # ax.axvline(269, 0.0, 1.0, ls='--', color='r')
        # ax.axvline(433, 0.0, 1.0, ls='--', color='r')
        plt.savefig('configurations-e2e-runtime-hist.{}'.format(
            FLAGS.file_format),
                    bbox_inches='tight')

        plt.clf()

    if FLAGS.plot_multi_histograms:
        fix, axs = plt.subplots(ncols=1, nrows=2, sharex=True)
        sns.histplot(
            data=runtime_data[runtime_data.configuration == 'deadlines'],
            x='e2eruntime',
            ax=axs[0],
            label='W/ deadlines',
            color=colors[1])
        axs[0].set_xlim(0, 450)
        handles, _ = axs[0].get_legend_handles_labels()
        axs[0].legend(handles=handles[0:1],
                      framealpha=0,
                      loc="upper right",
                      ncol=1,
                      handlelength=0.5,
                      handletextpad=0.3)

        sns.histplot(data=runtime_data[runtime_data.configuration == 'policy'],
                     x='e2eruntime',
                     ax=axs[1],
                     label='W/ deadlines + policy',
                     color=colors[2])
        handles, _ = axs[1].get_legend_handles_labels()
        axs[1].legend(handles=handles[0:1],
                      framealpha=0,
                      loc="upper left",
                      ncol=1,
                      handlelength=0.5,
                      handletextpad=0.3)

        # plt.yticks([0, 100, 200, 300, 400], ['0', '100', '200', '300', '400'])
        # #    axs[1].set_ylim(0, 400)
        # axs[1].set_xlim(0, 450)
        plt.xlabel('Response time [ms]')
        plt.savefig('configurations-e2e-runtime-multi-hist.{}'.format(
            FLAGS.file_format),
                    bbox_inches='tight')
        plt.clf()

    if FLAGS.plot_violin:
        sns.boxplot(
            x='configuration',
            y='e2e_runtime',
            data=runtime_data,
            saturation=1,
            #whis=(5, 95),
            #showfliers=True,
            palette=colors)
        max_runtime = runtime_data['e2e_runtime'].max()
        y_max = int(math.ceil(max_runtime / 100) * 100)
        plt.yticks([y for y in range(0, y_max + 1, 100)],
                   [str(y) for y in range(0, y_max + 1, 100)])
        plt.ylim(0, y_max + 1)
        plt.xlabel('')
        plt.xticks([0, 1, 2],
                   ['W/o deadlines', 'Static deadlines', 'Dyn. deadlines'])
        plt.ylabel('Response time [$ms$]')
        plt.savefig('configurations-e2e-runtime-violin.{}'.format(
            FLAGS.file_format),
                    bbox_inches='tight')
        plt.clf()


if __name__ == '__main__':
    app.run(main)
