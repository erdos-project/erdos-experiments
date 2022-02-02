import csv
import math

from absl import app, flags

import matplotlib
matplotlib.use("agg")
import matplotlib.pyplot as plt

import numpy as np

from pylot_utils import ProfileEvent, ProfileEvents, fix_pylot_profile, \
    read_challenge_results

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
flags.DEFINE_bool('plot_scores', False, 'Plot scores.')
flags.DEFINE_bool('plot_route_runtimes', False,
                  'Plot runtimes for each route.')
flags.DEFINE_bool('plot_runtimes', False,
                  'Plot runtimes aggregated for all routes.')
flags.DEFINE_bool('verbose', False, 'Enables verbose logging.')
flags.DEFINE_integer('start_route', 1, 'Id of the first completed route.')
flags.DEFINE_integer('end_route', 9, 'Id of the last completed route.')
flags.DEFINE_integer('num_reps', 5, 'Number of experiment repetitions.')
flags.DEFINE_list('towns', ['1'], 'List of towns.')


def plot_runtime(runtime_data, town, route, detector):
    data = runtime_data[(runtime_data.town == town)
                        & (runtime_data.route == route)
                        & (runtime_data.detector == detector)]

    sns.ecdfplot(data=data, x='e2e_runtime', hue='policy')
    plt.title('End-to-end runtime policy vs. no-policy')
    plt.xlabel('End-to-end runtime [$ms$]')
    plt.ylabel('CDF of end-to-end runtime')
    plt.savefig('policy-e2e-runtime-town-{}-route-{}.{}'.format(
        town, route, FLAGS.file_format),
                bbox_inches='tight')
    plt.clf()


def main(argv):
    plt.figure(figsize=(2.2, 1.2))
    set_paper_rcs()
    # setup_plot(FLAGS)

    colors = get_colors(['No-Constraints', 'Policy'])

    towns = [int(town) for town in FLAGS.towns]
    policy_setups = [False, True]
    detector = 4
    total_driven = 0
    runtimes_dfs = []
    scores_dfs = []
    for town in towns:
        for route in range(FLAGS.start_route, FLAGS.end_route + 1):
            for policy in policy_setups:
                if policy:
                    log_dir_base = FLAGS.base_dir + \
                        'logs_town_{}_route_{}_timely_True_edet_{}'.format(
                            town, route, detector)
                else:
                    log_dir_base = FLAGS.base_dir + \
                        'logs_nopolicy_town_{}_route_{}_timely_True_edet_{}'.format(
                            town, route, detector)
                runtimes_df, score_df, route_len = read_challenge_results(
                    log_dir_base, town, route, detector, FLAGS.num_reps,
                    'policy', policy)
                runtimes_dfs.append(runtimes_df)
                scores_dfs.append(score_df)
                total_driven += route_len

    runtime_data = pd.concat(runtimes_dfs)
    if FLAGS.plot_route_runtimes:
        for town in towns:
            for route in range(FLAGS.start_route, FLAGS.end_route + 1):
                plot_runtime(runtime_data, town, route, detector)

    if FLAGS.plot_runtimes:
        sns.ecdfplot(data=runtime_data,
                     x='e2e_runtime',
                     hue='policy',
                     palette=colors)
        # if not (FLAGS.paper_mode or FLAGS.small_paper_mode):
        #     plt.title('Response time policy vs. no-policy')
        plt.xlabel('Response time [$ms$]')
        plt.ylabel('CDF of response time')
        plt.savefig('policy-e2e-runtime-all-routes.{}'.format(
            FLAGS.file_format),
                    bbox_inches='tight')
        plt.clf()

        sns.boxplot(x='policy',
                    y='e2e_runtime',
                    data=runtime_data,
                    whis=(5, 95),
                    showfliers=False,
                    saturation=1,
                    palette=colors)
        # if not (FLAGS.paper_mode or FLAGS.small_paper_mode):
        #     plt.title('End-to-end runtime no-policy vs. policy')
        p95_runtime = runtime_data['e2e_runtime'].quantile(0.95)
        y_max = int(math.ceil(p95_runtime / 50) * 50)
        plt.ylim(0, y_max + 1)
        plt.yticks([y for y in range(0, y_max + 1, 50)],
                   [str(y) for y in range(0, y_max + 1, 50)])
        plt.xlabel('')
        plt.xticks([0, 1], ['W/o no-op policy', 'No-op policy'])
        plt.ylabel('Response time [$ms$]')
        plt.savefig('policy-e2e-runtime-all-routes-bbox.{}'.format(
            FLAGS.file_format),
                    bbox_inches='tight')
        plt.clf()

        if FLAGS.verbose:
            print('Printing stats with policy')
            data = runtime_data[runtime_data.policy == True]
            print('50th: {}'.format(data['e2e_runtime'].quantile(0.5)))
            print('75th: {}'.format(data['e2e_runtime'].quantile(0.75)))
            print('90th: {}'.format(data['e2e_runtime'].quantile(0.90)))
            print('99th: {}'.format(data['e2e_runtime'].quantile(0.99)))
            print('99.9th: {}'.format(data['e2e_runtime'].quantile(0.999)))
            print('Printing stats without policy')
            data = runtime_data[runtime_data.policy == False]
            print('50th: {}'.format(data['e2e_runtime'].quantile(0.5)))
            print('75th: {}'.format(data['e2e_runtime'].quantile(0.75)))
            print('90th: {}'.format(data['e2e_runtime'].quantile(0.90)))
            print('90th: {}'.format(data['e2e_runtime'].quantile(0.99)))
            print('99.9th: {}'.format(data['e2e_runtime'].quantile(0.999)))

    if FLAGS.plot_scores:
        data = pd.concat(scores_dfs)
        ax = sns.boxplot(x='route',
                         y='score',
                         hue='policy',
                         data=data,
                         saturation=1,
                         width=0.7,
                         whis=(0, 100),
                         showfliers=False)
        plt.title('Route score policy vs. no-policy')
        plt.xlabel('Route')
        plt.ylabel('Challenge route score')
        plt.ylim(0, 110)
        plt.savefig('policy-challenge-scores.{}'.format(FLAGS.file_format),
                    bbox_inches='tight')
        plt.clf()

        ax = sns.boxplot(x='route',
                         y='collisions_ped',
                         hue='policy',
                         data=data,
                         saturation=1,
                         width=0.7,
                         whis=(0, 100),
                         showfliers=False)
        plt.title('Policy vs. no-policy')
        plt.xlabel('Route')
        plt.ylabel('Challenge pedestrian collisions/km')
        plt.savefig('policy-challenge-collisions-pedestrians.{}'.format(
            FLAGS.file_format),
                    bbox_inches='tight')
        plt.clf()

        ax = sns.boxplot(x='route',
                         y='collisions_veh',
                         hue='policy',
                         data=data,
                         saturation=1,
                         width=0.7,
                         whis=(0, 100),
                         showfliers=False)
        plt.title('Policy vs. no-policy')
        plt.xlabel('Route')
        plt.ylabel('Challenge vehicle collisions/km')
        plt.savefig('policy-challenge-collisions-vehicles.{}'.format(
            FLAGS.file_format),
                    bbox_inches='tight')
        plt.clf()

        ax = sns.boxplot(x='route',
                         y='collisions_lay',
                         hue='policy',
                         data=data,
                         saturation=1,
                         width=0.7,
                         whis=(0, 100),
                         showfliers=False)
        plt.title('Policy vs. no-policy')
        plt.xlabel('Route')
        plt.ylabel('Challenge layout collisions/km')
        plt.savefig('policy-challenge-collisions-layout.{}'.format(
            FLAGS.file_format),
                    bbox_inches='tight')
        plt.clf()

    print('======= Total km driven {} ======'.format(total_driven))


if __name__ == '__main__':
    app.run(main)
