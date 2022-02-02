import csv
import json

import matplotlib
matplotlib.use("agg")
import matplotlib.pyplot as plt

import numpy as np

from absl import app, flags

from pylot_utils import ProfileEvent, ProfileEvents, fix_pylot_profile, \
    read_challenge_results

from utils import setup_plot

import pandas as pd

import seaborn as sns

FLAGS = flags.FLAGS
flags.DEFINE_string('base_dir', '', 'Path to the base dir where the logs are')
flags.DEFINE_bool('small_paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('stretched', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('slide_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('poster_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_string('file_name', 'pylot', 'File to write plot to.')
flags.DEFINE_string('file_format', 'png', 'File type of the output plot.')
flags.DEFINE_integer('ignore_first_sim_time_ms', 0,
                     'Ignore data from the first simulation time ms.')
flags.DEFINE_bool('filter_carla_cola', False,
                  'Filter out collisions with the carla cola truck')
flags.DEFINE_integer('start_route', 1, 'Id of the first completed route.')
flags.DEFINE_integer('end_route', 9, 'Id of the last completed route.')
flags.DEFINE_integer('num_reps', 5, 'Number of experiment repetitions.')
flags.DEFINE_list('towns', ['1'], 'List of towns.')


def print_collision_stats(data, km_driven):
    print('Filtering out collisions with CARLA cola truck: {}'.format(
        FLAGS.filter_carla_cola))
    num_vec_col = data[data.timely == 'True']['num_vec_collisions'].sum()
    num_ped_col = data[data.timely == 'True']['num_ped_collisions'].sum()
    print('Collisions: vehicles {}, pedestrians {}'.format(
        num_vec_col, num_ped_col))


def main(argv):
    setup_plot(FLAGS)
    towns = [int(town) for town in FLAGS.towns]
    km_driven = 0
    score_dfs = []
    runtime_dfs = []
    for town in towns:
        for route in range(FLAGS.start_route, FLAGS.end_route + 1):
            log_dir_base = FLAGS.base_dir + \
                'logs_town_{}_route_{}_timely_True_edet_4'.format(
                            town, route)
            runtimes_df, score_df, config_km_driven = read_challenge_results(
                log_dir_base, town, route, 4, FLAGS.num_reps, 'deadlines',
                'False')
            score_dfs.append(score_df)
            runtime_dfs.append(runtimes_df)
            km_driven += config_km_driven

            log_dir_base = FLAGS.base_dir + \
                'logs_frequency_town_{}_route_{}_timely_True_edet_4'.format(
                            town, route)
            runtimes_df, score_df, config_km_driven = read_challenge_results(
                log_dir_base, town, route, 4, FLAGS.num_reps, 'frequency',
                'True')
            score_dfs.append(score_df)
            runtime_dfs.append(runtimes_df)
            km_driven += config_km_driven

            log_dir_base = FLAGS.base_dir + \
                'logs_deadlines_town_{}_route_{}_timely_True_edet_4'.format(
                            town, route)
            runtimes_df, score_df, config_km_driven = read_challenge_results(
                log_dir_base, town, route, 4, FLAGS.num_reps, 'deadlines',
                'True')
            score_dfs.append(score_df)
            runtime_dfs.append(runtimes_df)
            km_driven += config_km_driven

            log_dir_base = FLAGS.base_dir + \
                'logs_policy_town_{}_route_{}_timely_True'.format(
                            town, route)
            runtimes_df, score_df, config_km_driven = read_challenge_results(
                log_dir_base, town, route, 4, FLAGS.num_reps, 'policy', 'True')
            score_dfs.append(score_df)
            runtime_dfs.append(runtimes_df)
            km_driven += config_km_driven

    runtime_data = pd.concat(runtime_dfs)
    data = pd.concat(score_dfs)

    num_vec_col = data[data.deadlines == 'True']['num_vec_collisions'].sum()
    num_ped_col = data[data.deadlines == 'True']['num_ped_collisions'].sum()
    print('Collisions with deadlines: vehicles {}, pedestrians {}'.format(
        num_vec_col, num_ped_col))

    num_vec_col = data[data.deadlines == 'False']['num_vec_collisions'].sum()
    num_ped_col = data[data.deadlines == 'False']['num_ped_collisions'].sum()
    print('Collisions without deadlines: vehicles {}, pedestrians {}'.format(
        num_vec_col, num_ped_col))

    num_vec_col = data[data.frequency == 'True']['num_vec_collisions'].sum()
    num_ped_col = data[data.frequency == 'True']['num_ped_collisions'].sum()
    print('Collisions with periodic: vehicles {}, pedestrians {}'.format(
        num_vec_col, num_ped_col))

    num_vec_col = data[data.policy == 'True']['num_vec_collisions'].sum()
    num_ped_col = data[data.policy == 'True']['num_ped_collisions'].sum()
    print('Collisions with policy: vehicles {}, pedestrians {}'.format(
        num_vec_col, num_ped_col))


if __name__ == '__main__':
    app.run(main)
