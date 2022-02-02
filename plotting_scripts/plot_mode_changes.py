from absl import app, flags

import brewer2mpl

import matplotlib
matplotlib.use("agg")
import matplotlib.pyplot as plt

import numpy as np

from pylot_utils import ProfileEvent, ProfileEvents, fix_pylot_profile, \
    read_end_to_end_runtimes

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


def read_detector_runtimes(log_file):
    runtimes = []
    with open(log_file) as lf:
        for line in lf:
            if 'detector runtime' in line:
                cols = line.split(' ')
                runtime = float(cols[6].rstrip())
                runtimes.append(runtime)
    return runtimes


def read_detector_deadlines(log_file):
    deadlines = []
    with open(log_file) as lf:
        for line in lf:
            if 'Detection allocated runtime' in line:
                cols = line.split(' ')
                deadline = int(cols[7].rstrip())
                deadlines.append(deadline)
    return deadlines


def read_planner_runtimes(log_file):
    runtimes = []
    with open(log_file) as lf:
        for line in lf:
            if 'Frenet runtime' in line:
                cols = line.split(' ')
                runtime = float(cols[6].rstrip())
                runtimes.append(runtime)
    return runtimes


def read_planner_deadlines(log_file):
    deadlines = []
    with open(log_file) as lf:
        for line in lf:
            if 'Planner allocated runtime' in line:
                cols = line.split(' ')
                deadline = int(cols[7].rstrip())
                deadlines.append(deadline)
    return deadlines


def main(argv):
    #setup_plot(FLAGS)
    matplotlib.rc('font', family='serif', size=7)
    matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('legend', fontsize=6)
    matplotlib.rc('figure', figsize=(3.33, 1.66))
    #  matplotlib.rc('figure.subplot', left=0.10, top=0.90, bottom=0.12, right=0.95)
    matplotlib.rc('axes', linewidth=0.5)
    matplotlib.rc('lines', linewidth=0.5)

    bmap = brewer2mpl.get_map('Set2', 'qualitative', 7)
    colors = bmap.mpl_colors

    deadline_color = 'r'
    detection_color, planning_color = colors[2], colors[6]

    log_dir = FLAGS.base_dir + \
        'logs_mode_change_1_sec_window_town_{}_route_{}_timely_True_run_{}/'.format(
            1, 2, 1)
    log_file = log_dir + 'challenge.log'
    result_file = log_dir + 'results.json'
    csv_file = log_dir + 'challenge.csv'
    profile_file = log_dir + 'challenge.json'
    detector_runtimes = read_detector_runtimes(log_file)[4500:4800]
    detector_deadlines = read_detector_deadlines(log_file)[4500:4800]
    planner_runtimes = read_planner_runtimes(log_file)[5100:5400]
    planner_deadlines = read_planner_deadlines(log_file)[5100:5400]

    timeline_len = len(detector_runtimes) * 50
    sim_time = [x for x in range(0, timeline_len, 50)]

    fig, axs = plt.subplots(ncols=2, nrows=1, sharey=True, figsize=(3.3, 0.80))

    sns.lineplot(x=sim_time,
                 y=detector_deadlines,
                 ax=axs[0],
                 label='Deadline',
                 color=deadline_color)
    axs[0].fill_between(sim_time,
                        0,
                        detector_runtimes,
                        label='Detection',
                        facecolor=detection_color)
    # sns.scatterplot(x=sim_time, y=detector_runtimes, marker='.')
    handles, _ = axs[0].get_legend_handles_labels()
    axs[0].legend(handles=handles,
                  framealpha=0,
                  loc=(0.00, 0.72),
                  ncol=2,
                  columnspacing=3,
                  labelspacing=0.15,
                  handlelength=0.5,
                  handletextpad=0.3)
    # axs[0].legend(handles=handles[1:],
    #               framealpha=0,
    #               loc=(0.75, 0.76),
    #               ncol=1,
    #               handlelength=0.5,
    #               handletextpad=0.3)

    axs[0].set_xlim(0, 15000)
    axs[0].set_ylim(0, 251)
    axs[0].set_yticks([x for x in range(0, 251, 50)])
    axs[0].set_xticks([x for x in range(0, timeline_len + 1, 5000)])
    axs[0].set_xticklabels(
        [str(int(x / 1000)) for x in range(0, timeline_len + 1, 5000)])

    sns.lineplot(x=sim_time,
                 y=planner_deadlines,
                 ax=axs[1],
                 label='Deadline',
                 color=deadline_color)
    axs[1].fill_between(sim_time,
                        0,
                        planner_runtimes,
                        label='Planning',
                        facecolor=planning_color)
    #ax = sns.scatterplot(x=sim_time, y=planner_runtimes, marker='.')

    handles, _ = axs[1].get_legend_handles_labels()
    axs[1].legend(handles=handles,
                  framealpha=0,
                  loc=(0, 0.72),
                  ncol=2,
                  columnspacing=3,
                  labelspacing=0.15,
                  handlelength=0.5,
                  handletextpad=0.3)
    #loc=(0.8, 1.36), framealpha=0, handlelength=1
    axs[1].set_xlim(0, 15000)
    axs[1].set_ylim(0, 251)
    axs[1].set_yticks([x for x in range(0, 251, 50)])
    axs[1].set_xticks([x for x in range(0, timeline_len + 1, 5000)])
    axs[1].set_xticklabels(
        [str(int(x / 1000)) for x in range(0, timeline_len + 1, 5000)])

    fig.text(0.02,
             0.45,
             'Response time [ms]',
             ha='center',
             rotation='vertical',
             va="center",
             color='k')
    #plt.xlabel('Time [s]')
    fig.text(0.5, -0.20, 'Time[s]', ha="center", color="k")
    plt.savefig('pylot-mode-changes.{}'.format(FLAGS.file_format),
                bbox_inches='tight')

    plt.clf()


if __name__ == '__main__':
    app.run(main)
