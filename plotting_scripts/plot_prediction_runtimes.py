import pickle

import brewer2mpl

import matplotlib
matplotlib.use("agg")
import matplotlib.pyplot as plt

import numpy as np

import pandas as pd

import seaborn as sns

from absl import app, flags

from utils import *

FLAGS = flags.FLAGS
flags.DEFINE_string('base_dir', '', 'Path to the base dir where the logs are')
flags.DEFINE_bool('small_paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('stretched', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('slide_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('poster_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_string('file_format', 'png', 'File type of the output plot.')
flags.DEFINE_string('file_name', 'prediction-runtime-horizon',
                    'Name of the file to output to.')
flags.DEFINE_list('horizons', '10,20,30,40,50',
                  'Comma separated list of horizons.')
flags.DEFINE_integer('hz', '10', 'Frequency of predictions.')


def main(argv):
    bmap = brewer2mpl.get_map('Set2', 'qualitative', 7)
    colors = bmap.mpl_colors[3:]
    hatches = ["////", "****"]

    if FLAGS.paper_mode:
        plt.figure(figsize=(3.33, 2.22))
        set_paper_rcs()
    elif FLAGS.small_paper_mode:
        plt.figure(figsize=(2.4, 1.66))
        set_paper_rcs()
    elif FLAGS.stretched:
        plt.figure(figsize=(3, 1.4))
        set_paper_rcs()
    elif FLAGS.slide_mode:
        plt.figure(figsize=(8, 6))
        set_slide_rcs()
    elif FLAGS.poster_mode:
        plt.figure(figsize=(12, 9))
        set_poster_rcs()
    else:
        plt.figure()
        set_rcs()

    ax = plt.gca()

    models = ["mfp", "r2p2"]
    legend_elements = []
    dfs = []
    for i, model in enumerate(models):
        for h in FLAGS.horizons:
            file_name = '{}/{}_timely_horizon_{}.pkl'.format(
                FLAGS.base_dir, model, h)
            f = open(file_name, 'rb')
            num_secs = int(int(h) * 1.0 / FLAGS.hz)
            data = pickle.load(f)
            df = pd.DataFrame({
                'model': [model] * len(data),
                'horizon': [num_secs] * len(data),
                'runtime': data
            })
            dfs.append(df)
        if model == 'mfp':
            label = 'MFP'
        elif model == 'r2p2':
            label = 'R2P2-MA'
        else:
            label = model
        legend_elements.append(
            Patch(facecolor=colors[i],
                  alpha=0.6,
                  hatch=hatches[i],
                  label=label))

    data = pd.concat(dfs)
    ax = sns.boxplot(x='horizon',
                     y='runtime',
                     hue='model',
                     data=data,
                     palette=colors,
                     width=0.7,
                     saturation=1,
                     whis=(5, 95),
                     showfliers=False)
    for i, box in enumerate(ax.artists):
        box.set_hatch(hatches[i % len(models)])

    adjust_box_widths(plt.gcf(), 0.8)

    plt.legend(handles=legend_elements,
               framealpha=0,
               handlelength=1.5,
               handletextpad=0.1)
    plt.xlabel('Prediction horizon [s]')
    plt.ylabel('Runtime [ms]')
    plt.savefig("{}.{}".format(FLAGS.file_name, FLAGS.file_format),
                format=FLAGS.file_format,
                bbox_inches='tight')


if __name__ == '__main__':
    app.run(main)
