import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import brewer2mpl

from utils import *

from absl import app
from absl import flags
from matplotlib.patches import PathPatch, Patch

OP_NAME_TO_TRACKER_NAME = {
    "tracker_sort": "SORT",
    "tracker_deep_sort": "DeepSORT",
    "tracker_da_siam_rpn": "DaSiamRPN",
}

FLAGS = flags.FLAGS
flags.DEFINE_string("log_file", "",
                    "Path to csv log file containing tracker runtimes.")
flags.DEFINE_string("mode", "paper", "Graph mode.")
flags.DEFINE_string('file_format', 'png', 'File type of the output plot.')
flags.DEFINE_string('file_name', '', 'Name of the file to output to.')


def plot_runtimes_vs_num_targets(tracker_metrics_df, tracker_name):
    runtimes_data = tracker_metrics_df[
        (tracker_metrics_df["variable"] == "num_targets")
        & (tracker_metrics_df["value"] < 13) &
        (tracker_metrics_df["value"] > 0) &
        (tracker_metrics_df["value"] % 3 == 1)]
    runtimes_data["value"] = runtimes_data["value"].astype(int)
    return runtimes_data


def main(_):
    bmap = brewer2mpl.get_map('Set2', 'qualitative', 7)
    colors = bmap.mpl_colors
    hatches = ["////", "****", "...."]

    if FLAGS.mode == 'paper':
        plt.figure(figsize=(3.33, 2.22))
        set_paper_rcs()
    elif FLAGS.mode == 'half_column':
        plt.figure(figsize=(1.33, 1))
        set_paper_rcs()
    elif FLAGS.mode == 'small_paper':
        plt.figure(figsize=(2.4, 1.66))
        set_paper_rcs()
    elif FLAGS.mode == 'slide':
        plt.figure(figsize=(8, 6))
        set_slide_rcs()
    elif FLAGS.mode == 'poster':
        plt.figure(figsize=(12, 9))
        set_poster_rcs()
    else:
        plt.figure()
        set_rcs()

    ax = plt.gca()

    all_metrics = pd.read_csv(FLAGS.log_file)
    runtimes = []
    legend_elements = []
    for i, tracker_name in enumerate(OP_NAME_TO_TRACKER_NAME.values()):
        tracker_metrics_df = all_metrics[all_metrics["op_name"] ==
                                         tracker_name]
        runtimes_data = tracker_metrics_df[
            (tracker_metrics_df["variable"] == "num_targets")
            & (tracker_metrics_df["value"] < 13) &
            (tracker_metrics_df["value"] > 0) &
            (tracker_metrics_df["value"] % 3 == 1)]
        runtimes_data["value"] = runtimes_data["value"].astype(int)
        runtimes_data["tracker"] = tracker_name

        for num_targets in range(1, 13, 3):
            print("Tracker {} stats for {} num targets".format(
                tracker_name, num_targets))
            res = tracker_metrics_df[
                (tracker_metrics_df["variable"] == "num_targets")
                & (tracker_metrics_df["value"] == num_targets)]
            print_stats(res["duration"].values)
        runtimes.append(runtimes_data)
        legend_elements.append(
            Patch(facecolor=colors[i],
                  alpha=0.6,
                  hatch=hatches[i],
                  label=tracker_name))

    merged_runtimes = pd.concat(runtimes)

    ax = sns.boxplot(x="value",
                     y="duration",
                     data=merged_runtimes,
                     hue="tracker",
                     palette=colors,
                     width=0.7,
                     saturation=1,
                     whis=(5, 95),
                     showfliers=False)

    for i, box in enumerate(ax.artists):
        box.set_hatch(hatches[i % len(runtimes)])

    adjust_box_widths(plt.gcf(), 0.8)

    plt.legend(handles=legend_elements,
               framealpha=0,
               handlelength=1.5,
               handletextpad=0.1)
    plt.ylabel("Runtime [ms]")
    plt.xlabel("Number of agents tracked")
    plt.savefig("{}.{}".format(FLAGS.file_name, FLAGS.file_format),
                format=FLAGS.file_format,
                bbox_inches='tight')


if __name__ == '__main__':
    app.run(main)
