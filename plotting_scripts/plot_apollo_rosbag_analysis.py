import csv
import sys

from absl import app, flags

import matplotlib
matplotlib.use("agg")
import matplotlib.pyplot as plt

import numpy as np

from utils import plot_box_and_whiskers, plot_cdf, set_paper_rcs, generate_ticks, print_stats

FLAGS = flags.FLAGS
flags.DEFINE_string('files', '', ', separated list of input files.')
flags.DEFINE_enum('graph_type', 'cdf', ['cdf', 'box_and_whiskers', 'timeline'],
                  'Type of graph to plot')
flags.DEFINE_string('labels', '',
                    ', separated list of labels to use for log files.')
flags.DEFINE_string('file_name', '', ', file to dump the results to.')
flags.DEFINE_string('xlabel', 'Runtime [ms]',
                    ', the xlabel to give the graph.')
flags.DEFINE_bool('paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('small_paper_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('slide_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('poster_mode', False, 'Adjusts the size of the plots.')
flags.DEFINE_bool('manual_mode', True, 'Adjusts the size of the plots.')
flags.DEFINE_string('file_format', 'pdf', 'File type of the output plot.')
flags.DEFINE_float('figsize_x', None, 'Figure size x value.')
flags.DEFINE_float('figsize_y', None, 'Figure size y value.')
flags.DEFINE_integer('max_y_val', None, 'Max y value.')
flags.DEFINE_integer('x_tick_increment', 10, 'x tick increment value.')
flags.DEFINE_string('markers', 'o,+,^,1,2', ', separated list of markers.')
flags.DEFINE_string('colors', 'r,b,g,c,m,y,k', ', separated list of colors.')


def get_data(csv_file):
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        lidar_vals, perception_vals, prediction_vals = [], [], []
        for row in reader:
            lidar, perception, prediction = row
            lidar = int(lidar)
            perception = int(perception) if perception != "None" else None
            prediction = int(prediction) if prediction != "None" else None
            lidar_vals.append(lidar)
            perception_vals.append(perception)
            prediction_vals.append(prediction)

        min_lidar_val = lidar_vals[0]
        lidar_x, lidar_y = [], []
        perception_x, perception_y = [], []
        prediction_x, prediction_y = [], []
        missed_lidar_x, missed_lidar_y = [], []
        for lidar, perception, prediction in zip(lidar_vals, perception_vals,
                                                 prediction_vals):
            x_val = (lidar - min_lidar_val) / 1e9
            if x_val < 20 or x_val > 60:
                continue
            x_val -= 20

            # Add LIDAR values.
            if perception is None:
                print("The last 10 perception runtimes were: {}".format(
                    [p_time for p_time in perception_y]))
                missed_lidar_x.append(x_val)
                missed_lidar_y.append(10)
            else:
                lidar_x.append(x_val)
                lidar_y.append(0)

            # Add Perception values.
            if perception is not None:
                perception_x.append(x_val)
                perception_y.append((perception - lidar) / 1e6)

            # Add prediction values.
            if prediction is not None:
                prediction_x.append(x_val)
                prediction_y.append((prediction - lidar) / 1e6)

        print("The perception stats are: ")
        print_stats(perception_y)
        print("The prediction stats are: ")
        print_stats(prediction_y)
        return [perception_x, prediction_x,
                missed_lidar_x], [perception_y, prediction_y, missed_lidar_y]


def main(argv):
    csv_files = FLAGS.files.split(',')
    if len(csv_files) != 1:
        raise ValueError("Multiple files were provided, this script works "
                         "with a single file.")
    labels = FLAGS.labels.split(',')
    markers = FLAGS.markers.split(',')
    colors = FLAGS.colors.split(',')

    if FLAGS.graph_type != "cdf":
        raise ValueError("Only CDF plots supported for now.")

    x_vals, y_vals = get_data(csv_files[0])
    plot_timeline(
        FLAGS,
        FLAGS.file_name,
        x_vals,
        y_vals,
        "Timeline [s]",
        "Response Time [ms]",
        ["Perception", "Prediction", "Dropped Sensor"],
        x_ticks_increment=5,
        y_ticks_increment=200,
        markers=["+", "^", "x"],
        markersize=[3, 3, 6],
        colors=["g", "c", "r"],
        #automatic_legend=True,
        #legend_fn=legend,
        cut_y_max=1000,
        cut_y_min=0,
        cut_x_max=41,
    )


def plot_timeline(flags,
                  plot_file_name,
                  all_x_vals,
                  all_y_vals,
                  x_label,
                  y_label,
                  labels,
                  x_ticks_increment=None,
                  y_ticks_increment=None,
                  automatic_legend=False,
                  legend_fn=None,
                  show_legend=False,
                  legend_outside_box=False,
                  markers=['o', '+', '^', '1', '2'],
                  markersize=[2, 2, 2, 2, 2],
                  colors=['r', 'b', 'g', 'c', 'm', 'y', 'k'],
                  cut_x_min=None,
                  cut_x_max=None,
                  cut_y_min=None,
                  cut_y_max=None,
                  line_width=0.0,
                  scale_down_x_ticks=1):
    plt.figure(figsize=(3.33, 1.2))
    set_paper_rcs()

    index = 0
    min_x_val = sys.maxsize
    max_x_val = -sys.maxsize
    min_y_val = sys.maxsize
    max_y_val = -sys.maxsize

    plots = []
    for index in range(0, len(all_x_vals)):
        min_x_val = min(min_x_val, np.min(all_x_vals[index]))
        max_x_val = max(max_x_val, np.max(all_x_vals[index]))
        min_y_val = min(min_y_val, np.min(all_y_vals[index]))
        max_y_val = max(max_y_val, np.max(all_y_vals[index]))
        plots.append(
            plt.plot(all_x_vals[index], [y for y in all_y_vals[index]],
                     label=labels[index],
                     color=colors[index],
                     marker=markers[index],
                     mfc='none',
                     mew=1.0,
                     mec=colors[index],
                     lw=line_width,
                     markersize=markersize[index]))

    print("Plotting {} stats".format(plot_file_name))
    print("Max x value: {}".format(max_x_val))
    print("Max y value: {}".format(max_y_val))

    if cut_x_min is not None:
        print('WARNING: Fixing min_x_val from {} to {}'.format(
            min_x_val, cut_x_min))
        min_x_val = cut_x_min
    if cut_x_max is not None:
        print('WARNING: Fixing max_x_val from {} to {}'.format(
            max_x_val, cut_x_max))
        max_x_val = cut_x_max
    if cut_y_min is not None:
        print('WARNING: Fixing min_y_val from {} to {}'.format(
            min_y_val, cut_y_min))
        min_y_val = cut_y_min
    if cut_y_max is not None:
        print('WARNING: Fixing max_y_val from {} to {}'.format(
            max_y_val, cut_y_max))
        max_y_val = cut_y_max

    plt.xlim(min_x_val, max_x_val)
    if x_ticks_increment is not None:
        ticks, ticks_str = generate_ticks(min_x_val, max_x_val,
                                          x_ticks_increment,
                                          scale_down_x_ticks)
        plt.xticks(ticks, ticks_str)

    plt.ylim(min_y_val, max_y_val)
    if y_ticks_increment is not None:
        ticks, ticks_str = generate_ticks(min_y_val, max_y_val,
                                          y_ticks_increment)
        plt.yticks(ticks, ticks_str)

    plt.xlabel(x_label)
    plt.ylabel(y_label)

    legend1 = plt.legend(plots[-1], ["Dropped Sensor Message"],
                         loc=(0, 0.8),
                         handlelength=1.5,
                         handletextpad=0.1,
                         frameon=False)
    legend2 = plt.legend(plots[0], ["Perception"],
                         loc=(0.7, 0.8),
                         handlelength=1.5,
                         handletextpad=0.1,
                         frameon=False)
    plt.legend(plots[1], ["Prediction"],
               loc=(0.7, 0.68),
               handlelength=1.5,
               handletextpad=0.1,
               frameon=False)
    #plt.legend(plots[:2], ["Perception", "Prediction"], loc=(0.85, 0.85), handlelength=1.5, handletextpad=0.1, frameon=False)
    #legends.append(plt.legend(handlelength=1.5, handletextpad=0.1))
    plt.gca().add_artist(legend1)
    plt.gca().add_artist(legend2)

    plt.savefig("{}.{}".format(plot_file_name, flags.file_format),
                format=flags.file_format,
                bbox_inches="tight")


if __name__ == "__main__":
    app.run(main)
