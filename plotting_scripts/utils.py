import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import PathPatch, Patch

import numpy as np

import box_and_whisker


# plot saving utility function
def writeout(filename_base, tight=True):
    for fmt in ['pdf']:
        if tight:
            plt.savefig('%s.%s' % (filename_base, fmt),
                        format=fmt,
                        bbox_inches='tight',
                        pad_inches=0.01)
        else:
            plt.savefig('%s.%s' % (filename_base, fmt), format=fmt)


def set_leg_fontsize(size):
    matplotlib.rc('legend', fontsize=size)


def set_paper_rcs():
    matplotlib.rc('font', family='serif', size=9)
    matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('legend', fontsize=8)
    matplotlib.rc('figure', figsize=(3.33, 2.22))
    #  matplotlib.rc('figure.subplot', left=0.10, top=0.90, bottom=0.12, right=0.95)
    matplotlib.rc('axes', linewidth=0.5)
    matplotlib.rc('lines', linewidth=0.5)


def set_square_paper_rcs():
    matplotlib.rc('font', family='serif', size=9)
    matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('legend', fontsize=8)
    matplotlib.rc('figure', figsize=(3.33, 3.33))
    #  matplotlib.rc('figure.subplot', left=0.10, top=0.90, bottom=0.12, right=0.95)
    matplotlib.rc('axes', linewidth=0.5)
    matplotlib.rc('lines', linewidth=0.5)


def set_rcs():
    #  matplotlib.rc('font',**{'family':'sans-serif','sans-serif':['Times'],
    #               'serif':['Times'],'size':10})
    matplotlib.rc('font', family='serif')
    matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('legend', fontsize=10)
    matplotlib.rc('figure', figsize=(6, 4))
    matplotlib.rc('figure.subplot',
                  left=0.10,
                  top=0.90,
                  bottom=0.12,
                  right=0.95)
    matplotlib.rc('axes', linewidth=0.5)
    matplotlib.rc('lines', linewidth=0.5)


def set_slide_rcs():
    matplotlib.rc('font', family='serif', size=19)
    matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('legend', fontsize=19)
    matplotlib.rc('figure', figsize=(8, 6))
    matplotlib.rc('axes', linewidth=1.5)
    matplotlib.rc('lines', linewidth=2.5)


def set_square_slide_rcs():
    matplotlib.rc('font', family='serif', size=19)
    matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('legend', fontsize=19)
    matplotlib.rc('figure', figsize=(8, 8))
    matplotlib.rc('axes', linewidth=1.5)
    matplotlib.rc('lines', linewidth=2.5)


def set_poster_rcs():
    matplotlib.rc('font', family='serif', size=28)
    matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('legend', fontsize=24)
    matplotlib.rc('figure', figsize=(12, 9))
    matplotlib.rc('axes', linewidth=1.5)
    matplotlib.rc('lines', linewidth=4.5)


def setup_plot(flags):
    if flags.paper_mode:
        plt.figure(figsize=(3.33, 2.22))
        set_paper_rcs()
    elif flags.small_paper_mode:
        plt.figure(figsize=(2.2, 1.66))
        set_paper_rcs()
    elif flags.slide_mode:
        plt.figure(figsize=(8, 6))
        matplotlib.rc('font', family='serif', size=19)
        matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
        matplotlib.rc('text', usetex=True)
        matplotlib.rc('legend', fontsize=19)
        matplotlib.rc('figure', figsize=(8, 6))
        matplotlib.rc('axes', linewidth=1.5)
        matplotlib.rc('lines', linewidth=2.5)
    elif flags.poster_mode:
        plt.figure(figsize=(12, 9))
        matplotlib.rc('font', family='serif', size=28)
        matplotlib.rc('text.latex', preamble=r'\usepackage{times,mathptmx}')
        matplotlib.rc('text', usetex=True)
        matplotlib.rc('legend', fontsize=24)
        matplotlib.rc('figure', figsize=(12, 9))
        matplotlib.rc('axes', linewidth=1.5)
        matplotlib.rc('lines', linewidth=4.5)
    elif flags.stretched:
        plt.figure(figsize=(3.33, 1.2))
        set_paper_rcs()
    else:
        plt.figure()
        set_rcs()


def generate_ticks(min_val, max_val, increment, scale_down=1):
    tick = []
    tick_str = []
    print("Warning: Generating ticks at integer values")
    while min_val <= max_val:
        tick.append(min_val)
        tick_str.append(str(int(min_val / scale_down)))
        min_val += increment
    return tick, tick_str


def print_stats(vals):
    print("Number of values {}".format(len(vals)))
    avg = np.mean(vals)
    print('AVG: {}'.format(avg))
    median = np.median(vals)
    print('MEDIAN: {}'.format(median))
    min_val = np.min(vals)
    print('MIN: {}'.format(min_val))
    max_val = np.max(vals)
    print('MAX: {}'.format(max_val))
    stddev = np.std(vals)
    print('STDDEV: {}'.format(stddev))
    print('PERCENTILES:')
    perc1 = np.percentile(vals, 1)
    print('  1st: {}'.format(perc1))
    perc10 = np.percentile(vals, 10)
    print(' 10th: {}'.format(perc10))
    perc25 = np.percentile(vals, 25)
    print(' 25th: {}'.format(perc25))
    perc50 = np.percentile(vals, 50)
    print(' 50th: {}'.format(perc50))
    perc75 = np.percentile(vals, 75)
    print(' 75th: {}'.format(perc75))
    perc90 = np.percentile(vals, 90)
    print(' 90th: {}'.format(perc90))
    perc99 = np.percentile(vals, 99)
    print(' 99th: {}'.format(perc99))
    perc999 = np.percentile(vals, 99.9)
    print(' 99.9th: {}'.format(perc999))


def plot_cdf(flags,
             plot_file_name,
             cdf_vals,
             x_label,
             y_label,
             labels,
             bin_width=1,
             x_ticks_increment=None,
             show_legend=False,
             legend_outside_box=False,
             automatic_legend=False,
             colors=['r', 'b', 'g', 'c', 'm', 'y', 'k'],
             scale_x_axis=1,
             verbose=True):
    setup_plot(flags)
    max_cdf_val = 0
    index = 0
    for vals in cdf_vals:
        if verbose:
            print('Statistics for {}'.format(labels[index]))
            print_stats(vals)
        min_val = np.min(vals)
        max_val = np.max(vals)
        max_cdf_val = max(max_val, max_cdf_val)

        bin_range = int(max_val - min_val)
        num_bins = int(bin_range / bin_width)
        (n, bins, patches) = plt.hist(
            vals,
            bins=num_bins,
            log=False,
            density=True,
            cumulative=True,
            histtype='step',
            color=colors[index],
        )
        #lw=1.5)
        # hack to add line to legend
        plt.plot([-100], [-100],
                 label=labels[index],
                 color=colors[index],
                 linestyle='solid',
                 lw=1.5)
        # hack to remove vertical bar
        patches[0].set_xy(patches[0].get_xy()[:-1])

        index += 1

    plt.xlim(0, max_cdf_val)
    if x_ticks_increment is not None:
        ticks, ticks_str = generate_ticks(0, max_cdf_val, x_ticks_increment,
                                          scale_x_axis)
        plt.xticks(ticks, ticks_str)

    plt.ylim(0, 1.0)
    plt.yticks(np.arange(0.0, 1.001, 0.2),
               ["{:.1f}".format(x) for x in np.arange(0.0, 1.001, 0.2)])

    # Set the axis labels.
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    if automatic_legend:
        if flags.small_paper_mode:
            plt.legend(fontsize="x-small")
        else:
            plt.legend()
    elif show_legend:
        if legend_outside_box:
            plt.legend(loc=10,
                       frameon=False,
                       handlelength=1.5,
                       handletextpad=0.1,
                       bbox_to_anchor=(0.5, 1.1),
                       ncol=2)
        else:
            plt.legend(loc=2,
                       frameon=False,
                       handlelength=1.5,
                       handletextpad=0.1)

    plt.savefig('{}.{}'.format(plot_file_name, flags.file_format),
                format=flags.file_format,
                bbox_inches='tight')


def plot_timeline(flags,
                  plot_file_name,
                  all_x_vals,
                  all_y_vals,
                  x_label,
                  y_label,
                  labels,
                  x_ticks_increment=None,
                  y_ticks_increment=None,
                  show_legend=False,
                  legend_outside_box=False,
                  markers=['o', '+', '^', '1', '2', 'v', '*'],
                  colors=[
                      'r', 'b', 'g', 'c', 'm', 'y', 'k', 'tab:purple',
                      'tab:gray', 'tab:pink', 'tab:brown', 'tab:orange'
                  ],
                  cut_x_min=None,
                  cut_x_max=None,
                  cut_y_min=None,
                  cut_y_max=None,
                  line_width=0.0,
                  scale_down_x_ticks=1,
                  verbose=False):
    setup_plot(flags)

    index = 0
    min_x_val = sys.maxsize
    max_x_val = -sys.maxsize
    min_y_val = sys.maxsize
    max_y_val = -sys.maxsize

    for index in range(0, len(all_x_vals)):
        min_x_val = min(min_x_val, np.min(all_x_vals[index]))
        max_x_val = max(max_x_val, np.max(all_x_vals[index]))
        min_y_val = min(min_y_val, np.min(all_y_vals[index]))
        max_y_val = max(max_y_val, np.max(all_y_vals[index]))
        plt.plot(all_x_vals[index], [y for y in all_y_vals[index]],
                 label=labels[index],
                 color=colors[index],
                 marker=markers[index],
                 mfc='none',
                 mew=1.0,
                 mec=colors[index],
                 lw=line_width,
                 markersize=2)

    if verbose:
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

    if show_legend:
        if legend_outside_box:
            plt.legend(loc=10,
                       frameon=False,
                       handlelength=1.5,
                       handletextpad=0.1,
                       bbox_to_anchor=(0.5, 1.1),
                       ncol=2)
        else:
            plt.legend(loc=2,
                       frameon=False,
                       handlelength=1.5,
                       handletextpad=0.1)

    plt.savefig("{}.{}".format(plot_file_name, flags.file_format),
                format=flags.file_format,
                bbox_inches="tight")


def plot_box_and_whiskers(flags,
                          plot_file_name,
                          y_values,
                          y_label,
                          labels,
                          show_legend=False,
                          cut_y_max=None):
    setup_plot(flags)

    ax = plt.gca()
    bp = box_and_whisker.percentile_box_plot(ax, y_values, color='b')
    plt.xlim(0.5, len(labels) + 0.5)

    plt.xticks(range(1,
                     len(labels) + 1), [label for label in labels],
               rotation=30,
               ha='right')

    plt.ylabel(y_label)
    if cut_y_max:
        print('WARNING: Fixing max y val to {}'.format(cut_y_max))
        plt.ylim(0, cut_y_max)

    if show_legend:
        plt.legend(loc=1, frameon=False, handlelength=1.5, handletextpad=0.1)

    plt.savefig("{}.{}".format(plot_file_name, flags.file_format),
                format=flags.file_format,
                bbox_inches="tight")


def adjust_box_widths(g, fac):
    """Adjust the withs of a seaborn-generated boxplot.

    From https://stackoverflow.com/questions/56838187/how-to-create-spacing-between-same-subgroup-in-seaborn-boxplot
    """

    # iterating through Axes instances
    for ax in g.axes:

        # iterating through axes artists:
        for c in ax.get_children():

            # searching for PathPatches
            if isinstance(c, PathPatch):
                # getting current width of box:
                p = c.get_path()
                verts = p.vertices
                verts_sub = verts[:-1]
                xmin = np.min(verts_sub[:, 0])
                xmax = np.max(verts_sub[:, 0])
                xmid = 0.5 * (xmin + xmax)
                xhalf = 0.5 * (xmax - xmin)

                # setting new width of box
                xmin_new = xmid - fac * xhalf
                xmax_new = xmid + fac * xhalf
                verts_sub[verts_sub[:, 0] == xmin, 0] = xmin_new
                verts_sub[verts_sub[:, 0] == xmax, 0] = xmax_new

                # setting new width of median line
                for l in ax.lines:
                    if np.all(l.get_xdata() == [xmin, xmax]):
                        l.set_xdata([xmin_new, xmax_new])
