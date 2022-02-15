import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import brewer2mpl
from utils import *
import systems_utils
from systems_utils import get_colors, COLOR_DICT, HATCH_DICT
from matplotlib.patches import Patch, Rectangle
from matplotlib.lines import Line2D
import matplotlib.patheffects as path_effects
import matplotlib as mpl
import fire

mpl.rcParams["hatch.linewidth"] = 0.4

bmap = brewer2mpl.get_map("Set2", "qualitative", 7)
markers = ["x", "o", "^", "v", "d"]
hatches = ["", "\\\\\\", "..."]

plt.figure(figsize=(3.33, 2.22))
set_paper_rcs()

FIGSIZE = (2.7, 1.2)

HATCH_DICT["Flink \([Ii]ntra"] = "...."


def plot_msg_size_latency(
    erdos_intra_process_filename: str,
    flink_intra_process_filename: str,
    erdos_inter_process_filename: str,
    flink_inter_process_filename: str,
    ros_inter_process_filename: str,
    output_filename: str = "graphs/msg_size_latency.pdf",
):
    intra_process_csvs = {
        "ERDOS (intra)": erdos_intra_process_filename,
        "Flink (intra)": flink_intra_process_filename,
    }

    inter_process_csvs = {
        "ERDOS (inter)": erdos_inter_process_filename,
        "Flink (inter)": flink_inter_process_filename,
        "ROS": ros_inter_process_filename,
    }

    fig = plt.figure(figsize=FIGSIZE)

    ax1 = fig.add_subplot(1, 2, 1)

    intra_process_df = systems_utils.load_csvs(intra_process_csvs)

    colors = get_colors(intra_process_df.name.unique())

    g = sns.barplot(
        x="latency_ms",
        y="msg_size",
        data=intra_process_df,
        hue="name",
        palette=colors,
        saturation=1,
        ci=90,
        orient="h",
        alpha=0.99,
        estimator=np.median,
    )

    g.set(xlabel=None)
    plt.title("Intra-Process", fontsize=mpl.rcParams["font.size"])

    itc_systems = intra_process_df.name.unique()
    for i, box in enumerate(ax1.patches):
        color = box.get_facecolor()[:-1]
        j = colors.index(color)
        system = itc_systems[j]
        box.set_hatch(HATCH_DICT[system])
        box.set_edgecolor("black")
        if box.get_width() > 0.5:
            y = box.get_y() + box.get_height() / 2 - 0.05
            x = 0.41
            text = ax1.text(
                x,
                y,
                systems_utils.bold(f"{box.get_width():.1f}"),
                fontsize=9,
                va="center",
                color="white",
            )
            text.set_path_effects(
                [
                    path_effects.Stroke(linewidth=1, foreground="black"),
                    path_effects.Normal(),
                ]
            )

    ax1.legend().set_visible(False)

    yticks, yticklabels = plt.yticks()
    new_yticklabels = [
        systems_utils.msg_size_to_str(int(y.get_text())) for y in yticklabels
    ]
    g.set(ylabel="Message size")
    ax1.set_ylim(ax1.get_ylim()[::-1])
    ax1.set_xlim(0, 0.5)
    ax1.set_xticks([0.25 * i for i in range(3)])

    ax2 = fig.add_subplot(1, 2, 2, sharey=ax1)

    inter_process_df = systems_utils.load_csvs(inter_process_csvs)

    ipc_systems = inter_process_df.name.unique()
    colors = get_colors(ipc_systems)

    g = sns.barplot(
        x="latency_ms",
        y="msg_size",
        data=inter_process_df,
        hue="name",
        palette=colors,
        saturation=1,
        ci=90,
        orient="h",
        alpha=0.99,
        estimator=np.median,
    )

    g.set(ylabel=None, xlabel=None)

    plt.title("Inter-Process", fontsize=mpl.rcParams["font.size"])
    plt.setp(ax2.get_yticklabels(), visible=False)

    ax2.set_xlim(0, 10)
    ax2.set_ylim(ax2.get_ylim()[::-1])
    ax2.legend().set_visible(False)

    ax1.set_yticklabels(new_yticklabels)

    for i, box in enumerate(ax2.patches):
        color = box.get_facecolor()[:-1]
        j = colors.index(color)
        system = ipc_systems[j]
        box.set_hatch(HATCH_DICT[system])
        box.set_edgecolor("black")
        if box.get_width() > 10:
            y = box.get_y() + box.get_height() / 2.5
            x = 7.75
            text = ax2.text(
                x,
                y,
                systems_utils.bold(f"{box.get_width():.1f}"),
                fontsize=5.5,
                va="center",
                color="white",
            )
            text.set_path_effects(
                [
                    path_effects.Stroke(linewidth=1, foreground="black"),
                    path_effects.Normal(),
                ]
            )

    ax2.text(-1, -2.1, "Latency [ms]", ha="center")

    legend_elements = [
        Patch(
            facecolor=COLOR_DICT[system],
            hatch=HATCH_DICT[system],
            label=system.split(" ")[0],
            alpha=0.99,
            edgecolor="black",
        )
        for system in list(["ROS", "Flink (intra)", "CarFlow (intra)"])
    ]

    ax1.legend(
        handles=legend_elements,
        loc=(0.35, 0.05),
        title=None,
        edgecolor="white",
        facecolor="white",
        borderpad=0,
        handlelength=1,
        framealpha=0,
    )

    set_paper_rcs()

    plt.savefig(output_filename, bbox_inches="tight")


if __name__ == "__main__":
    fire.Fire(plot_msg_size_latency)
