import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import brewer2mpl
from utils import *
import systems_utils
from systems_utils import get_colors, HATCH_DICT
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


def plot_broadcast_latency(
    erdos_intra_process_filename: str,
    flink_intra_process_filename: str,
    erdos_inter_process_filename: str,
    flink_inter_process_filename: str,
    ros_inter_process_filename: str,
    output_filename: str = "graphs/broadcast_latency.pdf",
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

    intra_process_df = systems_utils.load_csvs(intra_process_csvs)
    inter_process_df = systems_utils.load_csvs(inter_process_csvs)

    fig = plt.figure(figsize=FIGSIZE)

    ax1 = fig.add_subplot(1, 2, 1)

    itc_systems = intra_process_df.name.unique()
    colors = get_colors(itc_systems)

    g = sns.barplot(
        y="num_receivers",
        x="latency_ms",
        data=intra_process_df,
        hue="name",
        palette=colors,
        saturation=1,
        orient="h",
        ci="sd",
        alpha=0.99,
        estimator=np.median,
    )

    plt.xlabel(None)
    plt.title("Intra-Process", fontsize=mpl.rcParams["font.size"])
    g.set(ylabel="Num. Receivers")

    for i, box in enumerate(ax1.patches):
        color = box.get_facecolor()[:-1]
        j = colors.index(color)
        system = itc_systems[j]
        box.set_hatch(HATCH_DICT[system])
        box.set_edgecolor("black")
        if box.get_width() > 5:
            y = box.get_y() + box.get_height() / 2
            x = 3.2
            text = ax1.text(
                x,
                y,
                f"{box.get_width():.1f}",
                fontsize=8,
                va="center",
                color="white",
                fontweight="bold",
            )
            text.set_path_effects(
                [
                    path_effects.Stroke(linewidth=1, foreground="black"),
                    path_effects.Normal(),
                ]
            )

    ax1.legend().set_visible(False)

    ax1.set_ylim(ax1.get_ylim()[::-1])
    # ax1.set_xlim(0, 4)
    plt.xlim(0, 5)
    # ax1.set_xticks(list(range(0, 14, 4)))

    ax2 = fig.add_subplot(1, 2, 2, sharey=ax1)

    ipc_systems = inter_process_df.name.unique()
    colors = get_colors(ipc_systems)

    g = sns.barplot(
        y="num_receivers",
        x="latency_ms",
        data=inter_process_df,
        hue="name",
        palette=colors,
        saturation=1,
        ci="sd",
        orient="h",
        alpha=0.99,
        estimator=np.median,
    )

    g.set(ylabel=None, xlabel=None)
    plt.setp(ax2.get_yticklabels(), visible=False)
    plt.title("Inter-Process", fontsize=mpl.rcParams["font.size"])

    ax2.set_xlim(0, 60)
    ax2.set_ylim(ax2.get_ylim()[::-1])
    ax2.legend().set_visible(False)
    ax2.set_xticks([0, 20, 40, 60])

    for i, box in enumerate(ax2.patches):
        color = box.get_facecolor()[:-1]
        j = colors.index(color)
        system = ipc_systems[j]
        box.set_hatch(HATCH_DICT[system])
        box.set_edgecolor("black")

    ax2.text(-1, -2.5, "Latency [ms]", ha="center")

    plt.savefig(output_filename, bbox_inches="tight")


if __name__ == "__main__":
    fire.Fire(plot_broadcast_latency)
