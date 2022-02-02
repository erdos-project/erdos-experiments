import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.gridspec as gridspec
import brewer2mpl
from utils import *
from matplotlib.patches import Patch, Rectangle
from matplotlib.lines import Line2D
import matplotlib.patheffects as path_effects
import matplotlib as mpl
import pylot_color_utils

mpl.rcParams['hatch.linewidth'] = 0.4


def bold(s):
    return r"\textbf{" + s + r"}"


collisions = pd.DataFrame({
    # orange, gray, yellow, green
    "setup": [
        "Periodic\n(ROS)", "No\ndeadlines", "Static\ndeadlines",
        "Dynamic\ndeadlines"
    ],
    "collisions": [78, 36, 34, 25]
})

fig = plt.figure(figsize=(3.33, 0.8))
set_paper_rcs()

colors = pylot_color_utils.get_colors(
    ["ROS", "No-Constraints", "Deadlines", "Policy"])

g = sns.barplot(y="collisions",
                x="setup",
                data=collisions,
                palette=colors,
                saturation=1,
                ci="sd",
                alpha=.99,
                estimator=np.median,
                edgecolor="black")

g.set(xlabel=None, ylabel="Number of\ncollisions")
plt.ylim(0, 80)
plt.yticks([0, 40, 80])

ax = plt.gca()

max_val = collisions.collisions.max()

for i, box in enumerate(ax.patches):
    y = 5  # box.get_y() + box.get_height() - 15
    x = box.get_x() + box.get_width() / 2
    val_text = ax.text(x,
                       y,
                       bold(f"{box.get_height():.0f}"),
                       fontsize=7,
                       ha="center",
                       color="black",
                       fontweight="bold",
                       size=14)
    #     val_text.set_path_effects([path_effects.Stroke(linewidth=1, foreground='black'),
    #                    path_effects.Normal()])
    if box.get_height() < 70:
        nx = max_val / box.get_height()
        y = box.get_y() + box.get_height() + 5
        nx_text = ax.text(x,
                          y,
                          f"{nx:.1f}x",
                          fontsize=7,
                          ha="center",
                          color="black",
                          size=14)

plt.savefig("pylot_collisions.pdf", bbox_inches="tight")
