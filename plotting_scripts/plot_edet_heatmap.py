import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

argmax_edet_matrix = np.load("../experiments/waymo/argmax_edet_matrix.npy")

#mpl.rcParams.update(mpl.rcParamsDefault)

font = {'family': 'Times', 'size': 22}

import matplotlib
#matplotlib.rc('font', **font)
fig = plt.figure(figsize=(2.4, 1.66))
matplotlib.rc('font', family='serif', size=9)
matplotlib.rc('text.latex', preamble=[r'\usepackage{times,mathptmx}'])
matplotlib.rc('text', usetex=True)
matplotlib.rc('legend', fontsize=8)
matplotlib.rc('figure', figsize=(3.33, 2.22))
#  matplotlib.rc('figure.subplot', left=0.10, top=0.90, bottom=0.12, right=0.95)
matplotlib.rc('axes', linewidth=0.5)
matplotlib.rc('lines', linewidth=0.5)

ax = plt.gca()
ax.set_xticks(np.arange(-.5, 15, 5))
ax.set_yticks(np.arange(0, 12, 5) + 1)
ax.set_xticklabels([0, 10, 20, 30])
ax.set_yticklabels(reversed([0, 5, 10]))

plt.xlabel("Time [sec]", labelpad=0)
plt.ylabel("Scenario", labelpad=1)
im = plt.imshow(np.array(argmax_edet_matrix), cmap="YlOrRd", aspect='auto')

cbar = plt.colorbar(shrink=0.9)
cbar.set_ticks([])
cbar.set_ticks([0, 3.5, 7])
cbar.set_ticklabels(["EDet1", "EDet4", "EDet7"])
#plt.text(15.1, 6.85, "Accuracy", rotation=90, ha="center")

fig.savefig("scenario_edet_matrix.pdf", format="pdf", bbox_inches="tight")
