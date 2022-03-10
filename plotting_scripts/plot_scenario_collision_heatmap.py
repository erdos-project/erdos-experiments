from absl import app, flags

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import pandas as pd
import seaborn as sns
from utils import set_paper_rcs
from statistics import mean

# Log data is in experiments/scenario_runner/scenarios_collision

# Results Mask
## Set this mask in order to choose the configurations
results_mask = [0, 1, 2, 3, 4][::-1]
# Configuration Names
config_names = np.array(["125", "200", "250", "400", "500"])


def traffic_jam_scenario_results():
    # Traffic Jam Results
    traffic_jam_results = {
        8: np.array([7, 0, 0, 0, 0]),
        10: np.array([10, 2, 8, 0, 0]),
        12: np.array([10, 10, 10, 4, 10]),
    }

    traffic_jam_speeds = {
        (8, "125"): [
            2.3336666271190936, 2.5826942345693507, 2.625539900564399,
            2.4630885653172467, 2.5474155844862065, 2.5830371630678814,
            2.9037216236499677
        ],
        (10, "125"): [
            4.549161614960176, 9.123501712998973, 7.328066368129619,
            9.493209569093816, 5.0799484127535885, 9.420083260942288,
            9.07490694904878, 7.597166932791612, 5.0790518897275145,
            9.49311234516461
        ],
        (10, "200"): [7.667853546359189, 7.738065353513592],
        (10, "250"): [
            2.732328561290378, 3.468812357694637, 2.0975524221307356,
            6.061570795687315, 4.8025709869865505, 1.3419626712028525,
            6.084247339991617, 3.718696745559678
        ],
        (12, "200"): [
            10.531758195608015, 10.531814112290974, 6.631522605490724,
            6.9297279686017275, 6.423252767028788, 10.531805581678086,
            10.531838769120363, 10.531800790874662, 2.7269165523980265,
            8.988875048181203
        ],
        (12, "250"): [
            9.183825030411546, 9.546257054630678, 9.18365467155093,
            3.3383391536284384, 9.748980349432033, 9.655003360511543,
            8.960996593958063, 9.864865676587021, 3.330360395045713,
            9.325381779933496
        ],
        (12, "400"): [
            1.4169370020192595, 1.810372130904461, 5.829303748896193,
            1.728335600551587
        ],
        (12, "500"): [
            6.3355636784960705, 6.745425599812845, 8.544381621969812,
            3.1983212996461363, 6.77132040409377, 9.390516093446491,
            7.1840415321332625, 9.10907093102519, 2.850794076484411,
            6.905091535666133
        ],
    }

    traffic_jam_results_num_runs = 10
    xlabels = config_names[results_mask]

    # Get the max collision speed.
    traffic_jam_df = pd.concat(
        map(pd.DataFrame, [{
            "target_speed": [int(k[0])],
            "deadline": [str(k[1])],
            "collision_speed": np.max(v)
        } for k, v in filter(
            lambda x: x[0][0] % 2 == 0, traffic_jam_speeds.items())] + [
                pd.DataFrame({
                    "target_speed": [12],
                    "deadline": ["D3"],
                    "collision_speed": [6.8]
                })
            ])).reset_index().pivot(
                "deadline", "target_speed",
                "collision_speed").fillna(0).sort_index(ascending=False)
    return traffic_jam_df


def person_behind_car_results():
    # Person behind car results
    person_behind_car_results = {
        11: np.array([0, 0, 0, 0, 0]),
        12: np.array([5, 0, 7, 7, 10]),
        13: np.array([10, 10, 10, 10, 10]),
    }

    # Person behind car at point of collision
    person_behind_car_speeds = {
        (11, 125): [0],
        (12, "125"): [
            3.834851204872633, 3.9365858732633763, 3.8334822182059702,
            3.8337721935180014, 5.208377546480244
        ],
        (12, "250"): [
            3.7953475835718686, 4.34661542278236, 2.731714089964147,
            0.0058617744623146685, 3.1015998530176336, 0.2653812091929323,
            0.00730250338238119
        ],
        (12, "400"): [
            0.01470682247600576, 6.559455625118717, 2.731714089964147,
            0.0058617744623146685, 3.1015998530176336, 0.2653812091929323,
            0.00730250338238119
        ],
        (12, "500"): [
            6.918861561774308, 6.701498155369195, 4.9201436272610355,
            6.472134737412674, 4.686640993619152, 5.738583738449307,
            6.356266651477661, 4.691416215752175, 6.52558194984853,
            6.722929295938557
        ],
        (13, "125"): [
            8.81798807568223, 8.818074793193281, 5.738995085355604,
            5.765227266613277, 5.883049294742132, 5.765359428179171,
            5.987272828846497, 2.7965570729349127, 5.7390904377090335,
            6.274690427927436
        ],
        (13, "200"): [
            5.9017141508201, 5.901670470250733, 5.78379213046563,
            5.90161336360544, 5.90163650520926, 5.901510357728479,
            7.442600546244366, 5.996907994320218, 6.839073587277239,
            5.7958505930830935
        ],
        (13, "250"): [
            7.650985800643449, 7.520039109722852, 7.425590457363744,
            6.45677384924743, 6.153519550393394, 7.275715179377407,
            6.086156706552252, 6.833390033876289, 7.510461689390102,
            7.199112084906058
        ],
        (13, "400"): [
            6.871977978125713, 6.928225693231432, 6.58048870056221,
            9.675635104924966, 6.854494174861473, 6.655918831208402,
            6.902813783900431, 6.4951567264122385, 4.765948032788043,
            3.9909460272835573
        ],
        (13, "500"): [
            3.575892787995359, 2.845853376905324, 9.38712854359166,
            9.968601910457089, 8.64021717731768, 10.250982958536811,
            9.443490263091725, 9.444099573987103, 3.0987921147602395,
            7.201869134486841
        ],
    }

    person_behind_car_results_num_runs = 10
    xlabels = config_names[results_mask]

    # Get the max collision speed.
    person_behind_car_df = pd.concat(
        map(pd.DataFrame, [{
            "target_speed": [int(k[0])],
            "deadline": [str(k[1])],
            "collision_speed": np.max(v)
        } for k, v in filter(lambda x: x[0][0] in {11, 12, 13},
                             person_behind_car_speeds.items())] +
            [
                pd.DataFrame({
                    "target_speed": [13],
                    "deadline": ["D3"],
                    "collision_speed": [8.7]
                })
            ])).reset_index().pivot(
                "deadline", "target_speed",
                "collision_speed").fillna(0).sort_index(ascending=False)
    return person_behind_car_df


def main(argv):
    traffic_jam_df = traffic_jam_scenario_results()
    person_behind_car_df = person_behind_car_results()
    dfs = [person_behind_car_df, traffic_jam_df]

    set_paper_rcs()
    fig, axes = plt.subplots(2,
                             2,
                             sharex=False,
                             sharey=False,
                             figsize=(3.33, 1.25),
                             gridspec_kw={
                                 'height_ratios': [1, 5],
                                 'hspace': 0.1
                             })

    plt.xlabel("Deadline [ms]")
    plt.ylabel("common Y")

    max_speed = max([max(df) for df in dfs])

    for i in range(0, 2):
        ax = axes[0][i]
        df = dfs[i]
        g = sns.heatmap(df[df.index == "D3"],
                        cmap="RdYlGn_r",
                        annot=True,
                        linewidths=1.0,
                        linecolor="black",
                        ax=ax,
                        vmin=0,
                        vmax=max_speed,
                        cbar=False)
        ax.set_xlabel("")
        ax.set_xticks([])
        ax.set_ylabel("")
        if i == 0:
            scenario = "Person Behind Truck"
            g.set_yticklabels(g.get_yticklabels(), rotation=0)
            # plt.xlabel("Target Speed [m/s]", ax=ax)
            # ax.xaxis.set_label_coords(1.2, -0.15)
        else:
            scenario = "Traffic Jam"
            ax.set_ylabel("")
            ax.set_yticks([])
        ax.set_title(scenario)

        ax = axes[1][i]
        df = df[(df.index == "250") | (df.index == "125") | (df.index == "200")
                | (df.index == "400") | (df.index == "500")]
        g = sns.heatmap(df,
                        cmap="RdYlGn_r",
                        annot=True,
                        linewidths=0.1,
                        linecolor="black",
                        ax=ax,
                        vmin=0,
                        vmax=max_speed,
                        cbar=False)
        ax.set_xlabel("")
        ax.set_ylabel("")
        if i == 1:
            ax.set_ylabel("")
            ax.set_yticks([])
        else:
            ax.set_ylabel("Deadline [ms]")
            ax.set_yticks([4.5, 3.5, 2.5, 1.5, 0.5])
            ax.set_yticklabels(['125', '200', '250', '400', '500'], rotation=0)
            #g.set_yticklabels(g.get_yticklabels(),rotation=0)

    mappable = g.get_children()[0]
    cbar = plt.colorbar(mappable, ax=axes, location="right")
    cbar.ax.set_ylabel("Collision speed [m/s]", rotation=90)
    fig.text(0.45, -0.12, 'Driving Speed [m/s]', ha='center')

    plt.savefig("speed-deadline-dd-heatmap.pdf", bbox_inches='tight')


if __name__ == '__main__':
    app.run(main)
