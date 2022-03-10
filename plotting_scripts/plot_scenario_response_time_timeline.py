from absl import app, flags
import brewer2mpl
import os, csv, json
import matplotlib
from matplotlib.patches import Patch
import matplotlib.pyplot as plt
import numpy as np
from utils import set_paper_rcs

FLAGS = flags.FLAGS
flags.DEFINE_string(
    'static_log_base',
    '../experiments/scenario_runner/person_behind_car_response_time/person_behind_car_detection_200_planning_309_target_speed_12_Hz_5',
    'Base name of the static config log files')
flags.DEFINE_string(
    'dynamic_log_base',
    '../experiments/scenario_runner/person_behind_car_response_time/person_behind_car_dynamic_deadlines_target_speed_12_Hz_5',
    'Base name of the dynamic config log files')


class Result:

    def __init__(self):
        self.end_to_end_runtimes = {}
        self.detection_runtimes = {}
        self.tracking_runtimes = {}
        self.planning_runtimes = {}
        self.prediction_runtimes = {}


def get_config_data(file_name):
    final_result = Result()
    # Get the end-to-end runtime from the CSV file.
    csv_file = "{}.csv".format(file_name)
    if not os.path.exists(csv_file):
        print("[x] Skipping {} because it does not exist.".format(csv_file))
        return

    end_to_end_runtimes = set()
    with open(csv_file, 'r') as f:
        csv_reader = csv.reader(f)
        for row in csv_reader:
            if row[2] == "end-to-end-runtime":
                runtime, ts = float(row[-1]), float(row[1])
                final_result.end_to_end_runtimes[ts] = runtime
            elif row[2] == "collision":
                intensity = float(row[-1])

    # Get the decomposed runtimes from the JSON file.
    json_file = "{}.json".format(file_name)
    if not os.path.exists(json_file):
        print("[x] Skipping {} because it does not exist.".format(json_file))
        return
    with open(json_file, 'r') as f:
        json_reader = json.load(f)
        for row in json_reader:
            ts = float(row['args']['timestamp'][1:-1])
            if ts not in final_result.end_to_end_runtimes:
                continue
            if 'planning_operator' in row['name']:
                final_result.planning_runtimes[ts] = (float(row['dur']) / 1000)
            if 'efficientdet_operator' in row['name']:
                final_result.detection_runtimes[ts] = (float(row['dur']) /
                                                       1000)
            if 'tracker_sort' in row['name']:
                final_result.tracking_runtimes[ts] = (float(row['dur']) / 1000)
            if 'prediction_operator' in row['name']:
                final_result.prediction_runtimes[ts] = (float(row['dur']) /
                                                        1000)
            #if 'location_finder_history_operator' in row['name']:
            #    final_result.fusion_runtimes.append(
            #        float(row['dur']) / 1000)
            #if 'pid_agent_operator' in row['name']:
            #    final_result.control_runtimes.append(
            #        float(row['dur']) / 1000)
    return final_result


def main(argv):
    bmap = brewer2mpl.get_map('Set2', 'qualitative', 7)
    colors = bmap.mpl_colors
    e2e_color, perception_color, planning_color = colors[0], colors[2], colors[
        6]

    static_results = get_config_data(FLAGS.static_log_base)
    print("E2E: {}, DET: {}, PLAN: {}, TRK: {}, PRED: {}".format(
        len(static_results.end_to_end_runtimes.keys()),
        len(static_results.detection_runtimes.keys()),
        len(static_results.planning_runtimes.keys()),
        len(static_results.tracking_runtimes.keys()),
        len(static_results.prediction_runtimes.keys())))

    dynamic_results = get_config_data(FLAGS.dynamic_log_base)
    print("E2E: {}, DET: {}, PLAN: {}, TRK: {}, PRED: {}".format(
        len(dynamic_results.end_to_end_runtimes.keys()),
        len(dynamic_results.detection_runtimes.keys()),
        len(dynamic_results.planning_runtimes.keys()),
        len(dynamic_results.tracking_runtimes.keys()),
        len(dynamic_results.prediction_runtimes.keys())))

    # Plot the timeline of the PBC results.
    fig = plt.figure(figsize=(3.33, 1.66))
    set_paper_rcs()
    ax = fig.add_subplot(211)

    constrain_to = [11, 15]
    #constrain_to = [0, 111111]
    # Pedestrian at 23, Truck at 16

    # Dynamic Results
    ax2 = fig.add_subplot(212)

    TTD = {
        21617: 141,
        21817: 141,
        22017: 74,
        22217: 42,
        22417: 24,
        22617: 42,
        22817: 74,
        23017: 74,
        23217: 141,
        23417: 141,
        23617: 141,
    }

    # Dynamic deadlines
    DEADLINES = {
        21617: 345.26368323628736,
        21817: 239.73704564473405,
        22017: 199.41558492650447,
        22217: 136.11660387316184,
        22417: 115.97638020699821,
        22617: 130.8113305563115,
        22817: 162.37504049465466,
        23017: 213.95208640672976,
        23217: 296.32577543334486,
        23417: 465.46476001221225,
    }

    AVG_DET_RUNTIMES = {
        #Det: (p50, p99)
        24: (15.4145, 50.39676),
        42: (21.3215, 54.89371),
        74: (28.789, 72.12038),
        141: (46.164, 116.40679),
        200: (57.075, 146.77204),
    }

    xd, yd_e2e, yd_det, yd_plan, yd_track, yd_pred = [], [], [], [], [], []
    deadlines = []
    min_timestampd = min(dynamic_results.end_to_end_runtimes.keys())
    for timestamp in sorted(dynamic_results.end_to_end_runtimes.keys()):
        time = (timestamp - min_timestampd) / 1000
        if time >= constrain_to[0] and time <= constrain_to[1]:
            xd.append(time)
            if timestamp in DEADLINES:
                deadlines.append(DEADLINES[timestamp])
            else:
                deadlines.append(500)
            detector_in_use = 200
            detection_runtime = dynamic_results.detection_runtimes[timestamp]
            if timestamp in TTD:
                detector_in_use = TTD[timestamp]
                detection_runtime = AVG_DET_RUNTIMES[detector_in_use][-1]
            tracking_runtime = detection_runtime + dynamic_results.tracking_runtimes[
                timestamp]
            prediction_runtime = tracking_runtime + dynamic_results.tracking_runtimes[
                timestamp]
            yd_e2e.append(dynamic_results.end_to_end_runtimes[timestamp] -
                          detector_in_use + detection_runtime)
            yd_det.append(detection_runtime)
            yd_track.append(tracking_runtime)
            yd_pred.append(prediction_runtime)
            yd_plan.append(prediction_runtime +
                           dynamic_results.planning_runtimes[timestamp])
            if dynamic_results.detection_runtimes[timestamp] > 200:
                print(timestamp)

    print("Length of xd is {}".format(len(xd)))

    # Fill the area for the perception curve.
    ax2.fill_between(xd, 0, yd_det, facecolor=perception_color)

    # Fill the area for the planning curve.
    ax2.fill_between(xd, yd_det, yd_plan, facecolor=planning_color)

    # Fill the area for the e2e curve.
    ax2.fill_between(xd, yd_plan, yd_e2e, facecolor=e2e_color)

    # Plot the deadline curve
    ax2.plot(xd, deadlines, '--', lw=1, color='r')

    # Set the same axis on both ones.
    ax2.set_ylim(0, 600)
    ax2.set_yticks([0, 200, 400, 600])

    x, y_e2e, y_det, y_plan, y_track, y_pred = [], [], [], [], [], []
    min_timestamp = min(static_results.end_to_end_runtimes.keys())
    for timestamp in sorted(static_results.end_to_end_runtimes.keys()):
        time = (timestamp - min_timestamp) / 1000
        if time >= constrain_to[0] and time <= constrain_to[1]:
            x.append((timestamp - min_timestamp) / 1000)
            detection_runtime = static_results.detection_runtimes[timestamp]
            tracking_runtime = detection_runtime + static_results.tracking_runtimes[
                timestamp]
            prediction_runtime = tracking_runtime + static_results.tracking_runtimes[
                timestamp]
            y_e2e.append(static_results.end_to_end_runtimes[timestamp] - 200 +
                         detection_runtime)
            y_det.append(detection_runtime)
            y_track.append(tracking_runtime)
            y_pred.append(prediction_runtime)
            y_plan.append(prediction_runtime +
                          static_results.planning_runtimes[timestamp])
            if static_results.detection_runtimes[timestamp] > 200:
                print(timestamp)

    y_det += [0] * (len(xd) - len(x))
    y_e2e += [0] * (len(xd) - len(x))
    y_plan += [0] * (len(xd) - len(x))
    print("Length of x is {}".format(len(x)))

    # Fill the area for the perception curve.
    ax.fill_between(xd, 0, y_det, facecolor=perception_color)

    # Fill the area for the planning curve.
    ax.fill_between(xd, y_det, y_plan, facecolor=planning_color)

    # Fill the area for the e2e curve.
    ax.fill_between(xd, y_plan, y_e2e, facecolor=e2e_color)

    # Set limit and don't show x-axis
    ax.set_ylim(0, 600)
    ax.set_yticks([0, 200, 400, 600])
    ax.axes.get_xaxis().set_visible(False)

    # Plot the deadline lines on both the axis.
    ax.plot([xd[0], xd[-1]], [500, 500], '--', color='red', lw=1)

    # Dynamic detection
    first_dyn_detection = 21417
    ax2.vlines(linestyles='dashed',
               x=((first_dyn_detection - min_timestampd) / 1000),
               ymin=0,
               ymax=450,
               color='k',
               lw=1)

    # Static detection
    first_static_detection = 21417
    ax.vlines(linestyles='dashed',
              x=((first_static_detection - min_timestamp) / 1000),
              ymin=0,
              ymax=450,
              color='k',
              lw=1)

    # Static collision
    static_collision = 23272
    ax.vlines(linestyles='dashed',
              x=((static_collision - min_timestamp) / 1000),
              ymin=0,
              ymax=450,
              color='k',
              lw=1)

    # Add the required text
    ax.text(13.35, 395, 'D3 (Static Deadlines)', color='red', fontsize=7)
    ax2.text(13.4, 380, 'D3 (Dynamic Deadlines)', color='red', fontsize=7)

    ax.text(11.47, 350, 'Visible', color='k', fontsize=7)
    ax.text(12.6, 350, 'Collision', color='k', fontsize=7)

    legend_elements = [
        Patch(facecolor=e2e_color, label="Total"),
        Patch(facecolor=perception_color, label="Perception"),
        Patch(facecolor=planning_color, label="Planning"),
    ]
    plt.legend(handles=legend_elements,
               loc=(0.68, 1.12),
               framealpha=0,
               handlelength=1,
               prop={'size': 8},
               labelspacing=0.03)

    fig.text(0.5, -0.05, "Time [s]", ha="center", color='k')
    fig.text(0,
             0.5,
             "Response Time [ms]",
             ha="center",
             rotation='vertical',
             va="center",
             color='k')

    plt.savefig("dynamic_deadline_timeline.pdf",
                bbox_inches="tight",
                dpi=100,
                format="pdf")


if __name__ == '__main__':
    app.run(main)
