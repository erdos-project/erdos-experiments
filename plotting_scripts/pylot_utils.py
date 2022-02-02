import ast
import csv
import json

from absl import flags

import numpy as np
import pandas as pd

FLAGS = flags.FLAGS


class ProfileEvent(object):
    def __init__(self, json_dict):
        self.name = json_dict['name']
        self.event_time = float(json_dict['ts']) / 1000.0  # in ms
        self.runtime = float(json_dict['dur'])  # in us
        self.sim_time = int(
            json_dict['args']['timestamp'].strip('][').split(', ')[0])


class ProfileEvents(object):
    def __init__(self, profile_file, no_offset=False):
        data = None
        first_sim_time = None
        with open(profile_file) as prof_file:
            data = json.load(prof_file)
        self.events = []
        for entry in data:
            event = ProfileEvent(entry)
            if first_sim_time is None:
                first_sim_time = event.sim_time
            if no_offset:
                event.sim_time += FLAGS.ignore_first_sim_time_ms
            else:
                event.sim_time -= first_sim_time + FLAGS.ignore_first_sim_time_ms

            if event.sim_time >= 0:
                self.events.append(event)

    def check_if_timestamps_overlapped(self):
        """Checks if a component got delayed because its run for the previous
        timestamp didn't yet complete."""
        planning_end = 0
        planning_t = 0
        prediction_end = 0
        prediction_t = 0
        loc_end = 0
        loc_t = 0
        tracker_end = 0
        tracker_t = 0
        detection_end = 0
        detection_t = 0
        for event in self.events:
            end_time = event.event_time + event.runtime / 1000
            if event.name == 'planning_operator.on_watermark':
                if prediction_end < planning_end and prediction_t != planning_t:
                    print('Prediction from {} finished at {} before planning'
                          ' from {} finished at {}'.format(
                              prediction_t, prediction_end, event.sim_time,
                              planning_end))
                if end_time > planning_end:
                    planning_end = end_time
                    planning_t = event.sim_time
            elif (event.name == 'linear_prediction_operator.on_watermark'
                  or event.name ==
                  'linear_prediction_operator.generate_predicted_trajectories'
                  ):
                if loc_end < prediction_end and loc_t != prediction_t:
                    print(
                        'Loc find from {} finished at {} before prediction from'
                        ' {} finished at {}'.format(loc_t, loc_end,
                                                    event.sim_time,
                                                    prediction_end))
                if end_time > prediction_end:
                    prediction_end = end_time
                    prediction_t = event.sim_time
            elif (event.name ==
                  'center_camera_location_finder_history_operator.on_watermark'
                  ):
                if tracker_end < loc_end and tracker_t != loc_t:
                    print('Tracker from {} finished at {} before loc find from'
                          ' {} finished at {}'.format(tracker_t, tracker_end,
                                                      loc_t, loc_end))
                if end_time > loc_end:
                    loc_end = end_time
                    loc_t = event.sim_time
            elif event.name == 'tracker_sort.on_watermark':
                if detection_end < tracker_end and detection_t != tracker_t:
                    print('Detection from {} finished at {} before tracker '
                          'from {} finished at {}'.format(
                              detection_t, detection_end, tracker_t,
                              tracker_end))
                if end_time > tracker_end:
                    tracker_end = end_time
                    tracker_t = event.sim_time
            elif event.name == 'efficientdet_operator.on_watermark':
                if end_time > detection_end:
                    detection_end = end_time
                    detection_t = event.sim_time

    def get_runtimes(self,
                     event_name,
                     unit='ms',
                     timestamps_with_obstacles=None):
        runtimes = []
        for event in self.events:
            if (event.name == event_name
                    and (timestamps_with_obstacles is None
                         or event.sim_time in timestamps_with_obstacles)):
                if unit == 'ms':
                    runtimes.append(event.runtime / 1000)
                elif unit == 'us':
                    runtimes.append(event.runtime)
                else:
                    raise ValueError('Unexpected unit {}'.format(unit))
        return runtimes

    def get_filtered_runtimes(self,
                              event_name,
                              unit='ms',
                              timestamps_to_ban=None):
        runtimes = []
        for event in self.events:
            if event.name == event_name:
                if (timestamps_to_ban is not None
                        and event.sim_time in timestamps_to_ban):
                    runtimes.append(-1)
                else:
                    if unit == 'ms':
                        runtimes.append(event.runtime / 1000)
                    elif unit == 'us':
                        runtimes.append(event.runtime)
                    else:
                        raise ValueError('Unexpected unit {}'.format(unit))
        return runtimes

    def get_inter_exec(self, event_name):
        inter_exec = []
        last_event = None
        for event in self.events:
            if event.name == event_name:
                if last_event:
                    inter_exec.append(event.event_time - last_event.event_time)
                last_event = event
        return inter_exec

    def get_timeline(self, event_name, unit='ms'):
        timestamps = []
        runtimes = []
        for event in self.events:
            if event.name == event_name:
                timestamps.append(event.sim_time)
                if unit == 'ms':
                    runtimes.append(event.runtime / 1000)
                elif unit == 'us':
                    runtimes.append(event.runtime)
                else:
                    raise ValueError('Unexpected unit {}'.format(unit))
        return timestamps, runtimes


def read_end_to_end_runtimes(csv_file_path,
                             unit='ms',
                             timestamps_with_obstacles=None):
    first_sim_time = None
    csv_file = open(csv_file_path)
    csv_reader = csv.reader(csv_file)
    sim_times = []
    runtimes = []
    for row in csv_reader:
        sim_time = int(row[1])
        if not first_sim_time:
            first_sim_time = sim_time
        sim_time -= first_sim_time + FLAGS.ignore_first_sim_time_ms
        if (row[2] == 'end-to-end-runtime' and sim_time >= 0
                and (timestamps_with_obstacles is None
                     or sim_time in timestamps_with_obstacles)):
            sim_times.append(sim_time)
            if unit == 'ms':
                runtimes.append(float(row[3]))
            elif unit == 'us':
                runtimes.append(float(row[3]) * 1000)
            else:
                raise ValueError('Unexpected unit {}'.format(unit))
    return (sim_times, runtimes)


def converter(x):
    return ast.literal_eval(x)


def get_timestamps_with_obstacles(filename, obstacle_distance_threshold=10):
    """Finds timestamps when we detected obstacles."""
    print(filename)
    df = pd.read_csv(
        filename,
        names=["timestamp", "ms", "log_label", "label_info", "label_value"])
    df = df.dropna()
    df['label_value'] = df['label_value'].str.replace(" ", ", ")
    df['label_value'] = df['label_value'].apply(converter)

    obstacles = df[df['log_label'] == 'obstacle']
    obstacles = obstacles.set_index('ms')
    pose = df[df['log_label'] == 'pose']

    timestamps = []
    first_timestamp = df["ms"].min()
    for t, p in pose[["ms", "label_value"]].values:
        if t not in obstacles.index:
            continue
        obs = obstacles.loc[t]['label_value']
        if isinstance(obs, list):
            obs = [obs]
        else:
            obs = obs.values
        for o in obs:
            dist = np.linalg.norm(np.array(p) - np.array(o))
            if 0 < dist <= obstacle_distance_threshold:
                timestamps.append(t - first_timestamp)
    print("Selected {} timestamps".format(len(timestamps)))
    return timestamps


def fix_pylot_profile(file_path):
    with open(file_path, 'r') as f:
        contents = f.read()
    if contents[0] == "[":
        return
    print("Fixing Pylot {} json file".format(file_path))
    with open(file_path, 'w') as f:
        f.write("[\n")
        f.write(contents[:-2])
        f.write("\n]")


def read_challenge_runtimes(csv_file_path):
    csv_file = open(csv_file_path)
    csv_reader = csv.reader(csv_file)
    sensor_send_runtime = {}

    sim_times = []
    sensor_times = []
    e2e_runtimes = []
    e2e_runtimes_w_sensor = []
    sensor_send_runtimes = []
    for row in csv_reader:
        sim_time = int(row[1])
        event_name = row[2]
        if event_name == 'e2e_runtime':
            e2e_runtime = float(row[3])
            e2e_runtimes_w_sensor.append(e2e_runtime)
            e2e_runtimes.append(e2e_runtime - sensor_send_runtime[sim_time])
            sim_times.append(sim_time)
        elif event_name == 'sensor_send_runtime':
            sensor_send_runtime[sim_time] = float(row[3])
            sensor_send_runtimes.append(float(row[3]))

    return sim_times, e2e_runtimes, e2e_runtimes_w_sensor, sensor_send_runtimes


def read_challenge_collision_times(csv_file_path):
    csv_file = open(csv_file_path)
    csv_reader = csv.reader(csv_file)
    collisions_times = []
    prev_sim_time = 0
    prev_col_time = 0
    for row in csv_reader:
        sim_time = int(row[1])
        event_name = row[2]
        if event_name == 'collision':
            # TODO(ionel): Differentiate between the types of collisions.
            if prev_sim_time - prev_col_time > 300:
                # Ignore the repeatead collisions.
                collisions_times.append(prev_sim_time)
            prev_col_time = prev_sim_time
        else:
            prev_sim_time = sim_time
    return collisions_times


def print_collisions_with_outlier_runtimes(csv_file,
                                           sim_times,
                                           run_e2e,
                                           runtime_threshold=220):
    collision_times = read_challenge_collision_times(csv_file)
    for collision_time in collision_times:
        index = sim_times.index(collision_time)
        print("Collision at {}".format(sim_times[index]))
        for i in range(0, 21):
            if run_e2e[index - i] > runtime_threshold:
                print("Runtime {} at {}".format(run_e2e[index - i],
                                                sim_times[index - i]))


def read_challenge_stats(results_path, filter_carla_cola=False):
    with open(results_path) as f:
        data = json.load(f)
        score = float(
            data["_checkpoint"]["global_record"]["scores"]["score_composed"])
        num_col_vec = len(data["_checkpoint"]["records"][0]["infractions"]
                          ["collisions_vehicle"])
        cols_vec = data["_checkpoint"]["records"][0]["infractions"][
            "collisions_vehicle"]
        if filter_carla_cola:
            num_col_vec = 0
            for col in cols_vec:
                if 'carlacola' in col:
                    continue
                num_col_vec += 1
        else:
            num_col_vec = len(cols_vec)
        num_col_ped = len(data["_checkpoint"]["records"][0]["infractions"]
                          ["collisions_pedestrian"])

        # Collisions / km
        collision_ped = float(data["values"][3])
        collision_veh = float(data["values"][4])
        collision_lay = float(data["values"][5])
        # In meters.
        route_length = float(
            data["_checkpoint"]["records"][0]["meta"]["route_length"])
    return score, collision_ped, collision_veh, collision_lay, route_length, num_col_ped, num_col_vec


def read_challenge_deadline_misses(log_file):
    detection_miss = set()
    tracker_miss = set()
    loc_finder_miss = set()
    prediction_miss = set()
    planning_miss = set()
    with open(log_file) as f:
        for line in f:
            if 'deadline miss' in line:
                items = line.split(' ')
                op_name = items[1]
                sim_time = int(items[3][2:-2])
                if op_name == 'tracker_sort':
                    detection_miss.add(sim_time)
                elif op_name == 'center_camera_location_finder_history_operator':
                    tracker_miss.add(sim_time)
                elif op_name == 'linear_prediction_operator':
                    loc_finder_miss.add(sim_time)
                elif op_name == 'planning_operator':
                    prediction_miss.add(sim_time)
                elif op_name == 'pid_control_operator':
                    planning_miss.add(sim_time)
                else:
                    raise ValueError(
                        'Unexpected type of deadline miss: {}'.format(op_name))
    return (detection_miss, tracker_miss, loc_finder_miss, prediction_miss,
            planning_miss)


def read_challenge_results(log_dir_base,
                           town,
                           route,
                           detector,
                           num_reps,
                           segmentation_name,
                           segmentation_value,
                           filter_carla_cola=False):
    scores = []
    route_len = 0
    collisions_ped = []
    collisions_veh = []
    collisions_lay = []
    num_vec_collisions = []
    num_ped_collisions = []
    e2e_runtimes = []
    e2e_runtimes_w_sensor = []
    detector_runtimes = []
    loc_finder_runtimes = []
    tracker_runtimes = []
    prediction_runtimes = []
    planning_runtimes = []
    for run in range(1, num_reps + 1):
        log_dir = '{}_run_{}/'.format(log_dir_base, run)
        result_file = log_dir + 'results.json'
        csv_file = log_dir + 'challenge.csv'
        profile_file = log_dir + 'challenge.json'
        log_file = log_dir + 'challenge.log'
        # Get the runtimes
        fix_pylot_profile(profile_file)
        profile_events = ProfileEvents(profile_file, no_offset=True)
        # profile_events.check_if_timestamps_overlapped()

        # Get the end-to-end runtimes.
        (sim_times, run_e2e, run_e2e_w_sensor,
         _) = read_challenge_runtimes(csv_file)
        # print_collisions_with_outlier_runtimes(csv_file, sim_times, run_e2e)
        e2e_runtimes = e2e_runtimes + run_e2e
        e2e_runtimes_w_sensor = e2e_runtimes_w_sensor + run_e2e_w_sensor

        detection_miss = tracker_miss = loc_finder_miss = prediction_miss = planning_miss = None
        if segmentation_name == 'deadline' and segmentation_value:
            (detection_miss, tracker_miss, loc_finder_miss, prediction_miss,
             planning_miss) = read_challenge_deadline_misses(log_file)
            # num_times = len(run_e2e)
            # print('Percentage detection deadline misses {:0.2f}'.format(
            #     len(detection_miss) / num_times))
            # print('Percentage tracker deadline misses {:0.2f}'.format(
            #     len(tracker_miss) / num_times))
            # print('Percentage loc_finder deadline misses {:0.2f}'.format(
            #     len(loc_finder_miss) / num_times))
            # print('Percentage prediction deadline misses {:0.2f}'.format(
            #     len(prediction_miss) / num_times))
            # print('Percentage planning deadline misses {:0.2f}'.format(
            #     len(planning_miss) / num_times))

        run_detector_runtimes = profile_events.get_filtered_runtimes(
            'efficientdet_operator.on_watermark',
            timestamps_to_ban=detection_miss)
        run_loc_finder_runtimes = profile_events.get_filtered_runtimes(
            'center_camera_location_finder_history_operator.on_watermark',
            timestamps_to_ban=loc_finder_miss)
        run_tracker_runtimes = profile_events.get_filtered_runtimes(
            'tracker_sort.on_watermark', timestamps_to_ban=tracker_miss)
        run_prediction_runtimes = profile_events.get_filtered_runtimes(
            'linear_prediction_operator.generate_predicted_trajectories',
            timestamps_to_ban=prediction_miss)
        if len(run_prediction_runtimes) == 0:
            run_prediction_runtimes = profile_events.get_filtered_runtimes(
                'linear_prediction_operator.on_watermark',
                timestamps_to_ban=prediction_miss)
        run_planning_runtimes = profile_events.get_filtered_runtimes(
            'planning_operator.on_watermark', timestamps_to_ban=planning_miss)
        detector_runtimes = detector_runtimes + run_detector_runtimes
        loc_finder_runtimes = loc_finder_runtimes + run_loc_finder_runtimes
        tracker_runtimes = tracker_runtimes + run_tracker_runtimes
        prediction_runtimes = prediction_runtimes + run_prediction_runtimes
        planning_runtimes = planning_runtimes + run_planning_runtimes

        # Get the scores.
        score, cp, cv, cl, m_driven, num_ped_col, num_vec_col = \
            read_challenge_stats(result_file, filter_carla_cola)
        scores.append(score)
        collisions_ped.append(cp)
        collisions_veh.append(cv)
        collisions_lay.append(cl)
        num_vec_collisions.append(num_vec_col)
        num_ped_collisions.append(num_ped_col)
        route_len += m_driven

    # Transform to km.
    route_len /= 1000

    entries = len(e2e_runtimes)
    runtimes_df = pd.DataFrame({
        'town': [town] * entries,
        'route': [route] * entries,
        'detector': [detector] * entries,
        segmentation_name: [segmentation_value] * entries,
        'e2e_runtime': e2e_runtimes,
        'e2e_runtime_w_sensor': e2e_runtimes_w_sensor,
        'detector_runtime': detector_runtimes,
        'tracker_runtime': tracker_runtimes,
        'loc_finder_runtime': loc_finder_runtimes,
        'prediction_runtime': prediction_runtimes,
        'planning_runtime': planning_runtimes,
    })

    score_df = pd.DataFrame({
        'town': [town] * len(scores),
        'route': [route] * len(scores),
        segmentation_name: [segmentation_value] * len(scores),
        'detector': [detector] * len(scores),
        'score':
        scores,
        'collisions_ped':
        collisions_ped,
        'collisions_veh':
        collisions_veh,
        'collisions_lay':
        collisions_lay,
        'num_vec_collisions':
        num_vec_collisions,
        'num_ped_collisions':
        num_ped_collisions
    })

    return runtimes_df, score_df, route_len
