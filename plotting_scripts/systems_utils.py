import brewer2mpl
import pandas as pd
import re

bmap = brewer2mpl.get_map("Set2", "qualitative", 7)
colors = bmap.mpl_colors


class RegexDict(dict):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, key):
        for pattern in dict.keys(self):
            if re.match(pattern, key):
                return dict.get(self, pattern)
        if self.__contains__(key):
            return dict.get(self, key)
        raise KeyError("Could not match {}".format(key))

    def __contains__(self, key):
        for pattern in dict.keys(self):
            if re.match(pattern, key):
                return True
        return False


COLOR_DICT = RegexDict(
    {
        # "CarFlow \([Ii]ntra": colors[4],
        # "CarFlow \([Ii]nter": colors[0],
        "CarFlow": colors[4],
        # "(ERDOS|erdos) \([Ii]ntra": colors[4],
        # "(ERDOS|erdos) \([Ii]nter": colors[0],
        "ERDOS": colors[4],
        "ROS2": (1, 0.6941176470588235, 0.6039215686274509),
        "ROS": colors[1],
        # "Flink \([Ii]ntra": colors[3],
        # "Flink \([Ii]nter": colors[2],
        "Flink": colors[3],
        # Catch-all
        "[Ii]ntra": colors[4],
        "[Ii]nter": colors[0],
    }
)


def get_colors(systems):
    return [COLOR_DICT[system] for system in systems]


HATCH_DICT = RegexDict(
    {
        # "CarFlow \([Ii]ntra": "",
        # "CarFlow \([Ii]nter": "\\\\\\",
        # "(ERDOS|erdos) \([Ii]ntra": "",
        # "(ERDOS|erdos) \([Ii]nter": "\\\\\\",
        "CarFlow": "",
        "ERDOS": "",
        "ROS2": "*",
        "ROS": "xxx",
        # "Flink \([Ii]ntra": "...",
        # "Flink \([Ii]nter": "///",
        "Flink": "....",
        # Catch-all
        "[Ii]ntra": "",
        "[Ii]nter": "\\\\\\",
    }
)


def msg_size_to_str(x):
    if x / 1e6 >= 1:
        return f"{x // 1000000} MB"
    elif x / 1e3 >= 1:
        return f"{x // 1000} KB"
    else:
        return f"{x} B"


def load_csv(name, filename):
    df = pd.read_csv(filename)
    df["name"] = name
    df["latency_ms"] = df.latency_secs * 1e3
    return df


def load_csvs(d):
    return pd.concat([load_csv(name, filename) for name, filename in d.items()])


def bold(s):
    return r"\textbf{" + s + r"}"
