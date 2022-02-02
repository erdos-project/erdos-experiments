import brewer2mpl
import re

bmap = brewer2mpl.get_map('Set2', 'qualitative', 8)
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


COLOR_DICT = RegexDict({
    "CarFlow \([Ii]ntra": colors[4],
    "CarFlow \([Ii]nter": colors[0],
    "CarFlow": colors[0],
    "Pylot": colors[0],
    "(ERDOS|erdos) \([Ii]ntra": colors[4],
    "(ERDOS|erdos) \([Ii]nter": colors[0],
    "ERDOS": colors[0],
    "ROS": colors[1],
    "Flink \([Ii]ntra": colors[3],
    "Flink \([Ii]nter": colors[2],
    "Flink": colors[2],
    # Catch-all
    "[Ii]ntra": colors[4],
    "[Ii]nter": colors[0],
    "No-Constraints": colors[7],
    "No-Policy": colors[6],
    "Deadlines": colors[5],
    "Policy": colors[4],
    "Ideal": 'blue'
})


def get_colors(systems):
    return [COLOR_DICT[system] for system in systems]


HATCH_DICT = RegexDict({
    "CarFlow \([Ii]ntra": "",
    "CarFlow \([Ii]nter": "\\\\\\",
    "(ERDOS|erdos) \([Ii]ntra": "",
    "(ERDOS|erdos) \([Ii]nter": "\\\\\\",
    "CarFlow": "\\\\\\",
    "Pylot": "\\\\\\",
    "ERDOS": "\\\\\\",
    "ROS": "xxx",
    "Flink \([Ii]ntra": "...",
    "Flink \([Ii]nter": "///",
    "Flink": "///",
    # Catch-all
    "[Ii]ntra": "",
    "[Ii]nter": "\\\\\\",
})
