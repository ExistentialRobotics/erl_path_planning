import re

import yaml


with open("llm_heuristic_old.yaml", "r") as file:
    data = yaml.load(file, Loader=yaml.SafeLoader)

data_converted = {}
pattern = re.compile(r"move\((\d+),\ (\d+)\)")
for room_id1, paths in data["llm_paths"].items():
    data_converted[room_id1] = {}
    for room_id2, path in paths.items():
        path_converted = []
        for wp in path:
            m = pattern.findall(wp)[0]
            path_converted.append(dict(type="kEnterRoom", uuid1=int(m[0]), uuid2=int(m[1])))
        data_converted[room_id1][room_id2] = path_converted

data["llm_paths"] = data_converted


def list_representer(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)


with open("llm_heuristic.yaml", "w") as file:
    # yaml.add_representer(list, list_representer)
    yaml.dump(data, file)
