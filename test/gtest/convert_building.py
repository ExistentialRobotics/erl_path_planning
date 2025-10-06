import yaml


with open("building.yaml", "r") as file:
    data = yaml.load(file, Loader=yaml.SafeLoader)

for floor_id, floor in data["floors"].items():
    for room_id, room in floor["rooms"].items():
        for door_id, door in room["door_grids"].items():
            # [N, 2] to [2, N]
            room["door_grids"][door_id] = [[d[0] for d in door], [d[1] for d in door]]


def list_representer(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)


yaml.add_representer(list, list_representer)

with open("building.yaml", "w") as file:
    yaml.dump(data, file)
