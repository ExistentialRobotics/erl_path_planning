import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from io import StringIO
import argparse


def main(filename: str):
    with open(filename, "r") as file:
        assert file.readline().strip() == "AMRA* solution", f"{filename} does not contain AMRA* solution"

        goal_index = int(file.readline().strip().split(":")[1].strip())
        cost = float(file.readline().strip().split(":")[1].strip())
        w1_solve = float(file.readline().strip().split(":")[1].strip())
        w2_solve = float(file.readline().strip().split(":")[1].strip())
        search_time = float(file.readline().strip().split(":")[1].strip())
        num_waypoints = int(file.readline().strip().split(":")[1].strip())

        assert file.readline().strip() == "path:", f"{filename} does not contain path"
        path = pd.read_csv(StringIO(''.join([file.readline() for _ in range(num_waypoints + 1)])), sep=",")

        assert file.readline().strip() == "action_coords:", f"{filename} does not contain action_coords"
        action_coords = pd.read_csv(
            StringIO(''.join([file.readline() for _ in range(num_waypoints - 1)])),
            sep=",",
            header=None,
        )

        print(path)
        print(action_coords)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize AMRA* solution')
    parser.add_argument('filename', type=str, help='filename of AMRA* solution')
    main(parser.parse_args().filename)
