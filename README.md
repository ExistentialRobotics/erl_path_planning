# `erl_path_planning`

[![Tags](https://img.shields.io/github/v/tag/ExistentialRobotics/erl_path_planning?label=version)](https://github.com/ExistentialRobotics/erl_path_planning/tags)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS1](https://img.shields.io/badge/ROS1-noetic-blue)](http://wiki.ros.org/)
[![ROS2](https://img.shields.io/badge/ROS2-humble-blue)](https://docs.ros.org/)
[![ROS2](https://img.shields.io/badge/ROS2-jazzy-blue)](https://docs.ros.org/)

**`erl_path_planning` is a C++ library providing a collection of path planning algorithms for robotics applications.**

## Features

- **Multiple Planning Algorithms**: A*, AMRA*, Hybrid A*, and more
- **LTL Mission Planning**: Support for Linear Temporal Logic specifications in 2D and 3D
- **Flexible Heuristics**: Customizable heuristic functions including LLM-based scene graph heuristics
- **Kinodynamic Planning**: Dubins paths and Reeds-Shepp curves for car-like robots
- **Multi-Resolution Search**: AMRA* with anytime multi-resolution capabilities
- **Python Bindings**: Full Python API for all major algorithms

## Available Algorithms

### Search-Based Planners
- [**A***](include/erl_path_planning/astar.hpp): Classic optimal graph search algorithm
- [**AMRA***](docs/amra_star.md): Anytime Multi-Resolution Multi-Heuristic A* - efficient anytime planning with multiple heuristics and resolutions
- [**Hybrid A***](include/erl_path_planning/hybrid_astar.hpp): Grid-based planner with continuous state space for non-holonomic vehicles

### Kinodynamic Planning
- [**Dubins Path**](include/erl_path_planning/dubins_path.hpp): Shortest path for car-like robots with minimum turning radius (forward only)
- [**Reeds-Shepp Path**](include/erl_path_planning/reeds_shepp_path.hpp): Shortest path for car-like robots with minimum turning radius (forward and backward)

### Heuristics
- [**Heuristic Base**](include/erl_path_planning/heuristic.hpp): Base class for custom heuristics
- [**LTL 2D Heuristic**](include/erl_path_planning/ltl_2d_heuristic.hpp): Heuristic for LTL-based planning in 2D environments
- [**LTL 3D Heuristic**](include/erl_path_planning/ltl_3d_heuristic.hpp): Heuristic for LTL-based planning in 3D environments
- [**LLM Scene Graph Heuristic**](include/erl_path_planning/llm_scene_graph_heuristic.hpp): AI-powered heuristic using large language models and scene understanding

### Utilities
- [**Planning Output**](include/erl_path_planning/planning_output.hpp): Data structures for path and planning results
- [**Search Planning Interface**](include/erl_path_planning/search_planning_interface.hpp): Common interface for search-based planners

## Getting Started

### Prerequisites

- CMake 3.24 or higher
- A C++17 compatible compiler
- Python 3.8+ (for Python bindings)

### Create Workspace

```bash
mkdir -p <your_workspace>/src && \
vcs import --input https://raw.githubusercontent.com/ExistentialRobotics/erl_path_planning/refs/heads/main/erl_path_planning.repos <your_workspace>/src
```

### Dependencies

- [erl_cmake_tools](https://github.com/ExistentialRobotics/erl_cmake_tools)
- [erl_common](https://github.com/ExistentialRobotics/erl_common)
- [erl_env](https://github.com/ExistentialRobotics/erl_env)
- [erl_geometry](https://github.com/ExistentialRobotics/erl_geometry)
- [Spot](https://spot.lre.epita.fr/) (optional, for LTL planning)

#### Install Dependencies

```bash
# Ubuntu 20.04
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_common/refs/heads/main/scripts/setup_ubuntu_20.04.bash | bash
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_geometry/refs/heads/main/scripts/setup_ubuntu_20.04.bash | bash
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_env/refs/heads/main/scripts/setup_ubuntu_20.04.bash | bash

# Ubuntu 22.04, 24.04
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_common/refs/heads/main/scripts/setup_ubuntu_22.04_24.04.bash | bash
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_geometry/refs/heads/main/scripts/setup_ubuntu_22.04_24.04.bash | bash
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_env/refs/heads/main/scripts/setup_ubuntu_22.04_24.04.bash | bash

# Arch Linux (for Spot LTL support)
paru -S spot
```

### Docker Option

The easiest way to get started is to use the provided [Docker files](./docker), which contains all dependencies.

### Use as a Standard CMake Package

```bash
cd <your_workspace>
touch CMakeLists.txt
```

Add the following lines to your `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.24)
project(<your_project_name>)
add_subdirectory(src/erl_cmake_tools)
add_subdirectory(src/erl_common)
add_subdirectory(src/erl_covariance)
add_subdirectory(src/erl_geometry)
add_subdirectory(src/erl_env)
add_subdirectory(src/erl_path_planning)
```

Then run the following commands:

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
```

### Use as a ROS Package

```bash
cd <your_workspace>
source /opt/ros/<distro>/setup.bash
# for ROS1
catkin build erl_path_planning
source devel/setup.bash
# for ROS2
colcon build --packages-up-to erl_path_planning
source install/setup.bash
```

See also 🚪[erl_path_planning_ros](https://github.com/ExistentialRobotics/erl_path_planning_ros) for ROS nodes and integration.

### Install As Python Package

- Make sure you have installed all dependencies.
- Make sure you have the correct Python environment activated, `pipenv` is recommended.

```bash
cd <your_workspace>
for package in erl_cmake_tools erl_common erl_covariance erl_geometry erl_env erl_path_planning; do
    cd src/$package
    pip install . --verbose --no-build-isolation
    cd ../..
done
```

## Gallery

- [AMRA*](docs/amra_star.md): Detailed documentation on AMRA* algorithm with visualization examples and benchmarks

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Citation

If you use this library in your research, please cite:

```bibtex
@software{erl_path_planning,
  author = {Dai, Zhirui and Existential Robotics Lab},
  title = {erl_path_planning: A C++ Path Planning Library},
  url = {https://github.com/ExistentialRobotics/erl_path_planning},
  year = {2025}
}
```

For AMRA* specifically:
```bibtex
@inproceedings{srivastava2021amra,
  title={AMRA*: Anytime Multi-Resolution Multi-Heuristic A*},
  author={Srivastava, Dhruv and Likhachev, Maxim},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2021}
}
```

## Acknowledgments

- AMRA* implementation based on the [original work](https://github.com/dhruvms/amra) by Dhruv Srivastava
- Dubins and Reeds-Shepp path implementations inspired by classical motion planning literature

## Related Projects

- [erl_path_planning_ros](https://github.com/ExistentialRobotics/erl_path_planning_ros): ROS1/ROS2 integration for this library
- [erl_geometry](https://github.com/ExistentialRobotics/erl_geometry): Geometry processing library
- [erl_env](https://github.com/ExistentialRobotics/erl_env): Environment representation library
