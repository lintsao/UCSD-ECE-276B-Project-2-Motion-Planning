# UCSD ECE 276B Project 2: Motion Planning

This project will focus on comparing the performance of search-based and sampling-based motion planning algorithms in 3-D Euclidean space. You are provided with a set of 3-D environments described by a rectangular outer boundary and a set of rectangular obstacle blocks. Each rectangle is described by a 9-dimensional vector, specifying its lower left corner (xmin, ymin, zmin), its upper right corner (xmax, ymax, zmax), and its RGB color (for visualization). The start xs ∈ R3 and goal xτ ∈ R 3
coordinates are also specified for each of the available environments. The provided sample code includes a baseline planner which moves greedily toward the goal. This planner gets stuck in complex environments and is not very careful with collision checking

## Installation
To get started with the object detection project, follow these steps:

1. Clone this repository:
  ```bash
  git clone https://github.com/homerun-beauty/ECE-276B-PR2.git
  cd ECE-276B-PR2
  ```

2. Create a new virtual environment:
  ```bash
  python3 -m venv env
  source env/bin/activate  # For Unix/Linux
  ```

3. Install the required dependencies:
  ```bash
  pip3 install -r requirements.txt
  ```

4. You're ready to use the motion planning project!

## Usage

```
cd starter_code
python3 main.py <planner-name>
```
Replace <planner-name> with the following options:
- A* (Astar): Informed search algorithm with heuristic for optimal path planning.
- RRT (Rapidly-Exploring Random Tree): Randomized sampling-based motion planning algorithm.
- RRT* (Rapidly-Exploring Random Tree Star): Optimized version of RRT with improved path quality.
- Bidirectional RRT*: RRT* variant with exploration from both start and goal states.
- Bidirectional RRT* (with heuristic): Heuristic-guided variant of bidirectional RRT* algorithm.
- RRT-Connect: Connecting trees variant of RRT for motion planning.

## Contributing
Contributions are welcome! If you have any suggestions, bug reports, or feature requests, please open an issue or submit a pull request.

## License
This project is licensed under the MIT License. You are free to use, modify, and distribute this software for personal or commercial purposes.
