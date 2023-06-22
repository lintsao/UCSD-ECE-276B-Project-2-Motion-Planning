# UCSD ECE 276B Project 2: Motion Planning

This project will focus on comparing the performance of search-based and sampling-based motion planning algorithms in 3-D Euclidean space. You are provided with a set of 3-D environments described by a rectangular outer boundary and a set of rectangular obstacle blocks. Each rectangle is described by a 9-dimensional vector, specifying its lower left corner (xmin, ymin, zmin), its upper right corner (xmax, ymax, zmax), and its RGB color (for visualization). The start xs ∈ R3 and goal xτ ∈ R3 coordinates are also specified for each of the available environments. The provided sample code includes a baseline planner which moves greedily toward the goal. This planner gets stuck in complex environments and is not very careful with collision checking.

<p align="center">
  <img src="https://github.com/homerun-beauty/ECE-276B-PR2/blob/main/starter_code/img/rrt*/rrt_star_maze_1.png?raw=true" alt="Project Image" width="600">
</p>
<p align="center">Here is a visual representation of our project in action. The agent is inside the maze environment. </p>

To get started with the motion planning project, follow these steps:

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
- ```astar``` A* (Astar): Informed search algorithm with heuristic for optimal path planning.
- ```rrt``` RRT (Rapidly-Exploring Random Tree): Randomized sampling-based motion planning algorithm.
- ```rrt_star``` RRT* (Rapidly-Exploring Random Tree Star): Optimized version of RRT with improved path quality.
- ```rrt_star_bid``` Bidirectional RRT*: RRT* variant with exploration from both start and goal states.
- ``` rrt_star_bid_h``` Bidirectional RRT* (with heuristic): Heuristic-guided variant of bidirectional RRT* algorithm.
- ```rrt_connect``` RRT-Connect: Connecting trees variant of RRT for motion planning.

## Contributing
Contributions are welcome! If you have any suggestions, bug reports, or feature requests, please open an issue or submit a pull request.

## License
 
The MIT License (MIT)

Copyright (c) 2015 Chris Kibble

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
