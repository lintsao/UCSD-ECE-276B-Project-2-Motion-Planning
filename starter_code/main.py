import argparse
import Planner

from src.rrt.rrt import RRT
from src.rrt.rrt_star import RRTStar
from src.rrt.rrt_star_bid_h import RRTStarBidirectionalHeuristic
from src.rrt.rrt_star_bid import RRTStarBidirectional
from src.rrt.rrt_connect import RRTConnect

from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot

from utils import *

if __name__=="__main__":
  np.random.seed(seed=0)
  parser = argparse.ArgumentParser()
  parser.add_argument("planner_name", help=
                      "Choose a planner name: default, astar, rrt, rrt_star, rrt_star_bid, rrt_star_bid_h, rrt_connect")
  
  args = parser.parse_args()

  assert args.planner_name in ["default", "astar", "rrt", "rrt_star", "rrt_star_bid", "rrt_star_bid_h", "rrt_connect"]
  
  test_single_cube(args.planner_name)
  test_maze(args.planner_name) 
  test_flappy_bird(args.planner_name) # Astar: 0.5 fails, 0.1 ok
  test_monza(args.planner_name) # RRT fail
  test_window(args.planner_name)
  test_tower(args.planner_name)
  test_room(args.planner_name)
  
  plt.show(block=True)
  