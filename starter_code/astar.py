
# priority queue for OPEN list
from pqdict import pqdict
import math

class AStarNode(object):
  def __init__(self, pqkey, coord, hval):
    self.pqkey = pqkey
    self.coord = coord
    self.g = math.inf
    self.h = hval
    self.parent_node = None
    self.parent_action = None
    self.closed = False
  def __lt__(self, other):
    return self.g < other.g     


class AStar(object):
  @staticmethod
  def plan(start_coord, environment, epsilon = 1):
    # Initialize the graph and open list
    Graph = {}
    OPEN = pqdict()
    
    # current node
    curr = AStarNode(tuple(start_coord), start_coord, environment.getHeuristic(start_coord))
    curr.g = 0
    
    # TODO: Implement A* here
    pass




