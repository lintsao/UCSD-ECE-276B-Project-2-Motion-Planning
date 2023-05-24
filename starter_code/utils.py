import numpy as np
import time
import sys
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import Planner

from src.rrt.rrt import RRT
from src.rrt.rrt_star import RRTStar
from src.rrt.rrt_star_bid_h import RRTStarBidirectionalHeuristic
from src.rrt.rrt_star_bid import RRTStarBidirectional
from src.rrt.rrt_connect import RRTConnect

from src.search_space.search_space import SearchSpace

def tic():
  return time.time()


def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))


def load_map(fname):
  '''
  Loads the bounady and blocks from map file fname.
  
  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  
  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  '''
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  return boundary, blocks


def draw_map(boundary, blocks, start, goal):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  # ax = Axes3D(fig)
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5])
  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h
  
def check_collision(line_segment, aabb):
  if(
    line_segment['start_x'] > aabb['min_x'] and line_segment['start_x'] < aabb['max_x'] and
    line_segment['start_y'] > aabb['min_y'] and line_segment['start_y'] < aabb['max_y'] and
    line_segment['start_z'] > aabb['min_z'] and line_segment['start_z'] < aabb['max_z']
    ):
    print("Coliision happened: point inside the block.")
    return True
    
  # Compute the direction vector of the line segment
  direction = {
    'x': line_segment['end_x'] - line_segment['start_x'],
    'y': line_segment['end_y'] - line_segment['start_y'],
    'z': line_segment['end_z'] - line_segment['start_z']
  }
    
  for dim in ['x', 'y', 'z']:
    # print(f"Check dim: {dim}")
    t = 0.0
    if direction[dim] < sys.float_info.epsilon:
      # print(f"Dim {dim} == 0")
      continue
      
    if direction[dim] > 0:
      t = (aabb['min_' + dim] - line_segment['start_' + dim]) / direction[dim]
    else:
      t = (aabb['max_' + dim] - line_segment['start_' + dim]) / direction[dim]

    if (t > 0 and t < 1):
      intersect_point = {
                        'x': line_segment['start_x'] + t * direction['x'], 
                        'y': line_segment['start_y'] + t * direction['y'],
                        'z': line_segment['start_z'] + t * direction['z']
                        }
        
      next1 = chr(ord('x') + (ord(dim) + 1 - ord('x')) % 3)
      next2 = chr(ord('x') + (ord(dim) + 2 - ord('x')) % 3)
      
      if (
        aabb['min_' + next1] < intersect_point[next1] and intersect_point[next1] < aabb['max_' + next1] and
        aabb['min_' + next2] < intersect_point[next2] and intersect_point[next2] < aabb['max_' + next2]
        ):
        print(dim)
        print(next1, aabb['min_' + next1], intersect_point[next1], aabb['max_' + next1])
        print(next2, aabb['min_' + next2], intersect_point[next2], aabb['max_' + next2])
        print("Coliision happened: path hits the block.")
          
        return True

    return False
  
def runtest(planner_name, mapfile, start, goal, verbose = True):
  '''
  This function:
   * loads the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)
  # MP = Planner.MyPlanner(boundary, blocks) # TODO: replace this with your own planner implementation

  print(f"Use {planner_name}")

  if (planner_name == 'default'):
    MP = Planner.MyPlanner(boundary, blocks) # TODO: replace this with your own planner implementation
  elif (planner_name == 'astar'):
    MP = Planner.AStarPlanner(boundary, blocks, resolution=0.1, eps=1.0)
  else:
    dim = np.array([(boundary[0][0], boundary[0][3]), 
                    (boundary[0][1], boundary[0][4]), 
                    (boundary[0][2], boundary[0][5])])
    # create Search Space
    X = SearchSpace(dim, np.array(blocks)[:, :6])

    if (planner_name == 'rrt'):
      Q = np.array([(1, 1)])  # length of tree edges
      r = 0.05  # length of smallest edge to check for intersection with obstacles
      max_samples = 10000000  # max number of samples to take before timing out
      prc = 0.1  # probability of checking for a connection to goal

      # create rrt_search
      MP = RRT(X, Q, tuple(start), tuple(goal), max_samples, r, prc)

    elif (planner_name == 'rrt_star'):
      Q = np.array([(1, 1)])  # length of tree edges
      r = 0.05  # length of smallest edge to check for intersection with obstacles
      max_samples = 10000000  # max number of samples to take before timing out
      rewire_count = 32  # optional, number of nearby branches to rewire
      prc = 0.1  # probability of checking for a connection to goal

      # create rrt_search
      MP = RRTStar(X, Q, tuple(start), tuple(goal), max_samples, r, prc, rewire_count)

    elif (planner_name == 'rrt_star_bid'):
      Q = np.array([(1, 1)])  # length of tree edges
      r = 0.01  # length of smallest edge to check for intersection with obstacles
      max_samples = 10000000  # max number of samples to take before timing out
      rewire_count = 32  # optional, number of nearby branches to rewire
      prc = 0.01  # probability of checking for a connection to goal

      # create rrt_search
      MP = RRTStarBidirectional(X, Q, tuple(start), tuple(goal), max_samples, r, prc, rewire_count)

    elif (planner_name == 'rrt_star_bid_h'):
      Q = np.array([(1, 1)])  # length of tree edges
      r = 0.05  # length of smallest edge to check for intersection with obstacles
      max_samples = 10000000  # max number of samples to take before timing out
      rewire_count = 32  # optional, number of nearby branches to rewire
      prc = 0.01  # probability of checking for a connection to goal

      # create rrt_search
      MP = RRTStarBidirectionalHeuristic(X, Q, tuple(start), tuple(goal), max_samples, r, prc, rewire_count)

    elif (planner_name == 'rrt_connect'):
      Q = np.array([(1, 1)])  # length of tree edges
      r = 0.05  # length of smallest edge to check for intersection with obstacles
      max_samples = 10000000  # max number of samples to take before timing out
      prc = 0.1  # probability of checking for a connection to goal

      # create rrt_search
      MP = RRTConnect(X, Q, tuple(start), tuple(goal), max_samples, r, prc)

  # Display the environment
  if verbose:
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)

  # Call the motion planner
  t0 = tic()
  path_tmp = MP.plan(start, goal)

  toc(t0,"Planning")

  if (path_tmp != None):
    pass
  else:
    return False, 0, []
  
  try:
    if (len(path_tmp)):
      pass
  except Exception as err:
    return False, 0, []
  
  curr = 1
  cost = 0
  while (curr < len(path_tmp)):
    cost += ((path_tmp[curr][0] - path_tmp[curr - 1][0])**2 +
             (path_tmp[curr][1] - path_tmp[curr - 1][1])**2 + 
             (path_tmp[curr][2] - path_tmp[curr - 1][2])**2)**0.5
    curr += 1

  path = np.array([list(item) for item in path_tmp])
  
  # Plot the path
  if verbose:
    ax.plot(path[:,0],path[:,1],path[:,2],'r-')

  # TODO: You should verify whether the path actually intersects any of the obstacles in continuous space
  # TODO: You can implement your own algorithm or use an existing library for segment and 
  #       axis-aligned bounding box (AABB) intersection
  for block in blocks:
    block_dict = {'min_x': block[0], 'min_y': block[1], 'min_z': block[2], 
                  'max_x': block[3], 'max_y': block[4], 'max_z': block[5]}
    
    for i in range(1, len(path)):
      start_point = path[i - 1]
      end_point = path[i]
      line_segment = {'min_x': min(start_point[0], end_point[0]),
                      'min_y': min(start_point[1], end_point[1]), 
                      'min_z': min(start_point[2], end_point[2]), 
                      'max_x': max(start_point[0], end_point[0]), 
                      'max_y': max(start_point[1], end_point[1]), 
                      'max_z': max(start_point[2], end_point[2]),
                      'start_x': start_point[0], 'start_y': start_point[1], 'start_z': start_point[2],
                      'end_x': end_point[0], 'end_y': end_point[1], 'end_z': end_point[2]}

      collision = check_collision(line_segment, block_dict)

      if (collision):
        print(f"Collision at segement: {start_point} to {end_point}, block:", block)
        collision_seg = [[start_point[0], end_point[0]], [start_point[1], end_point[1]], [start_point[2], end_point[2]]]
        ax.plot(collision_seg [0], collision_seg[1], collision_seg[2], 'g-')
        break

    if (collision):
      break
    
  goal_reached = sum((path[-1]-goal)**2) <= 0.1
  print(f"goal_reached: {goal_reached}, collision: {collision}")
  print(f"cost: {cost}")
  success = (not collision) and goal_reached
  pathlength = np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))
  return success, pathlength, path
  
'''
Test case.
'''

def test_single_cube(planner_name, verbose = True):
  print('Running single cube test...\n') 
  start = np.array([2.3, 2.3, 1.3])
  goal = np.array([7.0, 7.0, 5.5])
  success, pathlength, path = runtest(planner_name, './maps/single_cube.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')
  
  
def test_maze(planner_name, verbose = True):
  print('Running maze test...\n') 
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  success, pathlength, path = runtest(planner_name, './maps/maze.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

    
def test_window(planner_name, verbose = True):
  print('Running window test...\n') 
  start = np.array([0.2, -4.9, 0.2])
  goal = np.array([6.0, 18.0, 3.0])
  success, pathlength, path = runtest(planner_name, './maps/window.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

  
def test_tower(planner_name, verbose = True):
  print('Running tower test...\n') 
  start = np.array([2.5, 4.0, 0.5])
  goal = np.array([4.0, 2.5, 19.5])
  success, pathlength, path = runtest(planner_name, './maps/tower.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

     
def test_flappy_bird(planner_name, verbose = True):
  print('Running flappy bird test...\n') 
  start = np.array([0.5, 2.5, 5.5])
  goal = np.array([19.0, 2.5, 5.5])
  success, pathlength, path = runtest(planner_name, './maps/flappy_bird.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength) 
  print('\n')

  
def test_room(planner_name, verbose = True):
  print('Running room test...\n') 
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([9.0, 7.0, 1.5])
  success, pathlength, path = runtest(planner_name, './maps/room.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


def test_monza(planner_name, verbose = True):
  print('Running monza test...\n')
  start = np.array([0.5, 1.0, 4.9])
  goal = np.array([3.8, 1.0, 0.1])
  success, pathlength, path = runtest(planner_name, './maps/monza.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')  