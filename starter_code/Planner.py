import numpy as np
import heapq

class MyPlanner:
  __slots__ = ['boundary', 'blocks']
  
  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks


  def plan(self,start,goal):
    path = [start]
    numofdirs = 26 # 3 * 3 * 3 - 1. (delete the same position.)
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1]) # Direction.
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    # print(dR)
    dR = np.delete(dR,13,axis=1)
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    
    for _ in range(2000):
      mindisttogoal = 1000000
      node = None
      for k in range(numofdirs):
        next = path[-1] + dR[:,k]
        
        # Check if this direction is valid
        if( next[0] < self.boundary[0,0] or next[0] > self.boundary[0,3] or \
            next[1] < self.boundary[0,1] or next[1] > self.boundary[0,4] or \
            next[2] < self.boundary[0,2] or next[2] > self.boundary[0,5] ):
          continue # Check the next step is inside the boundary.
        
        valid = True
        for k in range(self.blocks.shape[0]):
          if( next[0] >= self.blocks[k,0] and next[0] <= self.blocks[k,3] and\
              next[1] >= self.blocks[k,1] and next[1] <= self.blocks[k,4] and\
              next[2] >= self.blocks[k,2] and next[2] <= self.blocks[k,5] ):
            valid = False
            break # Check the next step is inside the block.
        if not valid:
          continue
        
        # Update next node
        disttogoal = sum((next - goal)**2)
        if( disttogoal < mindisttogoal):
          mindisttogoal = disttogoal
          node = next
      
      if node is None:
        break
      
      path.append(node)
      
      # Check if done
      if sum((path[-1]-goal)**2) <= 0.1:
        break
      
    return np.array(path)

class AStarPlanner:
  __slots__ = ['boundary', 'blocks', 'resolution', 
               'dp', 'map_index', 'open', 'closed', 
               'block_pos', 'start_tuple', 'goal_tuple',
               'eps']
  
  def __init__(self, boundary, blocks, resolution = 0.5, eps = 1.0):
    self.boundary = boundary
    self.blocks = blocks
    self.resolution = resolution
    self.dp = {}
    self.map_index = None
    self.open = []
    self.closed = set()
    self.block_pos = set()
    self.start_tuple = None
    self.goal_tuple = None
    self.eps = eps

    self.make_map()

  def make_map(self):
    size_x = int((self.boundary[0, 3] - self.boundary[0, 0]) / self.resolution)
    size_y = int((self.boundary[0, 4] - self.boundary[0, 1]) / self.resolution)
    size_z = int((self.boundary[0, 5] - self.boundary[0, 2]) / self.resolution)

    # print(size_x, size_y, size_z)

    [dX, dY, dZ] = np.meshgrid(np.arange(size_x + 1) , 
                               np.arange(size_y + 1) ,
                               np.arange(size_z + 1))

    self.map_index = np.vstack((dX.flatten(),
                                dY.flatten(),
                                dZ.flatten() ,
                                )) 
    
    for i in range(len(self.map_index[0])):
      self.dp[(self.map_index[0][i], self.map_index[1][i], self.map_index[2][i])] = [np.inf, None] # key: pos, val: (g_val, prev_pos)

    # Make obstacles in the map.
    for b in self.blocks:
      idx = np.where((self.map_index[0] * self.resolution + self.boundary[0, 0] >= b[0]) & 
                     (self.map_index[0] * self.resolution + self.boundary[0, 0] <= b[3]) &
                     (self.map_index[1] * self.resolution + self.boundary[0, 1] >= b[1]) & 
                     (self.map_index[1] * self.resolution + self.boundary[0, 1] <= b[4]) &
                     (self.map_index[2] * self.resolution + self.boundary[0, 2] >= b[2]) & 
                     (self.map_index[2] * self.resolution + self.boundary[0, 2] <= b[5])
                      )[0]
      
      for i in range(len(idx)):
        self.block_pos.add((self.map_index[0][idx[i]], self.map_index[1][idx[i]], self.map_index[2][idx[i]]))

  def octile_distance_3d(self, p1, p2):
      dx = abs(p1[0] - p2[0])
      dy = abs(p1[1] - p2[1])
      dz = abs(p1[2] - p2[2])
      return max(dx, dy, dz) + (np.sqrt(3) - 1) * min(min(dx, dy), dz)

  def plan(self, start, goal):
      # Find the grid which is the nearest to the start point and the goal point.
      for k, v in self.dp.items():
        if (k not in self.block_pos):
          dist = np.sqrt((k[0] * self.resolution + self.boundary[0, 0] - start[0])**2 + 
                         (k[1] * self.resolution + self.boundary[0, 1] - start[1])**2 + 
                         (k[2] * self.resolution + self.boundary[0, 2] - start[2])**2)
          
          if (self.start_tuple == None or
              dist < 
              (self.start_tuple[0] * self.resolution + self.boundary[0, 0] - start[0])**2 + 
              (self.start_tuple[1] * self.resolution + self.boundary[0, 1] - start[1])**2 + 
              (self.start_tuple[2] * self.resolution + self.boundary[0, 2] - start[2])**2 and
              dist <= self.resolution):
            self.start_tuple = k
          
          dist = np.sqrt((k[0] * self.resolution + self.boundary[0, 0] - goal[0])**2 + 
                         (k[1] * self.resolution + self.boundary[0, 1] - goal[1])**2 + 
                         (k[2] * self.resolution + self.boundary[0, 2] - goal[2])**2)
          
          if (self.goal_tuple == None or
              dist < 
              (self.goal_tuple[0] * self.resolution + self.boundary[0, 0] - goal[0])**2 + 
              (self.goal_tuple[1] * self.resolution + self.boundary[0, 1] - goal[1])**2 + 
              (self.goal_tuple[2] * self.resolution + self.boundary[0, 2] - goal[2])**2 and
              dist <= self.resolution):
            self.goal_tuple = k

      assert self.start_tuple != None and self.goal_tuple != None

      # Start planning.
      heapq.heappush(self.open, (0, 0, self.start_tuple)) # f, g, pos.
      self.dp[self.start_tuple] = [0, None]

      numofdirs = 26 # 3 * 3 * 3 - 1. (delete the same position.)
      [dX, dY, dZ] = np.meshgrid([-1,0,1],
                               [-1,0,1],
                               [-1,0,1]) # Direction.
      
      dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
      dR = np.delete(dR,13,axis=1)

      step = 0

      while (self.open):
          # mindisttogoal = 1000000
          f_i, g_i, node = heapq.heappop(self.open)
          self.closed.add(node)

          for k in range(numofdirs):
            next = (node[0] + dR[:, k][0], node[1] + dR[:, k][1], node[2] + dR[:, k][2])
            
            if (next in self.closed):
              continue

            # Check if this direction is valid
            if( next[0] * self.resolution + self.boundary[0, 0] < self.boundary[0,0] or 
                next[0] * self.resolution + self.boundary[0, 0] > self.boundary[0,3] or 
                next[1] * self.resolution + self.boundary[0, 1] < self.boundary[0,1] or 
                next[1] * self.resolution + self.boundary[0, 1] > self.boundary[0,4] or 
                next[2] * self.resolution + self.boundary[0, 2] < self.boundary[0,2] or 
                next[2] * self.resolution + self.boundary[0, 2] > self.boundary[0,5] ):
              continue # Check the next step is not inside the boundary.
            
            valid = True
            for k in range(self.blocks.shape[0]):
              if( next[0] * self.resolution + self.boundary[0, 0] >= self.blocks[k,0] and 
                  next[0] * self.resolution + self.boundary[0, 0] <= self.blocks[k,3] and
                  next[1] * self.resolution + self.boundary[0, 1] >= self.blocks[k,1] and 
                  next[1] * self.resolution + self.boundary[0, 1] <= self.blocks[k,4] and
                  next[2] * self.resolution + self.boundary[0, 2] >= self.blocks[k,2] and
                  next[2] * self.resolution + self.boundary[0, 2] <= self.blocks[k,5] or
                  next in self.block_pos):
                valid = False
                break # Check the next step is not inside the block.
            if not valid:
              continue
            
            c_ij = np.sqrt(dR[:, k][0] ** 2 + dR[:, k][1] ** 2 + dR[:, k][2] ** 2)
            h_i = self.octile_distance_3d(next, self.goal_tuple)
            f_i = g_i + self.eps * h_i

            if (self.dp[next][0] > c_ij + g_i):
              self.dp[next] = [c_ij + g_i, node]

              heapq.heappush(self.open, (f_i, self.dp[next][0], next)) # f, g, pos.

          step += 1

      path = [(self.goal_tuple[0] * self.resolution + self.boundary[0, 0], 
               self.goal_tuple[1] * self.resolution + self.boundary[0, 1], 
               self.goal_tuple[2] * self.resolution + self.boundary[0, 2])]
      
      prev = self.dp[self.goal_tuple][1]

      while (prev != None):
        curr = prev
        prev = self.dp[curr][1]
        if (prev):
          path.append((prev[0] * self.resolution + self.boundary[0, 0], 
                      prev[1] * self.resolution + self.boundary[0, 1], 
                      prev[2] * self.resolution + self.boundary[0, 2]))

      print(f"Total steps: {step}")
      assert curr == self.start_tuple
      path.reverse()

      return path
  