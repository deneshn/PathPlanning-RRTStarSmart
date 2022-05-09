import numpy as np
from scipy import spatial

# Node class stores information of costtocome, parent, and co-ordinates of each node
class Node:
    cost = 0
    parent = None
    coordinates = [0,0]
    x, y = 0,0
    def __init__(self,xc,yc,cost=0,parent=None):
        self.coordinates = np.asarray([xc,yc])
        self.x = xc
        self.y = yc
        self.cost = cost
        self.parent = parent
        
# KD-tree for efficient nearest neighbour search in O(log(n))
def Nearest(node,edges_coordinates,node_list):
    tree = spatial.KDTree(np.asarray(edges_coordinates))
    random_point = node.coordinates
    nearest_node_indices_list = tree.query(random_point)
    nearest_node_index=nearest_node_indices_list[1]
    nearest_node=node_list[nearest_node_index]
    
    return nearest_node


def line(z_near, z_new):
    return np.linalg.norm(z_near.coordinates - z_new.coordinates)
        
def generatePath(z_random, z_near, step_size,split_line=False):
    if line(z_random,z_near) <= step_size:
        return [z_random]
    
    # get the sine and cosine components in to parametric_angle
    sides= [z_random.x- z_near.x, z_random.y - z_near.y]
  
    parametric_angle = sides / np.linalg.norm(sides)
    
    divisions = 30
    
    split_path = []
   
    
    if split_line:
        # generate dotted path
        for i in range(divisions):
            next_point = z_near.coordinates + (step_size / divisions) * parametric_angle * i
            point=Node(next_point[0],next_point[1])
            split_path.append(point)
    else: 
        # generate  direct-line between nodes
        new_point = z_near.coordinates + step_size *parametric_angle
        point1=Node(new_point[0],new_point[1])
        return [point1]
    return split_path

def Near(V,z_new,node_coordinates,length_V):
    eta = 0.4 # maz radius
    gamma = 50
    zheta = gamma * (np.log(length_V) / length_V) # surface of the ball  
    t = np.power((gamma / zheta) * (np.log(length_V) / length_V),1/2)
    r = min(t,eta)
    near_set = kdTreeRadius(z_new,V,node_coordinates,r)

    return near_set


# The kd-tree giving a set of nodes nearest to a vertex
def kdTreeRadius(node,node_list,edges_coordinates,ball_radius):
    tree = spatial.KDTree(np.asarray(edges_coordinates))
    random_point = node.coordinates
    nearest_neighbours_indices = tree.query_ball_point(random_point,ball_radius)
    
    # return the list with nearest neighbours given a ball radius
    
    nearest_neighbours=[node_list[i] for i in nearest_neighbours_indices]
    return nearest_neighbours


def selectParent(z_near,z_nearest,z_new,step_size,obstacle):
    for z_near in z_near:
        # if obstacle free
        c_prime = z_near.cost + line(z_near,z_new)
        path_new = generatePath(z_new,z_near,step_size,split_line=True)
        if (not collisionWithRectangle(path_new,obstacle)):
            if c_prime < z_new.cost:
                z_new.cost = c_prime
                z_nearest = z_near
    return z_nearest

def collisionWithRectangle(potential_points, obstacles):
    
    for ((rectXCordinate, rectYCordinate), width, height) in obstacles:
        for point in potential_points:
            x, y = point.x, point.y
            if (x <= rectXCordinate + width and x >= rectXCordinate) and (y <= rectYCordinate + height and y >= rectYCordinate):
                    return True
    return False

def collisionWithRect(point_list, obstacles):
    
    for ((rectXCordinate, rectYCordinate), width, height) in obstacles:
        
            x, y = point_list[0],point_list[1]
            if (x <= rectXCordinate + width and x >= rectXCordinate) and (y <= rectYCordinate + height and y >= rectYCordinate):
                    return True
    return False

# Get the path given a starting point in the tree
def tracePath(child_node,path):
    if child_node.cost == 0:
        path.append((child_node,child_node))
        return path
    else:
        parent = child_node.parent
        path.append((parent,child_node))
        return tracePath(child_node.parent,path)

# compute the cost of a path
def costofPath(path):
    cost = 0
    for edge in path:
        cost += line(edge[0],edge[1])
    return cost


# Search for the least cost path maintained by the tree
def leastCost(V,goal_area):
  path_to_goal = []
  best_path = []
  cost = 1000
  for node in V:
      if collisionWithRectangle([node],goal_area):
          path_to_goal = tracePath(node,[])
          goal_cost = costofPath(path_to_goal)
          if goal_cost <= cost:
              cost = goal_cost
              best_path = path_to_goal
  return (best_path,cost) 

def rewire(z_near,E,z_new,step_size,obstacle):
    for z_near in z_near:
        path_new = generatePath(z_new,z_near,step_size,split_line=True)
        if (not collisionWithRectangle(path_new,obstacle)):
            if z_near.cost > z_new.cost + line(z_near,z_new):
                z_parent = z_near.parent
                z_near.parent = z_new
                z_near.cost = z_new.cost + line(z_near,z_new)
                E.remove((z_parent,z_near))
                E.append((z_new,z_near))
    return E

 