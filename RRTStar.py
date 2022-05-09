#!/usr/bin/env python3
import numpy as np
from RRTPlot import *
from RRTStarFunctions import *

# This function is the RRT* - Smart function 
def RRTStarSmart(start_point,goal,obstacles,K,epsilon):
  
    Edges = []
    best_path = []
    
    
    Vertices= [start_point] 
    T = [start_point.coordinates]
    reached_goal = False
    
    for node_count in range(K):
        
        # random sampling of point from free space
        z_random = Node(np.random.uniform(0,10), np.random.uniform(0,10))
        
        # finding the nearest node to random-node generated
        z_nearest = Nearest(z_random,T,Vertices)
        
        new_path = generatePath(z_random,z_nearest,epsilon,split_line=True)
        
        # Check if there is a collision
        if (not collisionWithRectangle(new_path,obstacles)):
            
            # adding the new node to Veticeslist and Tree
            z_new =new_path[-1]
            Vertices.append(z_new)
            T.append(z_new.coordinates)
            
            # assigning parent and cost to new node generated
            z_new.parent = z_nearest
            z_new.cost = z_nearest.cost + line(z_nearest,z_new)
            
            # finding the nearest neighbours
            z_near = Near(Vertices,z_new,T,len(Vertices))
            
            # Choose a parent from the nearest neighbours
            z_nearest = selectParent(z_near,z_nearest,z_new,epsilon,obstacles)
            
            z_new.parent = z_nearest
            z_new.cost = z_nearest.cost + line(z_nearest,z_new)
            Edges.append((z_nearest,z_new))
            
            # rewire the suurounding nodes
            Edges = rewire(z_near,Edges,z_new,epsilon,obstacles)
            
            # These if statements are used for the goal area and cost plotting
            if collisionWithRectangle([z_new],goal) and not reached_goal:
                reached_goal = True
            elif reached_goal:
                cost_to_path = leastCost(Vertices,goal)[1]
                cost_accumulations.append(cost_to_path)
            else: cost_accumulations.append(1000)
        else: cost_accumulations.append(cost_accumulations[-1])
        
    return (Vertices,Edges,best_path)

# This function is defined to generate the tree initially with RRT* and then optimizing 
# it with RRT* - Smart algorithm to get better results

def generateRRTStarTree(epsilon,K,obstacles,start_point,goal):
    
    """ arguments:  k= maximum no of nodes to generate,epsilon= step-size,
                    obstacles,start_point,goal_point"""
    
    print("******* ___ Generating RRT* ___********\n")
    RRTStar_output = RRTStarSmart(start_point,goal,obstacles,K,epsilon)
    path_to_goal, cost = leastCost(RRTStar_output[0],goal)
    
    RRTStar_output = (RRTStar_output[0],RRTStar_output[1],path_to_goal)
    print("Number of Nodes Explored : ", len(RRTStar_output[0])) 
    if cost == 1000:
        print("\nNo path exists for given resolution of samples")
    else:
        print("\nCost to reach the goal: ", cost)
    
    print("\n*******___ Solution found using RRT* ___*******")
    print("\n*******___ Starting RRT* - Smart ___******* ")
    
    # Plotting the RRT* Tree and obstacles 
    plotTree(RRTStar_output,goal,obstacles,start_state)

# Initiating different obstacle spaces

# ////////////////////////////////////////////////////////////

# Obstacle space 1

rect = [((1.5,5.5),6.5,1),((7,1),1.25,8),((8.25,2),1,1)]

# initializing start-point and goal-point

start_point = Node(4,5)

start_state=[((4,5),0.3,0.3)]

goal_point = [((8.25,9.2),0.4,0.4)]

epsilon=0.15
# Number of nodes to explore
K = 5000

# ////////////////////////////////////////////////////////////////////////

# =============================================================================
# Obstacle space 2

# rect = [((2,3),2,1),((2,4),0.5,3),((2,7),6.5,1),((8,4),0.5,3),((6.5,3),2,1)]

# start_point = Node(5,5)

# start_state=[((5,5),0.3,0.3)]

# goal_point = [((5,9),0.3,0.3)]

# epsilon=0.15

# K = 5000
# =============================================================================

# ////////////////////////////////////////////////////////////////////////////////

# =============================================================================
# Obstacle space 3

# rect = [((4.85,1),2.5,0.5),((1.5,2.5),3.5,1),((4.85,4.5),4,1),((1.5,6.5),3.5,1),((4.85,8.5),2.5,0.5)]

# start_point = Node(5,0.5)

# start_state=[((5,9.5),0.2,0.2)]

# goal_point = [((5,9.5),0.6,0.6)]

# epsilon=0.15

# K = 13000
# =============================================================================
obstacles=rect
cost_accumulations = [1000]

# function to generate nodes

generateRRTStarTree(epsilon,K,obstacles,start_point,goal_point)
costs = np.asarray(cost_accumulations)
plot_cost(costs,K)
# plot the cost of the best path found

