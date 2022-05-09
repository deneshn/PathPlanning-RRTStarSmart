#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from sympy import symbols 
from RRTStarFunctions import *


# This function is for plotting the tree from the edges that have 
# nodes accumulated along with the goal path and the obstacles.

def plotTree(RRTStar,goal,obstacle_space,start_state):
    
    ax = plt.axes(xlim=(0,10),ylim=(0,10))
    Nodes, Edges, path = RRTStar
    
    # plot of tree
    for coordinate1,coordinate2 in Edges:
        x1, x2 = coordinate1.x, coordinate2.x
        y1, y2 = coordinate1.y, coordinate2.y
        ax.plot([x1,x2],[y1,y2],'-',color='orange',lw=0.5)
        ax.plot(x2,y2,'co',markersize=1)
    
    
    
    # start-state rectangle
    for start_coordinate, length, breadth in start_state:
        ax.add_patch(patches.Rectangle(start_coordinate, length, breadth,fill=True,color="blue"))
    
    # goal-state rectangle
    for start_coordinate, length, breadth in goal:
        ax.add_patch(patches.Rectangle(start_coordinate, length, breadth,fill=True,color="blue"))
   
    
    # obstacles
    for start_coordinate, length, breadth in obstacle_space:
        ax.add_patch(patches.Rectangle(start_coordinate,length,breadth,fill=True,color="darkgreen"))
        

    goal_path=[]
    # goal path
    for coordinate1,coordinate2 in path:
       
        x1, x2 = coordinate1.x, coordinate2.x
        y1, y2 = coordinate1.y, coordinate2.y
        goal_path.append([x1,y1])
        goal_path.append([x2,y2])
        # ax.plot([x1,x2],[y1,y2],'r-',lw=3)
   
    # ax.plot([x1,x2],[y1,y2],'r-',lw=0.8,label='RRT *')
    
    plotRRTStarSmart(goal_path,obstacle_space,ax)
    
             
    
    # for i in range(len(goal_path)-1):
    #    x1= goal_path[i][0]
    #    y1= goal_path[i][1]
    #    x2= goal_path[i+1][0]
    #    y2= goal_path[i+1][1]
    #    ax.plot([x1,x2],[y1,y2],'go',lw=4)
    
    plt.legend(loc="lower left")
    plt.show()

# This function is to plot the goal path that is determined with RRT* - Smart
def plotRRTStarSmart(goal_path,obstacle_space,ax):
     # print("goal path is:",goal_path)
     p1=goal_path[0]
     p2=goal_path[-1]
     flag=False
    
     while(p1!=p2):
         
       
         q1,w1=p1
         q2,w2=p2
         
         
         # construct line
         slope=(w2-w1)/(q2-q1)
         x=symbols("x")
         
         y=slope*(x-q1)+w1
         
         for x_val in np.linspace(q1, q2,num=100):
             
            y_val= y.subs(x,x_val)
            
            
            if collisionWithRect([x_val,y_val], obstacle_space):
                p1=[q1,w1]
                p2=goal_path[(goal_path.index([q2,w2]))-1]
                break
            elif(x_val==q2):
                ax.plot([q1,q2],[w1,w2],'b-',lw=1.7)
                p1=[q2,w2]
                p2=goal_path[-1]
     ax.plot([q1,q2],[w1,w2],'b-',lw=0.8,label='RRT* Smart')           

# This function is for plotting the cost of the path that has been 
# plotted using RRT* - Smart
def plot_cost(costs,K):
    ax = plt.axes(xlim=(750,K))
    ax.plot(costs,color='blue')
    plt.xlabel("Iterations")
    plt.ylabel("Cost to Go")
    
    plt.grid()
    plt.show()
    