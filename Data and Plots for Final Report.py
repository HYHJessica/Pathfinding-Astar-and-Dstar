#!/usr/bin/env python
# coding: utf-8

# In[1]:


from astar import astar4connected as astar
import numpy as np
from dstar import makenodes, Dstar
import dstar
import time
import pandas as pd
import matplotlib.pyplot as plt
import os
import pygame
from tqdm import tqdm, trange


# In[2]:


def rover(grid,pos,fov):
    """Returns a list of tuples (x, y) containing the 
    coordinates of the viewable space around the rover"""
    v = np.transpose((np.zeros_like(grid)==0).nonzero())
    view = [(rx,ry) for (rx,ry) in v if ( pos[0] - fov < rx < pos[0] + fov and pos[1] - fov < ry < pos[1] + fov)]
    return view

def moving_average(x,w):
    return np.convolve(x,np.ones(w), 'valid')/w

def best_fit(x,y,deg):
    z = np.polyfit(x,y,deg)
    p = np.poly1d(z)
    return p(x)


# In[3]:


x,y,connections = 2,1000,4
start,goal = (0,0),(0,999)


maze = np.zeros((x,y))

for index,row in enumerate(maze):
    row[index*2+1:y:4] = 1
    
f = 'Nodes/(%s, %s) Init %s Connected.pickle' %(x,y,connections)
cwd = os.getcwd()
path = os.path.join(cwd,f)
if os.path.isfile(path):
    nodes = makenodes.reconstruct(path)
else:
    nodes = makenodes.makenodes(x,y,4,False,True,False)


# In[4]:


travelledDstar =[]
disttogoalDstar = []
ObstaclesDstar = []
repairsDstar = []
initialplansDstar = []
t = tqdm(range(1),desc = 'D* Incremental Repairing Test 0/1500 Travelled')
for i in t:

    travelled = []
    obstacles = []
    repairs = []
    disttogoal = []
    
    initial = time.time()
    planner = dstar.Dstar();
    nodes = planner.reset(nodes);
    P_Dstar = planner.plan(maze,nodes,start,goal)
    initialplansDstar.append(time.time()-initial)
    
    while P_Dstar:
        current_pos = P_Dstar[0]
        travelled.append(current_pos)
        P_Dstar.pop(0)

        current_node = next(node for node in nodes if node.position == current_pos)
        
        view = rover(maze,current_pos,4)
        
        if current_pos == goal: break
        for v in view:
            if maze[v[0]][v[1]] ==1 and (v[0],v[1]) not in obstacles:
                obstacle_pos = (v[0],v[1])
                obstacles.append(obstacle_pos)
                planner.modify_cost('Gate',10000,obstacle_pos)
                change = True
        if change:
            change = False
            repair = time.time()
            P_Dstar = planner.repair(current_node)
            repairs.append(time.time()-repair)
            disttogoal.append(len(P_Dstar))
            t.refresh()
            t.set_description('D* Incremental Repairing Test ' + str(len(travelled))+'/'+'1500 Travelled')
    repairsDstar.append(repairs)
    travelledDstar.append(travelled)
    disttogoalDstar.append(disttogoal)


# In[ ]:


replansAstar = []
initialplansAstar = []
disttogoalAstar = []
travelledAstar = []
t = tqdm(range(1),desc = 'A* Brute-Force Replanning Test 0/1500 Travelled')
for i in t:
    
    grid = np.zeros_like(maze)
    obstacles = []
    travelled = []
    replans = []
    disttogoal = []
    initial_time = time.time()
    P_Astar = astar(grid,start,goal)
    initialplansAstar.append(time.time()-initial_time)
    
    while P_Astar:
        
        current_pos = P_Astar[0]
        travelled.append(current_pos)
        P_Astar.pop(0)
        current_node = next(node for node in nodes if node.position == current_pos)

        view = rover(maze,current_pos,4)

        if current_pos == goal: break
        for v in view:
            if maze[v[0]][v[1]] ==1 and (v[0],v[1]) not in obstacles:
                obstacle_pos = (v[0],v[1])
                obstacles.append(obstacle_pos)
                grid[v[0]][v[1]] = 1
        if any(obstacle in P_Astar for obstacle in obstacles):
            replan = time.time()
            disttogoal.append(len(P_Astar))
            P_Astar = astar(grid,current_pos,goal)[1::]
            replans.append(time.time()-replan)
            t.refresh()
            t.set_description('A* Brute-Force Replanning Test ' + str(len(travelled))+'/'+'1500 Travelled')
    replansAstar.append(replans)
    travelledAstar.append(travelled)
    disttogoalAstar.append(disttogoal)


# In[ ]:


NodeTime = []
t = tqdm(range(100),desc = 'Testing Make-Nodes D* Grid = (0, 0)')
for i in t:
    t1 = time.time()
    nodes = makenodes.makenodes(i,i,4,0,0,0)
    t1 = time.time()-t1
    NodeTime.append(t1)
    t.set_description('Testing Make-Nodes D* Grid = (%s, %s)'%(i,i))
    


# In[ ]:


AstarTimes = []
t = tqdm(range(100),desc = 'Testing A* Initial Paths Grid = (0,0)')
for i in t:
    t1 = time.time()
    path = astar(np.zeros((i,i)),(0,0),(i-1,i-1))
    t1 = time.time()-t1
    NodeTime.append(t1)
    t.set_description('Testing A* Initial Paths Grid = (%s, %s)'%(i,i))


# In[ ]:


p1 = plt.scatter(disttogoalDstar[0],D1,1,label='Test Data')
p2, = plt.plot(disttogoalDstar[0],best_fit(disttogoalDstar[0],D1,2),label = '2nd Order PloyFit',color = 'red')
plt.title("D* Path Repair Time vs Distances to Goal.")
plt.xlabel("Distance to Goal (Nodes)")
plt.ylabel("Time (s)")
plt.legend(handles=[p1, p2], title='Legend', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()


# In[ ]:


p1 = plt.scatter(range(len(D1)),D1,1,label='Test Data')
p2, = plt.plot(best_fit(disttogoalDstar[0],D1,2),label = '2nd Order PloyFit',color = 'red')
plt.title("D* Path Repair Time from Start to Goal")
plt.xlabel("Repair Iteration")
plt.ylabel("Time (s)")
plt.legend(handles=[p1, p2], title='Legend', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()


# In[ ]:


p1 =plt.scatter(range(len(D1)),np.cumsum(D1),2,label='Test Data')
p2, = plt.plot(best_fit(range(len(D1)),np.cumsum(D1),3),label = '3rd Order PolyFit',color = 'red')
plt.title("Cumulative Sum of D* Repair Times from Start to Goal.")
plt.xlabel("Repair Iteration")
plt.ylabel("Time (s)")
plt.legend(handles=[p1,p2], title='Legend', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()


# In[ ]:


p1 = plt.scatter(disttogoalAstar[1:],D2,2,label = 'Test Data')
p2, = plt.plot(disttogoalAstar[1:],best_fit(disttogoalAstar[1:],D2,2), label = '2nd Order PolyFit',color = 'red')
plt.title("Brute-Force Replanning Time vs Distances to the Goal.")
plt.xlabel("Distance to Goal (Nodes)")
plt.ylabel("Time (s)")
plt.legend(handles=[p1, p2], title='Legend', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()


# In[ ]:


p1 = plt.scatter(range(len(D2)),D2,2, label = 'Test Data')
p2, = plt.plot(best_fit(range(len(D2)),D2,2), label = '2nd Order PolyFit',color = 'red')
plt.title("Brute-Force Replanning Time from Start to Goal.")
plt.ylabel("Time (s)")
plt.xlabel("Replanning Iteration")
plt.legend(handles=[p1, p2], title='Legend', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()


# In[ ]:


plt.plot(np.cumsum(D2),label = 'Test Data')
p2, = plt.plot(best_fit(range(len(D2)),np.cumsum(D2),3),label = '3rd Order PolyFit',color = 'red')
plt.title("Cumulative Sum of Brute-Force Replanning Times from Start to Goal.")
plt.ylabel("Time (s)")
plt.xlabel("Replanning Iteration")
plt.legend(handles=[p1,p2], title='Legend', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()


# In[ ]:


p1 = plt.scatter(range(len(NodeTime)),NodeTime,5,label='Test Data')
p2, = plt.plot(best_fit(range(len(NodeTime)),NodeTime,4),label = '4th Order PloyFit',color = 'red')
plt.title("D* Time to Make Nodes on Increasing Square Grids")
plt.xlabel("Grid Size (N x N)")
plt.ylabel("Time (s)")
plt.legend(handles=[p1, p2], title='Legend', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()


# In[ ]:


p1 = plt.scatter(range(len(AstarTimes)),AstarTimes,5,label='Test Data')
p2, = plt.plot(best_fit(range(len(AstarTimes)),AstarTimes,2),label = '2nd Order PloyFit',color = 'red')
plt.title("A* Paths on Increasing Square Grids ")
plt.xlabel("Grid Size (N x N)")
plt.ylabel("Time (s)")
plt.legend(handles=[p1, p2], title='Legend', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()

