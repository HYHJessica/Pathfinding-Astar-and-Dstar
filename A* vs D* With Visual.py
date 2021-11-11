#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May  8 22:24:24 2021

@author: KalebaKeitshokile
"""
import os 
import numpy as np
import pygame
import dstar
from funcs import randommap as randommap
from funcs import *
from astar import astar4connected as astar
import sys


#Drawing Functions to update grid and simulate a rover.

def drawrect(screen,color,row,column):
    """
    Draw a single block on the pygame window
    at coordinates (column, row)
    """
    pygame.draw.rect(screen,
                             color,
                             [(MARGIN + WIDTH) * column + MARGIN,
                              (MARGIN + HEIGHT) * row + MARGIN,
                              WIDTH,
                              HEIGHT])
def update_grid(grid):
    """
    Update pygame window
    """
    x = np.shape(grid)[0]
    y = np.shape(grid)[1]
    for row in range(x):
        for column in range(y):
            if grid[row][column] == 1:
                color = BLACK
            elif grid[row][column]== 0:
                color = WHITE
            elif grid[row][column] == 2:
                color = RED
            elif grid[row][column] == 3:
                color = PURPLE
            elif grid[row][column] == 4:
                color = BLUE
            elif grid[row][column]== 5:
                color = ORANGE
            elif grid[row][column] == 6:
                color = YELLOW
            elif grid[row][column]==7:
                color = GREEN
            elif grid[row][column]==8:
                color=MAROON
            elif grid[row][column]==9:
                color = PINK
            elif grid[row][column]==10:
                color=SKY_BLUE
            drawrect(screen,color,row,column)
    pygame.display.flip()


def rover(grid,pos,fov):
    """
    Return coordintes around current loaction
    within fov
    """
    v = np.transpose((np.zeros_like(grid)==0).nonzero())
    view = [(rx,ry) for (rx,ry) in v if ( pos[0] - fov < rx < pos[0] + fov and pos[1] - fov < ry < pos[1] + fov)]
    for v in view:
        if maze[v[0]][v[1]]==1:
            grid[v[0]][v[1]] = 1
        if grid[v[0]][v[1]] == 0: 
            grid[v[0]][v[1]] = 6

    update_grid(grid)
    return set(view)
    
def set_points(grid):
    """
     Return start,end

     waits for mouse down to select start and end at
     mouse window coordinates
    """
    i = 0
    startt =False
    while startt == False:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.MOUSEBUTTONDOWN:
                # User clicks the mouse. Get the position
                pos = pygame.mouse.get_pos()
                # Change the x/y screen coordinates to grid coordinates
                column = int(pos[0] // (WIDTH + MARGIN))
                row = int(pos[1] // (HEIGHT + MARGIN))
                if row>np.shape(grid)[0]-1 or column> np.shape(grid)[1]-1:
                    continue
                elif grid[row][column]==1: continue
                if i == 0: 
                    start = (row,column)
                    grid[row][column]=2
                    i +=1
                    update_grid(grid)
                elif i == 1:
                    end = (row,column)
                    grid[row][column]= 4
                    update_grid(grid)
                    startt = True
                    
                
    return start,end

done = False
#Configuring the maze
while not done:
    #Taking Optional parameters to perform different tests
    try:
        usr = int(input('1. Grid Used in Report (First 100 Columns).\n\n2. Load Random Grid.\n\nEnter a number to choose an option: '))
        #Regenerate the grid used in the report
        if usr == 1:
            x = 2
            y = 100
            maze = np.zeros((x,y))
            for i,pos in enumerate(maze):
                pos[i*2+1:y:4] = 1
            done = True
            continue
        #Make a custon grid
        elif usr == 2:

            x = abs(int(input('\nEnter the number of nodes in the x direction I typically use 100: ')))
            y = abs(int(input('\nEnter the number of nodes in the y direction I typically use 100: ')))
            d = abs(float(input('\nEnter the ratio of obstacles to empty spaces (between 0 and 1) I typically use 0.2: ')))
            maze = randommap(x,y,d)
            done = True
            continue
    except: pass
    print('\nThat was an invalid input.')

           
pygame.init() #Initialise pygame
connection = 4 #4 Connected or 8 Connected grid
fov = 10 #How many blocks around itself the robot sees
grid = np.copy(maze) #Copy maze

#Pygame window size, color and title parameters
scale = 50/((x+y)/2) *0.7
WIDTH = 17*scale
HEIGHT = 17*scale
MARGIN = 3*scale
WINDOW_SIZE = [int(y*HEIGHT+(y+1)*MARGIN),int(x*WIDTH+(x+1)*MARGIN)]
screen = pygame.display.set_mode(WINDOW_SIZE)
screen.fill(BLACK) 
pygame.display.set_caption("%s Connected D* Unknown Grid"%(connection))
update_grid(grid)
#Locate node files if they exists, else create the nodes
cwd = os.path.dirname(os.path.abspath(__file__))
f = 'Nodes/(%s, %s) Init %s Connected.pickle' %(x,y,connection)
path = os.path.join(cwd,f)
if os.path.isfile(path):
    nodes = dstar.makenodes.reconstruct(path)
else:
    nodes = dstar.makenodes.makenodes(x,y,4,cwd,True,False)

done = False
chnage = False
#Main Path Planning
while not done:
    update_grid(grid)
    grid = np.copy(maze)
    grid2 = np.zeros_like(maze)    
    start,goal = set_points(grid)
    update_grid(grid)

    #Generate A* initial path
    P = astar(grid2,start,goal)
    obstacles = set()

    #Rover following A* path
    while P:
                
        current_pos = P[0]
        P.pop(0)
        #Change the color of the current block
        if grid[current_pos[0]][current_pos[1]]!=1 and grid[current_pos[0]][current_pos[1]]!= 4 and grid[current_pos[0]][current_pos[1]]!=2:
            grid[current_pos[0]][current_pos[1]]=9
        for pos in P[1:-1]:
            if grid[pos[0]][pos[1]] != 1:
                grid[pos[0]][pos[1]]=10

        current_node = next(node for node in nodes if node.position == current_pos)
        
        view = rover(grid,current_pos,fov)
        
        if current_pos == goal: break
        for v in view:
            if maze[v[0]][v[1]] ==1 and (v[0],v[1]) not in obstacles:
                grid2[v[0]][v[1]] = 1
                obstacle_pos = (v[0],v[1])
                obstacles.add(obstacle_pos)
                change = True
        if any(obstacle in P for obstacle in obstacles):
            P = astar(grid2,current_pos,goal)

        
        
        for event in pygame.event.get():  
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()


    update_grid(grid)
    #Initialise D* path planner object
    planner = dstar.Dstar();planner.reset(nodes)
    P = planner.plan(grid,nodes,start,goal)
    obstacles = set()
    change = False
    #Rover following D* path
    while P:
                
        current_pos = P[0]
        P.pop(0)
        
        if grid[current_pos[0]][current_pos[1]]!=1 and grid[current_pos[0]][current_pos[1]]!= 4 and grid[current_pos[0]][current_pos[1]]!=2:
            grid[current_pos[0]][current_pos[1]]=8
        for pos in P[1:-1]:
            if grid[pos[0]][pos[1]] != 1:
                grid[pos[0]][pos[1]]=7

        current_node = next(node for node in nodes if node.position == current_pos)
        
        view = rover(grid,current_pos,fov)
        
        if current_pos == goal: break
        for v in view:
            if maze[v[0]][v[1]] ==1 and (v[0],v[1]) not in obstacles:
                obstacle_pos = (v[0],v[1])
                obstacles.add(obstacle_pos)
                planner.modify_cost('Gate',10000,obstacle_pos)

        P = planner.repair(current_node)
        if P == None:
            """This is included as there is a problem with D* where
             it sometimes assigns incorrect parent relationships causing 
             the program to return no path when in fact there is one. 
             The cause of this hasn't been identified so this loop resets 
             the nodes but keeps their state. It is then followed by performing
            a new initial plan from the current location. 
            It still works because if the robot is indeed trapped, P will still return None
            """
            for node in nodes:
                node.parent = None
                node.h = node.k = 0 
                node.tag = 'New'
            P = planner.plan(grid,nodes,current_pos,goal)

        
        
        for event in pygame.event.get():  
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
    
    restart = True

    #Restart or quit
    while restart: 
        if done == True: restart = True
        for event in pygame.event.get():  
            if event.type == pygame.QUIT:
                done = True  
                restart = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                restart = False
                
pygame.quit()