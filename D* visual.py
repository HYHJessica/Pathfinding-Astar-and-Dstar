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

connection,x,y = 4, 100, 100
scale = 50/((x+y)/2) *0.7


"""Drawing Functions to update grid and simulate a rover."""

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
ORANGE = (255,155,0)
PURPLE = (200,0,255)
BLUE = (0,100,200)
YELLOW = (255,255,0)

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
#Create maze and grid and find/create nodes
cwd = os.path.dirname(os.path.abspath(__file__))
filename = cwd+'/Nodes/(%s, %s) Init %s Connected.pickle'%(x,y,connection)
nodes = dstar.makenodes.reconstruct(filename)
maze =randommap(100,100,0.3)
grid = np.copy(maze)

#Pygame window parameters
clock = pygame.time.Clock()
clock.tick(1000)
WIDTH = 17*scale
HEIGHT = 17*scale
MARGIN = 3*scale
WINDOW_SIZE = [int(y*HEIGHT+(y+1)*MARGIN),int(x*WIDTH+(x+1)*MARGIN)]
screen = pygame.display.set_mode(WINDOW_SIZE)
screen.fill(BLACK)
pygame.display.set_caption("%s Connected D* Unknown Grid"%(connection))
update_grid(grid)

#Main Loop
done = False
while not done:

    grid = np.copy(maze)
    grid2 = np.zeros_like(maze)

    start,goal = set_points(grid)
    update_grid(grid)
    planner = dstar.Dstar();planner.reset(nodes);P = planner.plan(grid,nodes,start,goal)
    obstacles = []

    #Follow Path
    while P:
        current_pos = P[0]
        P.pop(0)

        if grid[current_pos[0]][current_pos[1]]!=1:
            grid[current_pos[0]][current_pos[1]]=4
        for pos in P[1:]:
            if grid[pos[0]][pos[1]] != 1:
                grid[pos[0]][pos[1]]=7

        current_node = next(node for node in nodes if node.position == current_pos)

        view = rover(grid,current_pos,4)

        if current_pos == goal: break
        for v in view:
            if maze[v[0]][v[1]] ==1 and (v[0],v[1]) not in obstacles:
                grid2[v[0]][v[1]] = 1
                obstacle_pos = (v[0],v[1])
                obstacles.append(obstacle_pos)
                planner.modify_cost('Gate',10000,obstacle_pos)


        P = planner.repair(current_node)

    #Restart or Quit
    restart = True
    while restart:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
                restart = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                restart = False

pygame.quit()
