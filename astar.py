#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 18 05:15:59 2021

@author: KalebaKeitshokile
"""
import numpy as np

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self,parent, position):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    def __lt__(self, other):
        return self.f < other.f

def astar4connected(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = np.array([])
    closed_list = []

    # Add the start node
    open_list = np.append(open_list,start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        open_list = np.sort(open_list)
        # Get the current node
        current_node = open_list[0]

        # Pop current off open list, add to closed list
        open_list = np.delete(open_list,0)
        closed_list.append(current_node)


        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position and check around it

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > np.shape(maze)[0]-1 or node_position[0] < 0 or node_position[1] > np.shape(maze)[1]-1 or node_position[1] < 0:
                continue

            # Make sure walkable terrain

            if maze[node_position[0]][node_position[1]] ==1:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            if child in closed_list:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = (abs((child.position[0]-end_node.position[0]))+abs((child.position[1]-end_node.position[1])**2))
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list = np.append(open_list,child)
            open_list = np.unique(open_list)

def astar8connected(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = np.array([])
    closed_list = []

    # Add the start node
    open_list = np.append(open_list,start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        open_list = np.sort(open_list)
        # Get the current node
        current_node = open_list[0]

        # Pop current off open list, add to closed list
        open_list = np.delete(open_list,0)
        closed_list.append(current_node)


        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0),(-1,- 1),(1, 1),(-1,1 ),(1,- 1)]: # Adjacent squares

            # Get node position and check around it
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            xx = current_node.position[0]
            xy = current_node.position[1]
            yx = node_position[0]
            yy = node_position[1]
            (x1,y1) = (xx-(xx-yx),yy-(yy-xy))
            (x2,y2) = (yx-(yx-xx),xy-(xy-yy))

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] ==1:
                continue
            #Make sure you're not walking through diagonally adjacent walls
            if maze[x1][y1] == maze[x2][y2] == 1:
                continue
            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            if child in closed_list:
                continue

            # Create the f, g, and h values
            child.g = current_node.g+np.sqrt(abs((child.position[0]-current_node.position[0])**2)+abs((child.position[1]-current_node.position[1])**2))
            child.h = np.sqrt(abs((child.position[0]-end_node.position[0])**2)+abs((child.position[1]-end_node.position[1])**2))
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list = np.append(open_list,child)
            open_list = np.unique(open_list)
