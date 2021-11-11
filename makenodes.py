#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 15 00:57:52 2021

@author: KalebaKeitshokile
"""
import pickle
import numpy as np
from tqdm import tqdm
import os

class Node():
    
    """A node class for D* Pathfinding"""
    
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position
        self.tag = "New"
        self.state = "Empty"
        self.h = 0
        self.k = 0
        self.neighbours = []

    def __eq__(self, other):
        if type(other) != Node : return False
        else: return self.position == other.position
    def __lt__(self, other):
        return self.k < other.k

def getneighbours(current_node,connections,nodes,grid):

    '''Get the neighbours of the given node as Node() objects'''
    
    neighbours = []
    positions = [(0, -1), (0, 1), (-1, 0), (1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    for new_position in positions[:connections]: # Adjacent squares

            # Get node position
            node_position = (int(current_node.position[0] + new_position[0]), int(current_node.position[1] + new_position[1]))

            # Make sure within range
            if node_position[0] > (len(grid) - 1) or node_position[0] < 0 or node_position[1] > (len(grid[len(grid)-1]) -1) or node_position[1] < 0:
                continue

            # Create new node
            new_node = [x for x in nodes if x.position == node_position][0]
            neighbours.append(new_node)
    return neighbours

def reconstruct(filename):

    """Reconstruct Pickled Data From a specific File"""
    
    with open(filename, 'rb') as file:
        f = pickle.load(file)
    nodes = f[0]
    neighbours = f[1]
    reconstructed = []
    for node in nodes:
        i = node[0]
        for neighbour in neighbours[i][1]:
            node[1].neighbours.append(neighbour)
        reconstructed.append(node[1])
    return reconstructed

def getendpoints(grid):
    
    """returns all locations on the grid"""
    
    x,y = np.where(grid == 0)
    endpoints = [(x,y) for (x,y) in zip(x,y)]
    return endpoints

def makenodes(x,y,connections,directory,save,progress_bar):

    """Given all the endpoints, make nodes for them"""
    
    grid = np.zeros((x,y))
    endpoints = getendpoints(grid)
    pre_nodes = [Node(None, pos) for pos in endpoints]
    neighbours = []
    nodes = []
    if progress_bar:
        t = tqdm(pre_nodes, desc = 'Making Nodes')
    else: t = pre_nodes
    for i, node in enumerate(t):
        
        neighbour = getneighbours(node, connections,pre_nodes,grid)
        nodes.append([i, node])
        neighbours.append([i, neighbour])
        
    arr = [nodes,neighbours]
    if save:
        f = 'Nodes/(%s, %s) Init %s Connected.pickle' %(x,y,connections)
        if directory:
            filename = os.path.join(directory,f)
        else: 
            cwd = os.getcwd()
            filename = os.path.join(cwd,f)
        with open(filename, 'wb') as file:
            pickle.dump(arr,file)

    reconstructed = []
    for node in nodes:
        i = node[0]
        for neighbour in neighbours[i][1]:
            node[1].neighbours.append(neighbour)
        reconstructed.append(node[1])
    return reconstructed