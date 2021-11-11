#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May  6 23:21:51 2021

@author: KalebaKeitshokile
"""
import numpy as np
import makenodes
from makenodes import Node as Node

class Dstar():
    """
    D* path planning object call as follows:
    planner = Dstar()
    path = planner.plan(grid,nodes,start,goal)
    path = planner.repair(current_node)
    """

    def __init__(self):
        """
        Initialise.
        max_cost = 10000 (Increase for grids larger than 10000 nodes)
        """
        self.max_cost = 10000
        self.open_list = []

    def getbackpointers(self,current,goal):
        '''Accepts list of nodes, current node and goal node and returns list of tuples in path or None'''
        P = []
        while current != goal:
            current = current.parent
            if current.state == 'Gate':
                print('Problem in path. Check for obstacles.')
                return None

            if not current in P:
                P.append(current.position)
            else:
                print('2 Nodes Referring to each other as parents %s and %s'%(current.position,current.parent.position))
                return  None
        return P

    def process_state(self):
        """Assign the correct cost and parent to the relevant nodes in the search"""
        if len(self.open_list) == 0: return -1
        x = min(self.open_list)
        k_old = x.k

        self.delete(x)

        if k_old < x.h: # Raise

            for y in x.neighbours:

                if y.tag != 'New' and y.h <= k_old and x.h > y.h + self.cost(y,x):

                    x.parent = y
                    x.h = y.h + self.cost(y,x)

        if k_old == x.h: #Lower

            for y in x.neighbours:

                if y.tag == "New" or y.parent == x and y.h != x.h + self.cost(x,y) or y.parent != x and y.h > x.h + self.cost(x,y):
                    y.parent = x
                    self.insert(y,x.h + self.cost(x,y))

        else:

            for y in x.neighbours:

                if y.tag == "New" or y.parent == x and y.h != x.h + self.cost(x,y):

                    y.parent = x
                    self.insert(y,x.h + self.cost(x,y))
                else:

                    if y.parent != x and y.h > x.h + self.cost(x,y):
                        self.insert(x,x.h)

                    else:

                        if y.parent != x and x.h > y.h + self.cost(y,x) and y.tag == "Closed" and y.h > k_old:
                            self.insert(y,y.h)

        if len(self.open_list) == 0: return -1
        else: return min(self.open_list).k


    def cost(self,X,Y):

        """Takes X and Y as Nodes. Returns cost of moving from X to Y."""

        if Y.state == 'Gate':

            return 10 * self.max_cost

        elif Y.state == 'Empty':

            c = np.sqrt(abs((X.position[0]-Y.position[0])**2)+abs((X.position[1]-Y.position[1])**2))
            if c == 1: return c
            else: return 1.4

    def insert(self,node,h_new):

        '''Add a node to the open list and change it's tage accordinly '''

        if node.tag == 'New':
            node.k = h_new
        elif node.tag == 'Open':
            node.k = min(node.k,h_new)
        elif node.tag == 'Closed':
            node.k = min(h_new,node.h)
        node.h = h_new
        node.tag = 'Open'
        self.open_list = np.append(self.open_list,node)


    def delete(self,node):
        '''Close a node and remove it from the open list'''

        node.tag = 'Closed'
        self.open_list = np.delete(self.open_list,0)
        self.open_list = np.unique(self.open_list)


    def modify_cost(self,tag,sensor_cost,position):

        '''Change the cost of a node and if it is closed put it on the open list with the new cost'''
        node = [node for node in self.nodes if node.position == position][0]
        node.state = tag
        if node.tag == 'Closed':
           self.insert(node,sensor_cost)


    def plan(self,grid,nodes,start,goal):

        ''' Initial Path Planning'''

        self.grid = grid
        self.nodes = nodes
        self.goal_node = [node for node in nodes if node.position == goal][0]
        self.goal_node.k = self.goal_node.h = 0
        self.insert(self.goal_node,0)
        self.start_node = [node for node in nodes if node.position == start][0]
        kmin = 0

        while self.start_node.tag != 'Closed' and kmin != -1:
            kmin = self.process_state()
        if self.start_node.tag == 'New': return None
        path = self.getbackpointers(self.start_node, self.goal_node)
        path.insert(0,start)
        return path

    def repair(self,current_node):
        """Repair a path when an obstcle is detected"""
        kmin = current_node.h
        if any(self.open_list): kmin = min(self.open_list).k

        while kmin < current_node.h and kmin != -1:

            kmin = self.process_state()

        if kmin == -1:

            return None
        else:
            path = self.getbackpointers(current_node,self.goal_node)
        return path

    def reset(self,nodes):
        """Reset all Nodes to New, Empty and 0 with no parents"""
        for node in nodes:
            node.tag = 'New'
            node.h = node.k = 0
            node.parent = None
            node.state = 'Empty'
        return nodes
