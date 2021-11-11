#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 18 05:37:34 2021

@author: KalebaKeitshokile
"""
import numpy as np

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 105)
RED = (255, 0, 0)
ORANGE = (255,155,0)
PURPLE = (200,0,255)
BLUE = (0,100,200)
YELLOW = (255,255,0)
PINK = (255,0,255)
SKY_BLUE = (0,255,255)
MAROON = (128,0,0)

def randommap(x,y,ratio):
    
    """Create a random map of x by y with obstacles to empty space ratio"""
    
    ratio = int(y * x * ratio)
    arr = np.zeros(y*x)
    arr[:ratio]=1
    np.random.shuffle(arr)
    arr = np.reshape(arr,(x,y)).astype('int8')
    return arr