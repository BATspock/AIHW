#import os 
import queue
import heapq
#import numpy as np
import sys
import math
from queue import PriorityQueue


f = open('input.txt')
inputs = f.read().splitlines()

algo = inputs[0]
#dim = tuple(map(int,inputs[1].split()))
beg = tuple(map(int,inputs[2].split()))
goal = tuple(map(int, inputs[3].split()))
nodes = inputs[4]


act = [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1],[1,1,0],[1,-1,0],[-1,1,0],[-1,-1,0],[1,0,1],[1,0,-1],[-1,0,1],[-1,0,-1],[0,1,1],[0,1,-1],[0,-1,1],[0,-1,-1]]
#cost = [10, 10, 10, 10, 10, 10, 14, 14, 14, 14, 14, 14, 14,14, 14, 14, 14, 14]
