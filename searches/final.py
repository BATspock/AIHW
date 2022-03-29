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
import queue 
import sys

def prepareForBFS(l, goal):
        nodeInfo = {}
        #visited = {}
        parent = {}

        for _ in range(5, int(l[4]) + 5):
            t = list(map(int,(l[_].split())))
            nodeInfo[tuple(t[:3])] = t[3:]
            #visited[tuple(t[:3])]= 0
            parent[tuple(t[:3])] = tuple(t[:3])
            
        if goal not in nodeInfo:
            nodeInfo[goal] = [-1]
            #visited[goal] = 0
            parent[goal] = goal


        return nodeInfo, parent


def BFS(nodeInfo, parent, beg, goal):
        
        if (beg not in nodeInfo) : return "FAIL"
        
        if beg == goal: return -1
        
        visited= set()
        q.put(beg)
        #visited[beg] = 1
        visited.add(beg)

        while(not q.empty()):
    
            node = q.get()
            #print(node)

            if (node == goal):
                return parent 

            for _ in nodeInfo[node]:
                #print(node)
                #newNode = tuple(np.array(list(node)) + np.array(act[_-1]))
                
                #n = list(node)
                newNode = ((node[0]+act[_-1][0], node[1]+act[_-1][1], node[2]+act[_-1][2]))
                if newNode in nodeInfo:
                    if  newNode in visited:
                        continue 
                    else:
                        visited.add(newNode)
                        parent[newNode] = node
                        q.put(newNode)


        return ("FAIL")


def BFSCost(parent, goal):
        costList = []
        node = goal
        
        while (parent[node]!=node):
            #print(node)
            costList.append(str(node[0]) + " " + str(node[1]) + " "+ str(node[2]) + " " + str(1))
            node = parent[node]
        
        #print(costList)
        costList.append(str(beg[0]) + " " + str(beg[1]) + " " + str(beg[2]) + " " + str(0))
        costList = costList[::-1]
        #costList.insert(0, (list(beg)+list(str(0))))

        textfile = open("output.txt", "w")
        textfile.write(str(len(costList)-1)+"\n"+str(len(costList))+"\n")
        for c in costList:
            textfile.write(c + "\n")
        textfile.close()