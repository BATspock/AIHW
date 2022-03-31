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
        textfile.close()def prepareForUCS(l, goal):
    nodeInfo = {}
    nodeInfoCost = {}
    #visited = {}

    for _ in range(5, int(l[4]) + 5):
        t = list(map(int,(l[_].split())))
        if (tuple(t[:3])) == beg:
            nodeInfoCost[tuple(t[:3])] = ((0,tuple(t[:3]), tuple(t[:3]),0))
        else:
            nodeInfoCost[tuple(t[:3])] = ((sys.maxsize,tuple(t[:3]), tuple(t[:3]),0))
            
        nodeInfo[tuple(t[:3])] = t[3:]
        #visited[tuple(t[:3])]= 0
        
    if goal not in nodeInfo:
        nodeInfoCost[goal] = ((sys.maxsize, goal, goal, 0))
        nodeInfo[goal] = [-1]
        #visited[goal] = 0
        
    return nodeInfo, nodeInfoCost#, visited#, parent



def _check(l, c):
    if len(l) == 0:
        return False
    if l[0][0] < c:
        return True
    return False



def UCS(nodeInfo, nodeInfoCost, beg, goal):
    
    if (beg not in nodeInfo) : return "FAIL"
    if beg == goal: return -1
    visited = set()
    l = []
    heapq.heappush(l, nodeInfoCost[beg])        
    costToGoal = nodeInfoCost[goal][0]

    
    while(not len(l) == 0 and _check(l, costToGoal)): 
        
        node = heapq.heappop(l)

        if node[1] == goal:
            
            if node[0] < costToGoal:
                costToGoal = node[0]
                
        else:
            for _ in nodeInfo[node[1]]:

                #newNode = tuple(np.array(node[1]) + np.array(act[_-1]))
                #n = list(node)
                newNode = ((node[1][0]+act[_-1][0], node[1][1]+act[_-1][1], node[1][2]+act[_-1][2]))
                
                if newNode in nodeInfo:
                    if newNode not in visited:

                        if (_-1)>5:

                            cost = nodeInfoCost[node[1]][0] +14
                            newNode = ((cost, newNode, tuple(node[1]), 14))

                            if (cost < costToGoal):                                
                                heapq.heappush(l, newNode)
                                
                            if (nodeInfoCost[newNode[1]][0]>cost):
                                nodeInfoCost[newNode[1]] = newNode
                                
                        else:

                            cost = nodeInfoCost[node[1]][0]+10
                            newNode = ((cost, newNode, tuple(node[1]), 10))

                            if (cost < costToGoal):
                                heapq.heappush(l, newNode)

                            if (nodeInfoCost[newNode[1]][0]>cost):
                                nodeInfoCost[newNode[1]] = newNode                                
                                


        if (node[1] != goal):
                #visited[node[1]] = 1
                visited.add(node[1])
            
    if (nodeInfoCost[goal][0] == sys.maxsize):
        return "FAIL"
    
    return nodeInfoCost

def UCSoutput(d, beg, goal):
    path = []
    
    textfile = open("output.txt", "w")
    textfile.writelines(str(d[goal][0])+"\n")
    
    node = goal
    
    while (not d[node][1] == d[node][2]):
        var = list(d[node][1])
        var.append(d[node][3])
        path.append(var)
        node = d[node][2]
        
    var = list(d[beg][1])
    var.append(d[beg][3])
    path.append(var)
    path = path[::-1]
    textfile.writelines(str(len(path))+"\n")
    
    for i in range(len(path)):
        for j in range(len(path[i])):
            if j == 3:
                textfile.writelines(str(path[i][j]))
            else:
                textfile.writelines(str(path[i][j]) + " ")
        if i != len(path)-1:
            textfile.writelines("\n")
    
    textfile.close()

def prepareForAstar(l, goal):
    nodeInfo = {}
    nodeInfoCost = {}
    visited = {}
    heuristicCost = {}
    
    for _ in range(5, int(l[4]) + 5):
        t = list(map(int,(l[_].split())))
        #heuristicCost[tuple(t[:3])] = np.sum(np.sqrt((np.array(tuple(t[:3])) - np.array(goal))**2))
        heuristicCost[tuple(t[:3])] = math.sqrt((t[:3][0]-goal[0])**2 + (t[:3][1]-goal[1])**2 + (t[:3][2]-goal[2])**2)

    for _ in range(5, int(l[4]) + 5):
        t = list(map(int,(l[_].split())))
        if (tuple(t[:3])) == goal:
            nodeInfoCost[tuple(t[:3])] = ((0,sys.maxsize,tuple(t[:3]), tuple(t[:3]),0))
            
        elif (tuple(t[:3])) == beg:
            nodeInfoCost[tuple(t[:3])] = ((0, 0, tuple(t[:3]), tuple(t[:3]), 0))
        else:
            nodeInfoCost[tuple(t[:3])] = ((heuristicCost[tuple(t[:3])],sys.maxsize,tuple(t[:3]), tuple(t[:3]),0))
        nodeInfo[tuple(t[:3])] = t[3:]
        #visited[tuple(t[:3])]= 0
        
    if goal not in nodeInfo:
        nodeInfo[goal] = [-1]
        nodeInfoCost[goal] = ((0, sys.maxsize,goal, goal, 0))
        heuristicCost[goal] = 0
        #visited[goal] = 0    
    del heuristicCost    
    return nodeInfo, nodeInfoCost#, visited, #heuristicCost#, parent


def Astar(nodeInfo, nodeInfoCost, beg, goal):
    
    if (beg not in nodeInfo) : return "FAIL"
    if beg == goal: return -1
    
    l = []
    heapq.heappush(l, nodeInfoCost[beg])        
    costToGoal = nodeInfoCost[goal][1]
    visited = set()
    
    while(not len(l) == 0 and _check(l, costToGoal)): 
        
        node = heapq.heappop(l)

        if node[2] == goal:
            
            if node[1] < costToGoal:
                costToGoal = node[1]
                
        else:
            for _ in nodeInfo[node[2]]:

                #newNode = tuple(np.array(node[2]) + np.array(act[_-1]))
                #n = list(node)
                newNode = ((node[2][0]+act[_-1][0], node[2][1]+act[_-1][1], node[2][2]+act[_-1][2]))
                
                if newNode in nodeInfo:
                    if newNode not in visited:

                        if (_-1)>5:
                            #update actual cost
                            hcost = nodeInfoCost[node[2]][1] + nodeInfoCost[newNode][0]+14
                            cost = nodeInfoCost[node[2]][1] +14
                            newNode = ((hcost, cost, newNode, tuple(node[2]), 14))

                            if (hcost < costToGoal):                                
                                heapq.heappush(l, newNode)
                                
                            if (nodeInfoCost[newNode[2]][1]>cost):
                                nodeInfoCost[newNode[2]] = newNode
                                
                        else:
                            
                            hcost = nodeInfoCost[node[2]][1] + nodeInfoCost[newNode][0] +10
                            cost = nodeInfoCost[node[2]][1]+10
                            newNode = ((hcost, cost, newNode, tuple(node[2]), 10))

                            if (hcost < costToGoal):
                                heapq.heappush(l, newNode)

                            if (nodeInfoCost[newNode[2]][1]>cost):
                                nodeInfoCost[newNode[2]] = newNode                                
                                


        if (node[2] != goal):
                #visited[node[2]] = 1
                visited.add(node[2])
            
    if (nodeInfoCost[goal][1] == sys.maxsize):
        return "FAIL"
    
    return nodeInfoCost

def AstartOutput(d, beg, goal):
    path = []
    
    textfile = open("output.txt", "w")
    textfile.writelines(str(d[goal][0])+"\n")
    
    node = goal
    
    while (not d[node][2] == d[node][3]):
        var = list(d[node][2])
        var.append(d[node][4])
        path.append(var)
        node = d[node][3]
        
    var = list(d[beg][2])
    var.append(d[beg][4])
    path.append(var)
    path = path[::-1]
    textfile.writelines(str(len(path))+"\n")
    
    for i in range(len(path)):
        for j in range(len(path[i])):
            if j == 3:
                textfile.writelines(str(path[i][j]))
            else:
                textfile.writelines(str(path[i][j]) + " ")
        if i != len(path)-1:
            textfile.writelines("\n")
    
    textfile.close()
 