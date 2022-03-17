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