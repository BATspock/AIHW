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
