def prepareForUCS(l, goal):
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
