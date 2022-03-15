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

