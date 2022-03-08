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