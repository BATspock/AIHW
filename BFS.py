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

def BFS(self, nodeInfo,visited, parent, beg, goal, dim):
        
        if (beg not in nodeInfo) : return "FAIL"
        

        q.put(beg)
        visited[beg] = 1

        while(not q.empty()):

            node = q.get()

            if (node == goal):
                return parent

            for _ in nodeInfo[node]:

                newNode = tuple(np.array(node) + np.array(act[_-1]))
                if newNode in nodeInfo:
                    if visited[newNode] == 0:
                        visited[newNode] = 1
                        parent[newNode] = node
                        q.put(newNode)

        return ("FAIL")

