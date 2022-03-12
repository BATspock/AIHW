from copy import deepcopy

def readInput(n, path="input.txt"):

    with open(path, 'r') as f:
        lines = f.readlines()

        piece_type = int(lines[0])

        previous_board = [[int(x) for x in line.rstrip('\n')] for line in lines[1:n+1]]
        board = [[int(x) for x in line.rstrip('\n')] for line in lines[n+1: 2*n+1]]

        return piece_type, previous_board, board
    
def writeOutput(result, path="output.txt"):
    res = ""
    if result == "PASS":
    	res = "PASS"
    else:
	    res += str(result[0]) + ',' + str(result[1])

    with open(path, 'w') as f:
        f.write(res)    




def coordinateNeighbors(i, j, board)->list:
    neighbors = []
        # Detect borders and add neighbor coordinates
    if i > 0: neighbors.append((i-1, j))
    if i < len(board) - 1: neighbors.append((i+1, j))
    if j > 0: neighbors.append((i, j-1))
    if j < len(board) - 1: neighbors.append((i, j+1))
    return neighbors

def groupStones(i, j, board, myPlayer)->list: 
    stack = [(i,j)]
    visited = {}
    visited[(i,j)] = True
    ally_members = []
    #print(stack)
    while stack:
        piece = stack.pop()
        ally_members.append(piece)
        #print("coordinate neighbors: ", coordinateNeighbors(i, j, board))
        for n in coordinateNeighbors(piece[0], piece[1], board):
            #print(n)
            #print("visted: ", visited)
            if board[n[0]][n[1]] == myPlayer: 
                
                if n not in visited:
                    
                    stack.append(n)
                    visited[n] = True
    #print(visited)
    return ally_members


def checkLiberty(i, j, board, myPlayer)-> bool:
    #print("######", i, j)
    for s in groupStones(i, j, board, myPlayer):
        #print(s)
        for n in coordinateNeighbors(s[0], s[1], board):
            if board[n[0]][n[1]] == 0:
                return True
    return False

def deadStonesCoordinates(board, myPlayer)->list:
    deadStones = list()
    for i in range(len(board)):
        for j in range(len(board)):
            if board[i][j] == myPlayer:
                if not checkLiberty(i, j, board, myPlayer):
                    #print(i,j)
                    deadStones.append((i,j))
    #print("dead stones array: ", deadStones)
    return deadStones



def libertyPresent(i, j, board, myPlayer)->set:
    liberty = set()

    for s in groupStones(i, j, board, myPlayer):
        for n in coordinateNeighbors(s[0], s[1], board):
            if board[n[0]][n[1]] == 0:
                liberty.add(n)
    return liberty

def makeMove(i, j, board, myPlayer):#->(list(list()), float, float):
    temp = deepcopy(board)
    temp[i][j] = myPlayer

    oppoPlayer = 1 if myPlayer == 2 else 2
    #print(board)
    oppoDeadStones = deadStonesCoordinates(temp, oppoPlayer)
    #print("dead stones opponents: ",oppoDeadStones)
    for s in oppoDeadStones:
        temp[s[0]][s[1]] = 0
    playerDeadStones = deadStonesCoordinates(temp, myPlayer)
    #print("player stones: ",playerDeadStones)
    for s in playerDeadStones:
        temp[s[0]][s[1]] = 0

    return temp, len(playerDeadStones), len(oppoDeadStones)

def heuristics(t, board)->float:
    #print("Heuristics")
    #go.visualize_board()
    bStones, wStones, vulB, vulW = 0, 6, 0, 0
    for r in range(len(board)):
        for c in range(len(board)):
            if board[r][c] == 1:
                #find liberty of the stone or the stone group 
                Xlib = len(libertyPresent(r,c, board, 1))
                #print(Xlib)
                if Xlib <=1:
                    vulB +=1
                bStones+=1
            elif board[r][c] == 2:
                Olib = len(libertyPresent(r,c, board, 2))
                if Olib <=1:
                    vulW+=1
                wStones+=1
    #print(vulB, vulW)
    if t == 1:
        return (10*bStones) - (10*wStones) + (2*vulW) - (1.5*vulB)
    return (10*wStones) - (10*bStones) + (2*vulB) - (1.5*vulW)


def getPossibleMoves(board, prevBoard, myPlayer):#->(list or None):
    
    moves = set()
    movesToCheck = list()

    for r in range(len(board)):
        for c in range(len(board)):
            if board[r][c] != 0:
                var = (libertyPresent(r, c, board, board[r][c]))
                for v in var:
                    moves.add(v)
    #print(moves)
    for m in moves:
        nextBoard, deadStonesPlayer, deadStonesOpponent = makeMove(m[0], m[1], board, myPlayer)

        if nextBoard != board and nextBoard != prevBoard:
            movesToCheck.append((m[0], m[1], deadStonesOpponent - deadStonesPlayer))
        
    if len(movesToCheck) > 0:
        return sorted(movesToCheck, key=lambda x : -x[2])

    return None


def MAX(board, prevBoard, myPlayer, depth, alpha, beta)->(float, list()):
    #print("######## IN MAX ###########")
    if depth == 0:
        #print(heuristics(myPlayer, board))
        return heuristics(myPlayer, board), []

    oppoPlayer = 1 if myPlayer == 2 else 2
    stones, opStones = 0,0
    for r in range(len(board)):
        for c in range(len(board)):
            if board[r][c]==oppoPlayer:
                opStones+=1
            if board[r][c]==myPlayer:
                stones+=1
    if stones == 0 and opStones == 0:
        return 100, [(2,2)]
    if stones == 0 and opStones == 1:
        if board[2][2] == oppoPlayer:
            return 100, [(2,1)]
        else:
            return 100, [(2,2)]

    moves = getPossibleMoves(board, prevBoard, myPlayer)
    #print(d, moves)
    maxV = float("-inf")
    maxAct = list()
    #print(moves)
     
#     print(d)
#     print("------------------------------>")
#     #go.visualize_board()
#     print("<------------------------------")
    for m in moves:
        #go.set_board(t, pb, cb)
        tempBoard = deepcopy(board)
        nextB, deadStones, opDeadStones = makeMove(m[0],m[1],tempBoard, myPlayer)
        #print(nextB, deadStones, opDeadStones)
        val, act = MIN(nextB, board, oppoPlayer, depth-1, alpha, beta)
        val+=(opDeadStones*5)-(deadStones*8.5)
        
        if val > maxV:
            maxV = val
            maxAct = [m]+act
        if maxV >= beta:
            return maxV, maxAct
        if maxV > alpha:
            alpha = maxV
            
    return maxV, maxAct


def MIN(board, prevBoard, myPlayer, depth, alpha, beta)->(float, list()):
    #print("######## IN MIN ###########")
    if depth == 0:
        return heuristics(myPlayer, board), []

    oppoPlayer = 1 if myPlayer == 2 else 2
    stones, opStones = 0,0
    for r in range(len(board)):
        for c in range(len(board)):
            if board[r][c]==oppoPlayer:
                opStones+=1
            if board[r][c]==myPlayer:
                stones+=1
    if stones == 0 and opStones == 0:
        return 100, [(2,2)]
    if stones == 0 and opStones == 1:
        if cb[2][2] == oppoPlayer:
            return 100, [(2,1)]
        else:
            return 100, [(2,2)]

    moves = getPossibleMoves(board, prevBoard, myPlayer)
    #print(d, moves)
    minV = float("inf")
    minAct = list()
#     print("######## IN MAX ###########")
#     print(d)
#     print("------------------------------>")
#     #go.visualize_board()
#     print("<------------------------------")
    for m in moves:
        #go.set_board(t, pb, cb)
        tempBoard = deepcopy(board)
        nextB, deadStones, opDeadStones = makeMove(m[0],m[1],tempBoard, myPlayer)
        #print(nextB, deadStones, opDeadStones, m)
        val, act = MAX(nextB, board, oppoPlayer, depth-1, alpha, beta)
        val+=(opDeadStones*5)-(deadStones*8.5)
        
        if val < minV:
            minV = val
            minAct = [m]+act
        if minV <= alpha:
            return minV, minAct
        if minV < beta:
            alpha = minV
            
    return minV, minAct

def bestMove(board, prevBoard, myPlayer, depth):
    alpha, beta = float('-inf'), float('inf')
    score, action = MAX(board, prevBoard, myPlayer, depth, alpha, beta)
    if len(action)>0:
        return action[0]
    return "PASS"



t, pb, cb = readInput(5, 'testing.txt')

print(t, cb , pb)

writeOutput(bestMove(cb, pb, t, 2))