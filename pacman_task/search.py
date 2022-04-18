"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from game import Directions
import util

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 17
    start = problem.getStartState()
    stack = util.Stack()
    visited = []
    visited.append(start.getPacmanPosition())
    curNode = (start, [])
    stack.put(curNode)

    while not stack.is_empty():
        curNode = stack.pop()
        pos = curNode[0].getPacmanPosition()
        if pos not in visited:
            visited.append(pos)
        successors = problem.getSuccessors(curNode[0])
        for s in successors:
            nextNode = s[1]
            curPath = curNode[1].copy()
            curPath.append(s[0])
            if problem.isGoalState(nextNode):
                curNode = (nextNode, curPath)
                print('Path cost: %d' % len(curNode[1]))
                return curNode[1]
            if nextNode.getPacmanPosition() not in visited:
                stack.put((nextNode, curPath))

    print('No path found!!!')
    return []
    

def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 18
    state = (problem.getStartState(), [])
    frontier = util.Queue()
    frontier.enqueue(state)
    explored = []
    
    while not frontier.is_empty():
        state = frontier.dequeue()
        explored.append(state[0].getPacmanPosition())

        #Duyệt qua từng trạng thái con
        s = problem.getSuccessors(state[0])
        for i in range(len(s)):
            nextState = s[i]
            loc = nextState[1].getPacmanPosition()
            if problem.isGoalState(nextState[1]):
                curNode = (nextState[1], state[1] + [nextState[0]])
                #Trả về đường đi sau khi tìm kiếm
                print('Path cost: %d' % len(curNode[1]))
                return curNode[1]
            if loc not in explored:
                frontier.enqueue((nextState[1], state[1] + [nextState[0]]))
                explored.append(loc)

    #Thoát vòng lặp khi chưa return => Trường hợp không có thức đường đi
    print('No path found!!!')
    return []


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    start = problem.getStartState()
    frontier = util.PriorityQueue()
    frontier.push((start, []), 0)
    explored = []

    isValidReturn = False

    while not frontier.isEmpty():
        _, _, (state, path) = frontier.pop()
        if problem.isGoalState(state):
            isValidReturn = True
            break
        pacman = state.getPacmanPosition()

        if pacman not in explored:
            explored.append(pacman)
            for child in problem.getSuccessors(state):
                childPos = child[1].getPacmanPosition()
                if childPos not in explored:
                    accumalatedPath = path.copy()
                    action = child[0]
                    accumalatedPath = accumalatedPath + [action]
                    frontier.update((child[1], accumalatedPath), problem.getCostOfActions(accumalatedPath))

    if isValidReturn:
        #Path exist
        print('Path cost: %d' % len(path))
        return path
    else:
        print('No path found!!!')
        return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def singleFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of single food search
    """
    # TODO 20
    # Manhattan distance between current position of Pacman and the food
    pacman = state.getPacmanPosition()
    return util.manhattanDistance(pacman, problem.getFoodItem())


def multiFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of multi-food search
    """
    foodList = problem.getFoodsList()
    currentPosition = state.getPacmanPosition()
    foodNotVisited = [True] * len(foodList)
    heuristic = 0

    if len(foodList) == 0:
        return heuristic

    #Loop until all the food are examined
    while foodNotVisited.count(True) == 0:
        #For each time, find the path from the current position to the nearest food
        min_distance = float('inf')
        nearest_food = -1

        #Find the nearest food
        for index, food in enumerate(foodList):
            distance = util.manhattanDistance(currentPosition, food)
            if distance < min_distance and foodNotVisited[index]:
                min_distance = distance
                nearest_food = index

        #Add up the Manhattan distance from the current location to the nearest food
        heuristic += min_distance
        foodNotVisited[nearest_food] = False

        #Reset the current location to the location of the just eaten food
        currentPosition = foodList[nearest_food]

    return heuristic
    

def aStarSearch(problem, heuristic=singleFoodSearchHeuristic):
    '''
    return a path to the goal
    '''
    # TODO 22
    currentState = (problem.getStartState(), [])

    #Initialize
    frontier = util.PriorityQueueWithFunction(heuristic, problem)
    frontier.push(currentState, 0)
    currentState = frontier.getQueue()[0]

    #List of visited nodes
    explored = []

    isValidReturn = False

    while not frontier.isEmpty():
        currentState = frontier.pop()

        getState = currentState[2][0]
        if problem.isGoalState(getState):
            isValidReturn = True
            break

        if getState.getPacmanPosition() not in explored:
            for successor in problem.getSuccessors(getState):
                nextLocation = successor[1].getPacmanPosition()
                if nextLocation not in explored:
                    path = currentState[2][1] + [successor[0]]
                    frontier.update((successor[1], path), problem.getCostOfActions(path))
            explored.append(getState.getPacmanPosition())

    if isValidReturn:
        print('Path cost: %d' % len(currentState[2][1]))
        return currentState[2][1]
    else:
        print('No path found!!!')
        return []


def breadthFirstSearchMultipleFood(problem):
    state = (problem.getStartState(), [])
    
    #Lấy mảng thức ăn
    foodState = state[0].getFood()

    #Mảng lưu kết quả đường đi trả về
    travelled = []

    #Khi còn thức ăn
    foodRemaining = foodState.count()
    while foodRemaining != 0:
        frontier = util.Queue()
        frontier.enqueue(state)
        explored = []

        isValidReturn = False

        breakFromSub = False 

        while not frontier.is_empty():
            if breakFromSub:
                break
            
            state = frontier.dequeue()
            currentPosition = state[0].getPacmanPosition()
            explored.append(currentPosition)

            if problem.isGoalState(state[0]):
                isValidReturn = True
                break

            #Thoát vòng lặp để tìm kiếm thức ăn tiếp theo khi đến vị trí của 1 điểm thức ăn
            if foodState[currentPosition[0]][currentPosition[1]]:
                isValidReturn = True
                break

            #Duyệt qua từng trạng thái con
            s = problem.getSuccessors(state[0])
            for i in range(len(s)):
                nextState = s[i]
                loc = nextState[1].getPacmanPosition()
                if foodState[loc[0]][loc[1]]:
                    #Next successor is an action that reaches a food iteam
                    #We break the subproblem and add up the path, move to the next subproblem
                    breakFromSub = True
                    state = (nextState[1], state[1] + [nextState[0]])
                    isValidReturn = True
                    break
                if loc not in explored:
                    frontier.enqueue((nextState[1], state[1] + [nextState[0]]))
                    explored.append(loc)

        if not isValidReturn:
            #Trường hợp không có đường đi
            print('No path found that can eat all the food!!!')
            return [] 

        #Khi thoát vòng lặp, tức tìm kiếm được 1 điểm thức ăn mới
        #Cộng đường đi đã đi vào mảng kết quả để trả về sau
        travelled += state[1]

        #Cập nhật lại trạng thái tìm kiếm để tìm tiếp điểm thức ăn sau
        state = (state[0], [])
        pos = state[0].getPacmanPosition()
        foodState[pos[0]][pos[1]] = False
        foodRemaining = foodState.count()

    print('Path cost: %d' % len(travelled))
    return travelled


def depthFirstSearchMultipleFood(problem):
    start = problem.getStartState()
    curNode = (start, [])
    food = curNode[0].getFood()
    result = []
    count = 0
    while food.count() > 0:
        stack = util.Stack()
        stack.put(curNode)
        visited = []
        pos = curNode[0].getPacmanPosition()
        visited.append(pos)

        #This variable to identify if the loop is break after reaching a food or cannot find any valid path
        isValidReturn = False

        #This varialbe is to break the outer loop in case the inner if function find the successor reached the stop condition
        #for subproblem (reaches a food)
        breakFromSub = False 

        while not stack.is_empty():
            if breakFromSub:
                break
            curNode = stack.pop()
            pos = curNode[0].getPacmanPosition()
            if problem.isGoalState(curNode[0]):
                isValidReturn = True
                break
            if food[pos[0]][pos[1]]:
                isValidReturn = True
                break
            if pos not in visited:
                visited.append(pos)
            successors = problem.getSuccessors(curNode[0])
            count += 1
            for s in successors:
                nextNode = s[1]
                curPath = curNode[1].copy()
                curPath.append(s[0])
                nextPos = nextNode.getPacmanPosition()
                if food[nextPos[0]][nextPos[1]]:
                    #Next successor is an action that reaches a food iteam
                    #We break the subproblem and add up the path, move to the next subproblem
                    breakFromSub = True
                    curNode = (nextNode, curPath)
                    isValidReturn = True
                    break
                if problem.isGoalState(nextNode):
                    print('Path cost: %d' % len(result))
                    result += curPath
                    return result
                if nextNode.getPacmanPosition() not in visited:
                    stack.put((nextNode, curPath))

        if not isValidReturn:
            #Trường hợp không có đường đi
            print('No path found that can eat all the food!!!')
            return [] 

        result += curNode[1]
        curNode = (curNode[0], [])
        food[pos[0]][pos[1]] = False

    print('Path cost: %d' % len(result))
    return result


def uniformCostSearchMultipleFood(problem):
    start = problem.getStartState()
    s = (start, [])
    getFood = start.getFood()
    accumulatedPath = []

    while getFood.count() > 0:
        frontier = util.PriorityQueue()
        frontier.push(s, 0)
        explored = []

        isValidReturn = False

        while not frontier.isEmpty():
            _, _, (state, path) = frontier.pop()
            if problem.isGoalState(state):
                isValidReturn = True
                break
            
            pacman = state.getPacmanPosition()

            if getFood[pacman[0]][pacman[1]]:
                isValidReturn = True
                break

            if pacman not in explored:
                explored.append(pacman)
                for child in problem.getSuccessors(state):
                    childPos = child[1].getPacmanPosition()
                    if childPos not in explored:
                        accPath = path.copy()
                        action = child[0]
                        accPath = accPath + [action]
                        frontier.push((child[1], accPath), problem.getCostOfActions(accPath))

        if not isValidReturn:
            #Trường hợp không có đường đi
            print(frontier.getQueue())
            print('No path found that can eat all the food!!!')
            return [] 
            
        accumulatedPath += path.copy()
        s = (state, [])
        foodPos = state.getPacmanPosition()
        getFood[foodPos[0]][foodPos[1]] = False

    print('Path cost: %d' % len(accumulatedPath))
    return accumulatedPath


def aStarSearchMultipleFood(problem, heuristic=multiFoodSearchHeuristic):
    currentState = (problem.getStartState(), [])
    foodGrid = currentState[0].getFood()

    #This array store the path to all the food
    path = []

    #Loop when there are still food to eat
    while foodGrid.count() > 0:
        #Initialize
        frontier = util.PriorityQueueWithFunction(heuristic, problem)
        frontier.push(currentState, 0)
        currentState = frontier.getQueue()[0]

        #List of visited nodes
        explored = []

        isValidReturn = False

        while not frontier.isEmpty():
            currentState = frontier.pop()

            getState = currentState[2][0]
            
            if problem.isGoalState(getState):
                isValidReturn = True
                break

            if (foodGrid[getState.getPacmanPosition()[0]][getState.getPacmanPosition()[1]]):
                problem.removeFood(getState.getPacmanPosition())
                isValidReturn = True
                break

            if getState.getPacmanPosition() not in explored:
                for successor in problem.getSuccessors(getState):
                    nextLocation = successor[1].getPacmanPosition()
                    if nextLocation not in explored:
                        pathList = currentState[2][1] + [successor[0]]
                        frontier.update((successor[1], pathList), problem.getCostOfActions(pathList))
                explored.append(getState.getPacmanPosition())

        if not isValidReturn:
            #Trường hợp không có đường đi
            print('No path found that can eat all the food!!!')
            return [] 

        path += currentState[2][1]
        foodGrid[currentState[2][0].getPacmanPosition()[0]][currentState[2][0].getPacmanPosition()[1]] = False
        currentState = (currentState[2][0], [])

    print('Path cost: %d' % len(path))
    return path


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
bfsAll = breadthFirstSearchMultipleFood
dfsAll = depthFirstSearchMultipleFood
ucsAll = uniformCostSearchMultipleFood
astarAll = aStarSearchMultipleFood