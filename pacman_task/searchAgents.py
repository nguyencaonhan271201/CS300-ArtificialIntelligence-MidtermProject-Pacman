import random
import search
import problems
import time

from game import Agent
from game import Directions

class GoWestAgent(Agent):
    def getAction(self, state):
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


class RandomAgent(Agent):
    def getAction(self, state):
        actions = state.getLegalPacmanActions()
        random.shuffle(actions)
        return actions[0]


class TestAgent(Agent):
    def __init__(self):
        self.actionsHistory = []
        self.isBacktracking = False

    def getReverseAction(self, action):
        if action == Directions.EAST:
            return Directions.WEST
        elif action == Directions.WEST:
            return Directions.EAST
        elif action == Directions.NORTH:
            return Directions.SOUTH
        elif action == Directions.SOUTH:
            return Directions.NORTH
        pass

    def getAction(self, state):
        if len(self.actionsHistory) == 10:
            self.isBacktracking = True

        if not self.isBacktracking or len(self.actionsHistory) == 0:
            self.isBacktracking = False
            actions = state.getLegalPacmanActions()
            self.actionsHistory.append(actions[0])
        else:
            return self.getReverseAction(self.actionsHistory.pop())

        return self.actionsHistory[-1]


class SearchAgent(Agent):
    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        start = time.time()
        self.problem = self.problemType(state)
        self.path = self.searchAlgorithm(self.problem)
        self.action = None
        print('Solution founded after %.1f seconds.' % (time.time() - start))

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        # TODO 12

        if self.action is None:
            self.action = 0
        else:
            self.action += 1

        if self.action >= len(self.path):
            return Directions.STOP
        
        getAction = self.path[self.action]
        if getAction == 'East':
            return Directions.EAST
        elif getAction == 'West':
            return Directions.WEST
        elif getAction == 'North':
            return Directions.NORTH
        elif getAction == 'South':
            return Directions.SOUTH

#Those agents solve the single-food problems
#Caution: Passing a multiple food layout to this agent will display a warning and the agent will only eat one food
class BFSFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchAlgorithm = search.bfs
        self.problemType = problems.SingleFoodSearchProblem
    pass


class DFSFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchAlgorithm = search.dfs
        self.problemType = problems.SingleFoodSearchProblem
    pass


class UCSFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchAlgorithm = search.ucs
        self.problemType = problems.SingleFoodSearchProblem
    pass


class AStarFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchAlgorithm = search.astar
        self.problemType = problems.SingleFoodSearchProblem
    pass


#Those agents solve the multiple food problems
class BFSMultipleFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchAlgorithm = search.bfsAll
        self.problemType = problems.MultiFoodSearchProblem
    pass


class DFSMultipleFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchAlgorithm = search.dfsAll
        self.problemType = problems.MultiFoodSearchProblem
    pass


class UCSMultipleFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchAlgorithm = search.ucsAll
        self.problemType = problems.MultiFoodSearchProblem
    pass


class AStarMultipleFoodSearchAgent(SearchAgent):
    def __init__(self):
        self.searchAlgorithm = search.astarAll
        self.problemType = problems.MultiFoodSearchProblem
    pass