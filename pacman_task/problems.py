from game import Actions, Directions
import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


class SingleFoodSearchProblem(SearchProblem):
    def __init__(self, startingGameState):
        # TODO 1
        self.startingGameState = startingGameState
        self.numberOfFoods = startingGameState.getFood().count()

        if self.numberOfFoods != 1:
            print("Multiple food items identified!!!")
            print("Caution: This search agent is designed for solving single-food layouts only!!!")
            print("The agent will only eat one food item and the problem will not terminate!!!")
            print("The search agents for multiple-food problems have the name in format <SearchAlgorithm>MultipleFoodSearchAgent")

        #Because for single food search problem, there is only one food item so pre-find the food here to save up time during the search process
        self.foodItem = (-1, -1)
        foodGrid = startingGameState.getFood()
        for i in range(foodGrid.width):
            for j in range(foodGrid.height):
                if self.startingGameState.hasFood(i, j):
                    self.foodItem = (i, j)
                    break

    def getFoodItem(self):
        return self.foodItem

    def getStartState(self):
        # TODO 2
        return self.startingGameState

    def isGoalState(self, state):
        # TODO 3
        # Is goal state when the number of food decreased by 1
        return state.getFood().count() == self.numberOfFoods - 1

    def getSuccessors(self, state):
        # TODO 4
        # Get the list of legal actions
        actions = state.getLegalPacmanActions()
        # Store the successfors for each action in an array
        successors = [(action, state.generatePacmanSuccessor(action)) for action in actions]
        return successors

    def getCostOfActions(self, actions):
        # TODO 5
        #Because in the getSuccessors method, the problem object only returns the legal actions
        #for a specific location of Pacman. So if an action is added to the path for checking,
        #that path is ensured to be valid. Therefore, for this step, my team just returns the len of the
        #list of actions because each action costs a unit cost.
        return len(actions)


class MultiFoodSearchProblem(SearchProblem):
    def __init__(self, startingGameState):
        # TODO 6
        self.startingGameState = startingGameState
        #Pre-compute the list of food items
        self.foodsList = []
        foodGrid = self.startingGameState.getFood()
        for i in range(foodGrid.width):
            for j in range(foodGrid.height):
                if self.startingGameState.hasFood(i, j):
                    self.foodsList.append((i, j))

    def removeFood(self, food):
        #In case of eaten a food => remove from list of available food
        self.foodsList.remove(food)

    def getFoodsList(self):
        return self.foodsList

    def getStartState(self):
        # TODO 7
        return self.startingGameState

    def isGoalState(self, state):
        # TODO 8
        # For multiple food search problem, the goal state is when there are no food in the maze
        return state.getFood().count() == 0

    def getSuccessors(self, state):
        # TODO 9
         # Get the list of legal actions
        actions = state.getLegalPacmanActions()
        actions.remove(Directions.STOP)
        # Store the successfors for each action in an array
        successors = [(action, state.generatePacmanSuccessor(action)) for action in actions]
        return successors

    def getCostOfActions(self, actions):
        # TODO 10
        #Because in the getSuccessors method, the problem object only returns the legal actions
        #for a specific location of Pacman. So if an action is added to the path for checking,
        #that path is ensured to be valid. Therefore, for this step, my team just returns the len of the
        #list of actions because each action costs a unit cost.
        return len(actions)
