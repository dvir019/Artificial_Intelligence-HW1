import search
import random
import math


ids = ["111111111", "111111111"]
DRONES = "drones"
PACKAGES = "packages"
LOCATION = "location"
CLIENTS = "clients"


class DroneProblem(search.Problem):
    """This class implements a medical problem according to problem description file"""



    def __init__(self, initial):
        """Don't forget to implement the goal test
        You should change the initial to your own representation.
        search.Problem.__init__(self, initial) creates the root node"""
        print("----------------------------------------------------------------")
        print(self.get_initial_drones(initial))
        search.Problem.__init__(self, initial)
        
    def actions(self, state):
        """Returns all the actions that can be executed in the given
        state. The result should be a tuple (or other iterable) of actions
        as defined in the problem description file"""

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""

    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
        state can be accessed via node.state)
        and returns a goal distance estimate"""
        return 0

    """Feel free to add your own functions
    (-2, -2, None) means there was a timeout"""

    def get_initial_drones(self, initial):
        drones = initial[DRONES]
        initial_drones = {}
        for drone in drones:
            initial_drones[drone] = {LOCATION: drones[drone], PACKAGES: set()}

        return initial_drones

    def get_initial_packages(self, initial):
        packages = initial[PACKAGES]
        initial_packages = {}
        for package in packages:
            initial_packages[package] = {LOCATION: packages[package], CLIENTS: None}

    def get_client_by_package(self, package, ):


def create_drone_problem(game):
    return DroneProblem(game)

