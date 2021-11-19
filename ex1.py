import search
import random
import math
import pickle


ids = ["111111111", "111111111"]

# Objects
DRONES = "drones"
PACKAGES = "packages"
LOCATION = "location"
CLIENTS = "clients"

# Actions
MOVE = "move"
PICK_UP = "pick up"
DELIVER = "deliver"
WAIT = "wait"

class DroneProblem(search.Problem):
    """This class implements a medical problem according to problem description file"""



    def __init__(self, initial):
        """Don't forget to implement the goal test
        You should change the initial to your own representation.
        search.Problem.__init__(self, initial) creates the root node"""
        initial_drones = self.get_initial_drones(initial)
        initial_packages = self.get_initial_packages(initial)
        initial_clients = self.get_initial_clients(initial, initial_packages)

        initial_state = {DRONES: initial_drones,
                         PACKAGES: initial_packages,
                         CLIENTS: initial_clients}
        initial_state_hashable = self.dumps(initial_state)

        search.Problem.__init__(self, initial_state_hashable)
        
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
        state = self.loads(state)
        clients = state[CLIENTS]
        for client in clients:
            if clients[client]:
                return False
        return True

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
            initial_drones[drone] = {LOCATION: drones[drone], PACKAGES: list()}

        return initial_drones

    def get_initial_packages(self, initial):
        packages = initial[PACKAGES]
        initial_packages = {}
        for package in packages:
            initial_packages[package] = {LOCATION: packages[package], CLIENTS: None}

        return initial_packages

    def get_initial_clients(self, initial, initial_packages):
        clients = initial[CLIENTS]
        unwanted_packages = set(initial_packages.keys())
        initial_clients = {}
        for client in clients:
            client_packages = list(clients[client][PACKAGES])
            for package in client_packages:
                if package not in unwanted_packages:
                    pass  # TODO: Determine what to if UNSOLVABLE!
                else:
                    unwanted_packages.remove(package)
                    initial_packages[package][CLIENTS] = client
            initial_clients[client] = client_packages

        for package in unwanted_packages:
            initial_packages.pop(package)

        return initial_clients

    def dumps(self, state):
        return pickle.dumps(state)

    def loads(self, state):
        return pickle.loads(state)

def create_drone_problem(game):
    return DroneProblem(game)

