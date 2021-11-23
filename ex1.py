import itertools

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
MAP = "map"
ROUND = "round"

# Actions
MOVE = "move"
PICK_UP = "pick up"
DELIVER = "deliver"
WAIT = "wait"

# Map
PASSABLE = "passable"
IMPASSABLE = "impassable"


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
                         CLIENTS: initial_clients,
                         ROUND: 0}  # TODO: check if should be in state
        initial_state_hashable = self.dumps(initial_state)

        self.map = initial[MAP]
        self.client_paths = initial[CLIENTS]

        search.Problem.__init__(self, initial_state_hashable)

    def actions(self, state):
        """Returns all the actions that can be executed in the given
        state. The result should be a tuple (or other iterable) of actions
        as defined in the problem description file"""

        state_dict = self.loads(state)
        drones = state_dict[DRONES]
        atomic_actions = []

        for drone in drones:
            drone_atomic_actions = self.get_atomic_actions(drone, state_dict)
            atomic_actions.append(drone_atomic_actions)

        return list(itertools.product(*atomic_actions))

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""

        new_state = self.loads(state)

        for atomic_action in action:
            act = action[0]  # ?: Isn't it supposed to be atomic_action[0]?
            drone = atomic_action[1]
            if act == MOVE:
                new_location = atomic_action[2]
                new_state[DRONES][drone][LOCATION] = new_location
            elif act == PICK_UP:
                package = atomic_action[2]
                new_state[DRONES][drone][PACKAGES].append(package)
                new_state[PACKAGES][package][
                    LOCATION] = drone  # TODO: Check if OK
            elif act == DELIVER:
                client = atomic_action[2]
                package = atomic_action[3]
                new_state[DRONES][drone][PACKAGES].remove(package)
                new_state[PACKAGES][package][
                    LOCATION] = DELIVER  # TODO: Check if OK (Maybe just delete the package)
                new_state[CLIENTS][client].remove(package)

        return self.dumps(new_state)

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
            initial_packages[package] = {LOCATION: packages[package],
                                         CLIENTS: None}

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

    def get_atomic_actions(self, drone, state_dict):
        """ Get all the possible atomic actions for a specific drone
        :return: Tuple that contains every possible atomic action for the  drone
        """
        drone_object = state_dict[DRONES][drone]
        drone_location = drone_object[LOCATION]
        drone_packages = drone_object[PACKAGES]
        drone_data = [drone, drone_location, drone_packages]

        possible_atomic_actions = [(WAIT, drone)]
        possible_atomic_actions.extend(
            self.get_move_atomic_actions(drone_data))
        possible_atomic_actions.extend(
            self.get_deliver_atomic_actions(drone_data, state_dict))
        possible_atomic_actions.extend(self.get_pickup_atomic_actions(
            drone_data, state_dict))

        return tuple(possible_atomic_actions)

    def get_move_atomic_actions(self, drone_data):
        """ Get all the possible atomic actions the relate to movement for a
        specific drone
        :return: List that contains every possible move atomic action for the
        drone
        """
        drone = drone_data[0]
        location = drone_data[1]
        x = location[0]
        y = location[1]

        move_actions = []
        map_size_x = len(self.map)
        # TODO: make sure we reject a problem  with an empty map
        map_size_y = len(self.map[0])

        if x > 0 and self.map[x - 1][y] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0] - 1, location[1])))

        if x < map_size_x - 1 and self.map[x + 1][y] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0] + 1, location[1])))

        if y > 0 and self.map[x][y - 1] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0], location[1] - 1)))

        if y < map_size_y - 1 and self.map[x][y + 1] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0], location[1] + 1)))

        return move_actions

    def get_deliver_atomic_actions(self, drone_data, state_dict):
        """ Get all the possible atomic actions that relate to package delivery
        for a specific drone
        :return: List that contains every possible package delivery atomic
        action for the drone
        """
        drone, location, drone_packages = drone_data
        deliver_actions = []
        current_round = state_dict[ROUND]

        clients_on_current_location = self.clients_on_current_location(
            location, current_round)

        for client in clients_on_current_location:
            package_tuple = state_dict[CLIENTS][client][PACKAGES]
            for package in package_tuple:
                if package in drone_packages:
                    deliver_actions.append(drone, DELIVER, client, package)
        return deliver_actions

    def get_pickup_atomic_actions(self, drone_data, state_dict):
        """ Get all the possible atomic actions the relate to package pickup
        for a specific drone
        :return: List that contains every possible package pickup atomic
        action for the drone
        """
        drone, location, drone_packages = drone_data
        pickup_actions = []

        packages_on_current_location = self.packages_on_current_location(
            location, state_dict)
        for package in packages_on_current_location:
            if package in drone_packages:
                pickup_actions.append((PICK_UP, drone, package))
        return pickup_actions

    def clients_on_current_location(self, location, current_round):
        """ Get all the clients that currently in a specific location in the map
        :return: List of the clients
        """
        clients_list = []
        for client in self.client_paths:
            client_path = self.client_paths[client_path]
            position_on_path = current_round % (len(client_path) - 1)
            client_location = client_path[position_on_path]
            if client_location == location:
                clients_list.append(client)
        return clients_list

    def packages_on_current_location(self, location, state_dict):
        package_list = []
        for package in state_dict[PACKAGES]:
            if state_dict[PACKAGES][package] == location:
                package_list.append(package)

        return package_list


def create_drone_problem(game):
    return DroneProblem(game)
