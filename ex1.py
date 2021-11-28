import itertools

import search
import random
import math
import pickle
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import floyd_warshall


ids = ["111111111", "111111111"]

# Objects
DRONES = "drones"
PACKAGES = "packages"
LOCATION = "location"
CLIENTS = "clients"
MAP = "map"
PATH = "path"

# Actions
MOVE = "move"
PICK_UP = "pick up"
DELIVER = "deliver"
WAIT = "wait"

# Map
PASSABLE = "P"
IMPASSABLE = "I"


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

        self.map = initial[MAP]
        self.client_paths = self.get_clients_paths(initial)
        self.current_round = 0

        graph = self.create_map_graph(self.map)
        self.dist_matrix = self.create_dist_matrix(graph)

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
        actions = list(filter(self.filter_duplicate_pickups,
                              itertools.product(*atomic_actions)))
        # self.current_round += 1
        return actions

    def filter_duplicate_pickups(self, action):
        package_actions = []
        for atomic_action in action:
            if atomic_action[0] == PICK_UP:
                package_actions.append(atomic_action[2])
        return len(package_actions) == len(set(package_actions))

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""

        new_state = self.loads(state)

        for atomic_action in action:
            act = atomic_action[0]
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
                new_state[CLIENTS][client][PACKAGES].remove(
                    package)  # Add here packages

        self.update_clients_location(new_state)

        return self.dumps(new_state)

    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""
        dict_state = self.loads(state)
        clients = dict_state[CLIENTS]
        for client in clients:
            if clients[client][PACKAGES]:  # Add here packages
                return False
        return True

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
        state can be accessed via node.state)
        and returns a goal distance estimate"""

        # h = 0
        # state_dict = self.loads(node.state)
        # drones = state_dict[DRONES]
        # for drone in drones:
        #     packages = state_dict[DRONES][drone][PACKAGES]
        #     if packages == [] and self.package_exists(state_dict):
        #         h += self.shortest_distance_from_package(drone, state_dict)
        #     else:
        #         h += self.shortest_distance_from_client(drone, state_dict)
        # return h

        # count = 0
        # dict_state = self.loads(node.state)
        # for client in dict_state[CLIENTS]:
        #     count += len(dict_state[CLIENTS][client][PACKAGES])
        # return count + node.depth
        return self.objects_distance_sum_h(node) + node.depth

    def objects_distance_sum_h(self, node):
        state_dict = self.loads(node.state)
        packages = state_dict[PACKAGES]
        objects_locations = [packages[package][LOCATION] for package in packages if not isinstance(packages[package][LOCATION], str)]

        clients = state_dict[CLIENTS]
        # clients_locations = []
        for client in clients:
            if clients[client][PACKAGES]:
                clients_index = clients[client][LOCATION]
                client_path = self.client_paths[client]
                objects_locations.append(client_path[0])


        objects_locations_indexes = [self.convert_tuple_to_index(loc) for loc in objects_locations]

        sum = 0
        drones = state_dict[DRONES]
        for drone in drones:
            drone_location = drones[drone][LOCATION]
            drone_location_index = self.convert_tuple_to_index(drone_location)
            sum += self.distance_sum_from_location(drone_location_index, objects_locations_indexes)

        return sum

    def distance_sum_from_location(self, location, locations_list):
        sum = 0
        for index in locations_list:
            sum += self.dist_matrix[location][index]
        return sum


    def package_exists(self, state_dict):
        packages = state_dict[PACKAGES]
        for package in packages:
            if not isinstance(packages[package][LOCATION], str):
                return True
        return False

    def shortest_distance_from_package(self, drone, state_dict):
        drone_location = state_dict[DRONES][drone][LOCATION]
        package_locations = []
        for package in state_dict[PACKAGES]:
            package_location = state_dict[PACKAGES][package][LOCATION]
            if not isinstance(package, str):
                package_locations.append(package_location)
        if package_locations == []:
            return 0
        package_distances = [np.linalg.norm(np.array(drone_location) -
                                            np.array(package_location)) for
                             package_location in package_locations]
        return min(package_distances)

    def shortest_distance_from_client(self, drone, state_dict):
        drone_location = state_dict[DRONES][drone][LOCATION]
        client_locations = []
        for client in state_dict[CLIENTS]:
            location_index = state_dict[CLIENTS][client][LOCATION]
            client_path = self.client_paths[client]
            client_location = client_path[location_index % len(client_path)]
            client_locations.append(client_location)
        client_distances = [np.linalg.norm(np.array(drone_location) -
                                           np.array(client_location)) for
                            client_location in client_locations]
        return min(client_distances)

    def get_clients_paths(self, initial):
        initial_clients = initial[CLIENTS]
        clients_path = {}
        for client in initial_clients:
            clients_path[client] = initial_clients[client][PATH]
        return clients_path

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
            initial_packages[package] = {LOCATION: packages[package]}

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
            initial_clients[client] = {PACKAGES: client_packages,
                                       LOCATION: 0}

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

        clients_on_current_location = self.clients_on_current_location(
            location, state_dict)

        for client in clients_on_current_location:
            package_tuple = state_dict[CLIENTS][client][
                PACKAGES]  # Add here packages
            for package in package_tuple:
                if package in drone_packages:
                    deliver_actions.append((DELIVER, drone, client, package))
        return deliver_actions

    def get_pickup_atomic_actions(self, drone_data, state_dict):
        """ Get all the possible atomic actions the relate to package pickup
        for a specific drone
        :return: List that contains every possible package pickup atomic
        action for the drone
        """
        drone, location, drone_packages = drone_data
        pickup_actions = []

        if len(drone_packages) < 2:
            packages_on_current_location = self.packages_on_current_location(
                location, state_dict)
            for package in packages_on_current_location:
                pickup_actions.append((PICK_UP, drone, package))
        return pickup_actions

    def clients_on_current_location(self, location, state_dict):
        """ Get all the clients that currently in a specific location in the map
        :return: List of the clients
        """
        clients_list = []
        for client in self.client_paths:
            client_path = self.client_paths[client]
            position_on_path = state_dict[CLIENTS][client][LOCATION]
            client_location = client_path[position_on_path]
            if client_location == location:
                clients_list.append(client)
        return clients_list

    def packages_on_current_location(self, location, state_dict):
        package_list = []
        for package in state_dict[PACKAGES]:
            if state_dict[PACKAGES][package][LOCATION] == location:
                package_list.append(package)

        return package_list

    def update_clients_location(self, state_dict):
        clients = state_dict[CLIENTS]
        for client in clients:
            path_length = len(self.client_paths[client])
            old_index = clients[client][LOCATION]
            new_index = (old_index + 1) % path_length
            clients[client][LOCATION] = new_index
            # print(f"Turn index of {client}: {old_index} -> {new_index}")

    def create_map_graph(self, problem_map):
        m = len(problem_map)
        n = len(problem_map[0])
        # matrix = [[0 for _ in range(m * n)] for _ in range(m * n)]
        row = []
        col = []
        data = []
        for i in range(m):
            for j in range(n):
                index = self.convert_tuple_to_index((i, j))
                if i < m - 1 and problem_map[i + 1][j] == PASSABLE:
                    lower_index = index + n
                    row.append(index)
                    col.append(lower_index)
                    data.append(1)
                if i > 0 and problem_map[i - 1][j] == PASSABLE:
                    upper_index = index - n
                    row.append(index)
                    col.append(upper_index)
                    data.append(1)
                if j < n - 1 and problem_map[i][j + 1] == PASSABLE:
                    right_index = index + 1
                    row.append(index)
                    col.append(right_index)
                    data.append(1)
                if j > 0 and problem_map[i][j - 1] == PASSABLE:
                    left_index = index - 1
                    row.append(index)
                    col.append(left_index)
                    data.append(1)
        return csr_matrix((data, (row, col)), shape=(m * n, m * n))

    def create_dist_matrix(self, graph):
        dist_matrix, predecessors = floyd_warshall(csgraph=graph, directed=True, return_predecessors=True, unweighted=False)
        return dist_matrix

    def convert_index_to_tuple(self, index, m, n):
        return index // n, index - (index // n) * m

    def convert_tuple_to_index(self, t):
        # m = len(self.map)
        n = len(self.map[0])
        return t[0] * n + t[1]



def create_drone_problem(game):
    return DroneProblem(game)
#
#
#
# def convert_tuple_to_index(t, m, n):
#     return t[0] * n + t[1]

# def create_map_graph(problem_map):
#     m = len(problem_map)
#     n = len(problem_map[0])
#     matrix = [[0 for _ in range(m * n)] for _ in range(m * n)]
#     for i in range(m):
#         for j in range(n):
#             index = convert_tuple_to_index((i, j), m, n)
#             if i < m - 1 and problem_map[i + 1][j] == PASSABLE:
#                 lower_index = index + n
#                 matrix[index][lower_index] = 1
#             if i > 0 and problem_map[i - 1][j] == PASSABLE:
#                 upper_index = index - n
#                 matrix[index][upper_index] = 1
#             if j < n - 1 and problem_map[i][j + 1] == PASSABLE:
#                 right_index = index + 1
#                 matrix[index][right_index] = 1
#             if j > 0 and problem_map[i][j - 1] == PASSABLE:
#                 left_index = index - 1
#                 matrix[index][left_index] = 1
#     return csr_matrix(matrix)
#
#
# def create_map_graph2(problem_map):
#     m = len(problem_map)
#     n = len(problem_map[0])
#     # matrix = [[0 for _ in range(m * n)] for _ in range(m * n)]
#     row = []
#     col = []
#     data = []
#     for i in range(m):
#         for j in range(n):
#             index = convert_tuple_to_index((i, j), m, n)
#             if i < m - 1 and problem_map[i + 1][j] == PASSABLE:
#                 lower_index = index + n
#                 row.append(index)
#                 col.append(lower_index)
#                 data.append(1)
#             if i > 0 and problem_map[i - 1][j] == PASSABLE:
#                 upper_index = index - n
#                 row.append(index)
#                 col.append(upper_index)
#                 data.append(1)
#             if j < n - 1 and problem_map[i][j + 1] == PASSABLE:
#                 right_index = index + 1
#                 row.append(index)
#                 col.append(right_index)
#                 data.append(1)
#             if j > 0 and problem_map[i][j - 1] == PASSABLE:
#                 left_index = index - 1
#                 row.append(index)
#                 col.append(left_index)
#                 data.append(1)
#     return csr_matrix((data, (row, col)), shape=(m * n, m * n))

# def construct_path(pre, start, end):
#     path = [end]
#     current = end
#     while current != start:
#         if current == -9999:
#             return []
#         current = pre[current]
#         path.append(current)
#     path.reverse()
#     return path
#
# def construct_paths(pre):
#     d = {}
#     for i in range(len(pre)):
#         for j in range(len(pre[0])):
#             d[(i, j)] = construct_path(pre[i], i, j)
#     return d
#
# if __name__ == '__main__':
#     p_map =    [['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'I', 'I', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'I', 'I', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'I', 'I', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'I', 'I', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P']]
#     # p_map = [['P', 'P', 'P', 'P'],
#     #          ['P', 'P', 'P', 'P'],
#     #          ['P', 'I', 'P', 'P'],
#     #          ['P', 'P', 'P', 'P'], ]
#     p_map = [['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'I', 'P'],
#                 ['P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'I', 'I', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P'],
#                 ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P']]
#     print(f"{len(p_map)} by {len(p_map[0])}")
#     import time
#     cur = time.time()
#     graph1 = create_map_graph(p_map)
#     dist_matrix1, predecessors1 = floyd_warshall(csgraph=graph1, directed=True, return_predecessors=True)
#     graph2 = create_map_graph2(p_map)
#     dist_matrix2, predecessors2 = floyd_warshall(csgraph=graph2, directed=True, return_predecessors=True, unweighted=False)
#     print(time.time() - cur)
#     # print((dist_matrix1 == dist_matrix2).all() and (predecessors1 == predecessors2).all())
#     # print(list(predecessors2[0]))
#
#     p = predecessors2
#     pp = [list(a) for a in p]
#     # for a in pp:
#     #     print(a)
#     # x = reconstruct_path(graph2, predecessors2[0])
#     x = construct_paths(p).items()
#     # for k, v in x:
#     #     print(f"{k}: {v}")
#     # print(x)

