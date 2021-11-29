import ex1
import search
import time


def timeout_exec(func, args=(), kwargs={}, timeout_duration=10, default=None):
    """This function will spawn a thread and run the given function
    using the args, kwargs and return the given default value if the
    timeout_duration is exceeded.
    """
    import threading
    class InterruptableThread(threading.Thread):
        def __init__(self):
            threading.Thread.__init__(self)
            self.result = default

        def run(self):
            # try:
            self.result = func(*args, **kwargs)
            # except Exception as e:
            #    self.result = (-3, -3, e)

    it = InterruptableThread()
    it.start()
    it.join(timeout_duration)
    if it.is_alive():
        return default
    else:
        return it.result


def check_problem(p, search_method, timeout):
    """ Constructs a problem using ex1.create_wumpus_problem,
    and solves it using the given search_method with the given timeout.
    Returns a tuple of (solution length, solution time, solution)"""

    """ (-2, -2, None) means there was a timeout
    (-3, -3, ERR) means there was some error ERR during search """

    t1 = time.time()
    s = timeout_exec(search_method, args=[p], timeout_duration=timeout)
    t2 = time.time()

    if isinstance(s, search.Node):
        solve = s
        solution = list(map(lambda n: n.action, solve.path()))[1:]
        return (len(solution), t2 - t1, solution)
    elif s is None:
        return (-2, -2, None)
    else:
        return s


def solve_problems(problems):
    solved = 0
    for problem in problems:
        try:
            p = ex1.create_drone_problem(problem)
        except Exception as e:
            print("Error creating problem: ", e)
            return None
        timeout = 60
        result = check_problem(p, (lambda p: search.best_first_graph_search(p, p.h)), timeout)
        print("GBFS ", result)
        if result[2] != None:
            if result[0] != -3:
                solved = solved + 1


def main():
    print(ex1.ids)
    """Here goes the input you want to check"""
    problems = [
        {
            "map": [['P', 'P'], ],
            "drones": {'drone 1': (0, 1)},
            "packages": {'package 1': (0, 1)},
            "clients": {'Yossi': {"path": [(0, 1), (0, 0)],
                                  "packages": ('package 1', )}}
        },
        {
            "map": [['P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'P'], ],
            "drones": {'drone 1': (3, 3)},
            "packages": {'package 1': (0, 2),
                         'package 2': (2, 0)},
            "clients": {'Yossi': {"path": [(0, 1), (1, 1), (1, 0), (0, 0)],
                                  "packages": ('package 1', 'package 2')}}
        },

        # {
        #     "map": [['P', 'P', 'P', 'P'],
        #             ['P', 'P', 'P', 'P'],
        #             ['P', 'I', 'P', 'P'],
        #             ['P', 'P', 'P', 'P'], ],
        #     "drones": {'drone 1': (3, 3)},
        #     "packages": {'package 1': (2, 1),
        #                  'package 2': (2, 0)},
        #     "clients": {'Yossi': {"path": [(0, 1), (1, 1), (1, 0), (0, 0)],
        #                           "packages": ('package 1', 'package 2')}}
        # },
        #
        # {
        #     "map": [['P', 'P', 'P', 'P'],
        #             ['P', 'P', 'P', 'P'],
        #             ['P', 'I', 'P', 'I'],
        #             ['P', 'P', 'P', 'P'], ],
        #     "drones": {'drone 1': (3, 3),
        #                'drone 2': (1, 0)},
        #     "packages": {'package 1': (0, 3),
        #                  'package 2': (3, 0),
        #                  'package 3': (2, 1)},
        #     "clients": {'Sarah': {"path": [(0, 2), (2, 2), (2, 0), (0, 0)],
        #                           "packages": ('package 1', 'package 2', 'package 3')}}
        # },

        {
            "map": [['P', 'P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'I', 'P'],
                    ['P', 'P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'I', 'P'],
                    ['P', 'P', 'P', 'P', 'P'], ],
            "drones": {'drone 1': (3, 3),
                       'drone 2': (1, 0)},
            "packages": {'package 1': (0, 4),
                         'package 2': (3, 2),
                         'package 3': (2, 1),
                         'package 4': (2, 4), },
            "clients": {'Alice': {"path": [(0, 1), (1, 1), (1, 0), (0, 0), (2, 2)],
                                  "packages": ('package 1', 'package 2', 'package 3')},
                        'Bob': {"path": [(4, 3), (2, 2), (4, 2), (4, 4)],
                                "packages": ('package 4', )},
                        }
        },
        {
            "map": [['P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'P'], ],
            "drones": {'drone 1': (3, 3)},
            "packages": {'package 1': (0, 2),
                         'package 2': (0, 2)},
            "clients": {'Alice': {"path": [(0, 1), (1, 1), (1, 0), (0, 0)],
                                  "packages": ('package 1', 'package 2')}}
        },

        {
            "map": [['P', 'P', 'P', 'P'],
                    ['P', 'P', 'I', 'P'],
                    ['P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'P'], ],
            "drones": {'drone 1': (3, 3)},
            "packages": {'package 1': (0, 2),
                         'package 2': (0, 2)},
            "clients": {'Alice': {"path": [(0, 1), (1, 1), (2, 0), (0, 0)],
                                  "packages": ('package 1', 'package 2')}}
        },

        {
            "map": [['P', 'P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'P', 'P'],
                    ['P', 'P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P'], ],
            "drones": {'drone 1': (3, 0),
                       'drone 2': (2, 4)},
            "packages": {'package 1': (3, 4)},
            "clients": {'Alice': {"path": [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)],
                                  "packages": ('package 1',)}}
        },

        {
            "map": [['P', 'P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'P', 'P'],
                    ['P', 'P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P'], ],
            "drones": {'drone 1': (3, 0), },
            "packages": {'package 1': (3, 4),
                         'package 2': (3, 4)},
            "clients": {'Alice': {"path": [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)],
                                  "packages": ('package 1',)},
                        'Bob': {"path": [(2, 0), (2, 1), (2, 3), (2, 4)],
                                "packages": ('package 2',)}
                        }
        },

        {
            "map": [['P', 'P', 'P'],
                    ['P', 'P', 'P'],
                    ['P', 'I', 'P']],
            "drones": {'drone 1': (0, 1)},
            "packages": {'package 1': (0, 2),
                         'package 2': (0, 2)},
            "clients": {'Alice': {"path": [(2, 0), (2, 2)],
                                  "packages": ('package 1', 'package 2')}}
        },
        # ---- Competative
        {
            "map": [['P', 'P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'P', 'P'],
                    ['P', 'P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P'], ],
            "drones": {'drone 1': (3, 0),
                       'drone 2': (2, 4)},
            "packages": {'package 1': (3, 4),
                         'package 2': (3, 4),
                         'package 3': (3, 4),
                         'package 4': (3, 4)},
            "clients": {'Alice': {"path": [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)],
                                  "packages": ('package 1', 'package 2')},
                        'Bob': {"path": [(2, 3), (3, 3), (3, 2)],
                                "packages": ('package 3', 'package 4')},
                        }
        },

        {
            "map": [['P', 'P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'P', 'P'],
                    ['P', 'P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P'],
                    ['P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P'], ],
            "drones": {'drone 1': (3, 0),
                       'drone 2': (2, 4)},
            "packages": {'package 1': (5, 4),
                         'package 2': (5, 4),
                         'package 3': (5, 4),
                         'package 4': (5, 4),
                         'package 5': (5, 4)},
            "clients": {'Alice': {"path": [(0, 0), (0, 1), (0, 3), (0, 4)],
                                  "packages": ('package 1', 'package 2')},
                        'Bob': {"path": [(2, 3), (3, 3), (3, 2)],
                                "packages": ('package 3', 'package 4')},
                        'Charlie': {"path": [(5, 1)],
                                    "packages": ('package 5',)},
                        }
        },

        {
            "map": [['P', 'P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'P', 'P'],
                    ['P', 'P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P'],
                    ['P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P'], ],
            "drones": {'drone 1': (3, 0),
                       'drone 2': (2, 4),
                       'drone 3': (5, 0)},
            "packages": {'package 1': (5, 4),
                         'package 2': (5, 4),
                         'package 3': (5, 4),
                         'package 4': (5, 4),
                         'package 5': (5, 4)},
            "clients": {'Alice': {"path": [(0, 0), (0, 1), (0, 3), (0, 4)],
                                  "packages": ('package 5', 'package 2')},
                        'Bob': {"path": [(2, 3), (3, 3), (3, 2)],
                                "packages": ('package 3', 'package 4')},
                        'Charlie': {"path": [(5, 1)],
                                    "packages": ('package 1',)},
                        }
        },

        {
            "map": [['P', 'P', 'P', 'P', 'P'],
                    ['P', 'I', 'P', 'P', 'P'],
                    ['P', 'P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P'],
                    ['P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P'], ],
            "drones": {'drone 1': (3, 0),
                       'drone 2': (2, 4)},
            "packages": {'package 1': (5, 4),
                         'package 2': (5, 4),
                         'package 3': (5, 4),
                         'package 4': (5, 4),
                         'package 5': (5, 4)},
            "clients": {'Alice': {"path": [(0, 0), (0, 1), (0, 3), (0, 4)],
                                  "packages": ('package 1', 'package 2')},
                        'Bob': {"path": [(2, 3), (3, 3), (3, 2)],
                                "packages": ('package 3', 'package 4')},
                        'Charlie': {"path": [(5, 1)],
                                    "packages": ('package 5',)},
                        }
        },
#         {
#             "map": [['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'I', 'P'],
#                     ['P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P'],
#                     ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'I', 'I', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'I', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P'],
#                     ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P']],
#             "drones": {'drone 1': (14, 10),
#                        'drone 2': (8, 11),
#                        'drone 3': (9, 3),},
#             "packages": {'p1': (9, 6),
#                          'p2': (9, 6),
#                          'p3': (9, 6),
#                          'p4': (9, 6),
#                          'p5': (9, 6),
#                          'p6': (9, 6),
# },
#             "clients": {'Alice': {"path": [(0, 0), (0, 1), (1, 1), (1, 0)],
#                                   "packages": ('p1', 'p2', 'p3', 'p4', 'p5')},
#                         'Bob': {"path": [(4, 4), (4, 5), (5, 5), (5, 4)],
#                                 "packages": ('p6', )},
#
#                         }
#         },
        {
            "map": [['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'I', 'P'],
                    ['P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P'],
                    ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'I', 'I', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P'],
                    ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P']],
            "drones": {'drone 1': (14, 10),
                       'drone 2': (8, 11),
                       'drone 3': (9, 3),
                       'drone 4': (6, 0),
                       'drone 5': (1, 12),
                       'drone 6': (13, 2)},
            "packages": {'p1': (9, 6),
                         'p2': (9, 6),
                         'p3': (9, 6),
                         'p4': (9, 6),
                         'p5': (9, 6),
                         'p6': (9, 6),
                         'p7': (9, 6),
                         'p8': (9, 6),
                         'p9': (9, 6),
                         'p10': (9, 6),
                         'p11': (9, 6),
                         'p12': (9, 6),
                         'p13': (9, 6),
                         'p14': (9, 6),
                         'p15': (9, 6),
                         'p16': (9, 6),
                         'p17': (9, 6),
                         'p18': (9, 6),
                         'p19': (9, 6),
                         'p20': (9, 6)},
            "clients": {'Alice': {"path": [(0, 0), (0, 1), (1, 1), (1, 0)],
                                  "packages": ('p1', 'p2', 'p3', 'p4', 'p5')},
                        'Bob': {"path": [(4, 4), (4, 5), (5, 5), (5, 4)],
                                "packages": ('p6', 'p7', 'p8', 'p9', 'p10')},
                        'Charlie': {"path": [(8, 11), (9, 3), (8, 0), (3, 12)],
                                    "packages": ('p11', 'p12', 'p13', 'p14', 'p15')},
                        'David': {"path": [(14, 14), (3, 13), (6, 6), (6, 7)],
                                  "packages": ('p16', 'p17', 'p18', 'p19', 'p20')},
                        }
        },
    ]
    solve_problems(problems)


if __name__ == '__main__':
    main()

    # print([(('move', 'drone 1', (3, 1)), ('move', 'drone 2', (3, 4)), ('move', 'drone 3', (4, 0))), (('move', 'drone 1', (4, 1)), ('move', 'drone 2', (4, 4)), ('move', 'drone 3', (4, 1))), (('move', 'drone 1', (4, 2)), ('move', 'drone 2', (5, 4)), ('move', 'drone 3', (4, 2))), (('move', 'drone 1', (4, 3)), ('pick up', 'drone 2', 'package 1'), ('move', 'drone 3', (4, 3))), (('move', 'drone 1', (4, 4)), ('pick up', 'drone 2', 'package 2'), ('move', 'drone 3', (4, 4))), (('move', 'drone 1', (5, 4)), ('move', 'drone 2', (4, 4)), ('move', 'drone 3', (5, 4))), (('pick up', 'drone 1', 'package 3'), ('move', 'drone 2', (3, 4)), ('pick up', 'drone 3', 'package 4')), (('move', 'drone 1', (4, 4)), ('move', 'drone 2', (2, 4)), ('pick up', 'drone 3', 'package 5')), (('move', 'drone 1', (3, 4)), ('move', 'drone 2', (2, 3)), ('move', 'drone 3', (4, 4))), (('move', 'drone 1', (2, 4)), ('move', 'drone 2', (1, 3)), ('move', 'drone 3', (3, 4))), (('move', 'drone 1', (2, 3)), ('move', 'drone 2', (2, 3)), ('move', 'drone 3', (2, 4))), (('wait', 'drone 1'), ('move', 'drone 2', (1, 3)), ('move', 'drone 3', (2, 3))), (('deliver', 'drone 1', 'Bob', 'package 3'), ('move', 'drone 2', (0, 3)), ('deliver', 'drone 3', 'Bob', 'package 4')), (('move', 'drone 1', (1, 3)), ('move', 'drone 2', (0, 2)), ('move', 'drone 3', (1, 3))), (('move', 'drone 1', (0, 3)), ('move', 'drone 2', (0, 1)), ('move', 'drone 3', (0, 3))), (('move', 'drone 1', (0, 2)), ('move', 'drone 2', (0, 0)), ('move', 'drone 3', (0, 2))), (('move', 'drone 1', (0, 1)), ('deliver', 'drone 2', 'Alice', 'package 2'), ('move', 'drone 3', (0, 1))), (('move', 'drone 1', (0, 0)), ('move', 'drone 2', (1, 0)), ('deliver', 'drone 3', 'Alice', 'package 5')), (('move', 'drone 1', (1, 0)), ('move', 'drone 2', (2, 0)), ('move', 'drone 3', (0, 0))), (('move', 'drone 1', (2, 0)), ('move', 'drone 2', (2, 1)), ('move', 'drone 3', (1, 0))), (('move', 'drone 1', (2, 1)), ('move', 'drone 2', (3, 1)), ('move', 'drone 3', (2, 0))), (('move', 'drone 1', (3, 1)), ('move', 'drone 2', (4, 1)), ('move', 'drone 3', (2, 1))), (('move', 'drone 1', (4, 1)), ('move', 'drone 2', (5, 1)), ('move', 'drone 3', (3, 1))), (('move', 'drone 1', (3, 1)), ('deliver', 'drone 2', 'Charlie', 'package 1'), ('move', 'drone 3', (2, 1)))])
    # print([(('move', 'drone 1', (3, 1)), ('move', 'drone 2', (3, 4)), ('move', 'drone 3', (4, 0))), (('move', 'drone 1', (4, 1)), ('move', 'drone 2', (4, 4)), ('move', 'drone 3', (4, 1))), (('move', 'drone 1', (4, 2)), ('move', 'drone 2', (5, 4)), ('move', 'drone 3', (4, 2))), (('move', 'drone 1', (4, 3)), ('pick up', 'drone 2', 'package 1'), ('move', 'drone 3', (4, 3))), (('move', 'drone 1', (4, 4)), ('pick up', 'drone 2', 'package 2'), ('move', 'drone 3', (4, 4))), (('move', 'drone 1', (3, 4)), ('move', 'drone 2', (4, 4)), ('move', 'drone 3', (5, 4))), (('move', 'drone 1', (4, 4)), ('move', 'drone 2', (4, 3)), ('pick up', 'drone 3', 'package 3')), (('move', 'drone 1', (3, 4)), ('move', 'drone 2', (4, 2)), ('pick up', 'drone 3', 'package 4')), (('move', 'drone 1', (2, 4)), ('move', 'drone 2', (4, 1)), ('move', 'drone 3', (4, 4))), (('move', 'drone 1', (2, 3)), ('move', 'drone 2', (5, 1)), ('move', 'drone 3', (3, 4))), (('move', 'drone 1', (1, 3)), ('deliver', 'drone 2', 'Charlie', 'package 1'), ('move', 'drone 3', (2, 4))), (('move', 'drone 1', (2, 3)), ('move', 'drone 2', (4, 1)), ('move', 'drone 3', (2, 3))), (('move', 'drone 1', (1, 3)), ('move', 'drone 2', (4, 2)), ('deliver', 'drone 3', 'Bob', 'package 4')), (('move', 'drone 1', (2, 3)), ('move', 'drone 2', (4, 3)), ('wait', 'drone 3')), (('move', 'drone 1', (1, 3)), ('move', 'drone 2', (4, 4)), ('wait', 'drone 3')), (('move', 'drone 1', (0, 3)), ('move', 'drone 2', (5, 4)), ('deliver', 'drone 3', 'Bob', 'package 3')), (('move', 'drone 1', (0, 2)), ('pick up', 'drone 2', 'package 5'), ('move', 'drone 3', (1, 3))), (('move', 'drone 1', (0, 1)), ('move', 'drone 2', (4, 4)), ('move', 'drone 3', (0, 3))), (('move', 'drone 1', (0, 0)), ('move', 'drone 2', (3, 4)), ('move', 'drone 3', (0, 2))), (('move', 'drone 1', (0, 1)), ('move', 'drone 2', (2, 4)), ('move', 'drone 3', (0, 1))), (('move', 'drone 1', (0, 0)), ('move', 'drone 2', (1, 4)), ('move', 'drone 3', (0, 0))), (('move', 'drone 1', (0, 1)), ('move', 'drone 2', (0, 4)), ('move', 'drone 3', (0, 1))), (('move', 'drone 1', (0, 0)), ('move', 'drone 2', (0, 3)), ('move', 'drone 3', (0, 0))), (('move', 'drone 1', (0, 1)), ('move', 'drone 2', (0, 2)), ('move', 'drone 3', (0, 1))), (('move', 'drone 1', (0, 0)), ('move', 'drone 2', (0, 1)), ('move', 'drone 3', (0, 0))), (('move', 'drone 1', (0, 1)), ('deliver', 'drone 2', 'Alice', 'package 5'), ('move', 'drone 3', (0, 1))), (('move', 'drone 1', (0, 0)), ('move', 'drone 2', (0, 0)), ('move', 'drone 3', (0, 0))), (('move', 'drone 1', (0, 1)), ('wait', 'drone 2'), ('move', 'drone 3', (0, 1))), (('move', 'drone 1', (0, 0)), ('deliver', 'drone 2', 'Alice', 'package 2'), ('move', 'drone 3', (0, 0)))])

