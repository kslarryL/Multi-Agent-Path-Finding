import heapq
import time as timer

from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    for timeStep in range((max(len(path1), len(path2)))):
        # vertices collision
        if get_location(path1, timeStep) == get_location(path2, timeStep):
            return {'time_step': timeStep, 'loc': [get_location(path1, timeStep)]}
        # edge collision
        elif timeStep >= 0:
            if (get_location(path1, timeStep) == get_location(path2, timeStep + 1)) and (get_location(path1, timeStep + 1) == get_location(path2, timeStep)):
                return {'a1': 0, 'a2': 0, 'time_step': timeStep+1,
                        'loc': [get_location(path1, timeStep), get_location(path1, timeStep + 1)]}

    return {}


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []
    for indexA in range(len(paths)):
        indexB = indexA + 1
        while indexB <= len(paths)-1:
            if detect_collision(paths[indexA], paths[indexB]) != {}:
                collision = detect_collision(paths[indexA], paths[indexB])
                collision['a1'] = indexA
                collision['a2'] = indexB
                collisions.append(collision)
            indexB += 1
                #print("adding", collision)
                #print("adding ab", collisions)
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    #print("colllll", collision['loc'], collision['loc'][0])
    if len(collision['loc']) == 0:
        return []
    #vertax constraint
    elif len(collision['loc']) == 1:
        #print("elif")
        constraints1 = {'agent': collision['a1'], 'loc': collision['loc'], 'time_step': collision['time_step']}
        constraints2 = {'agent': collision['a2'], 'loc': collision['loc'], 'time_step': collision['time_step']}
    #edge constraint
    else:
        #print("else")
        constraints2 = ({'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'time_step': collision['time_step']})
        constraints1 = ({'agent': collision['a1'], 'loc': [collision['loc'][0], collision['loc'][1]], 'time_step': collision['time_step']})
    #print("colllll", collision)
    return [constraints1, constraints2]


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # contraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        ##
        print("path is ", root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print("3.1", root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print("3.2", standard_splitting(collision))

        #test 3.3
        #path = [[(2, 1), (2, 2), (3, 2), (3, 2), (3, 3), (3, 4), (3, 5)], [(1, 2), (1, 3), (1, 4), (2, 4), (2, 4), (3, 4), (4, 4)]]
        #print("3.3",  detect_collisions(path))
        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        #print("self, ", self.open_list)

        expandedNode = 0

        iter = 0
        #while len(self.open_list) > 0:
        while len(self.open_list) > 0:
            #iter +=1
            P = self.pop_node()

            if len(P['collisions']) == 0:
                #print(P['collisions'])
                #print(P['constraints'])
                print("Paths:", P['paths'])
                print("Expanded Nodes are ", expandedNode)
                return P['paths']
            expandedNode += 1
            if(iter > 100):
                return []

            collision = P['collisions'][0]
            constraints = standard_splitting(collision)
            for constraint in constraints:
                New_Constraint = P['constraints']
                Q = {'cost': 0, 'constraints': New_Constraint, 'paths': P['paths'], 'collisions': []}
                Q['constraints'].append(constraint)
                agent = constraint['agent']
                path1 = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                               agent, Q['constraints'])
                if path1 != None:
                    Q['paths'][agent] = path1
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    #print("collison path ", '\n', Q['paths'][0], '\n', Q['paths'][1])

                    #print("collisions  ", Q['collisions'])
                    self.push_node(Q)
                    #print("constraints ttt", Q['constraints'])
                    #print('cost', Q['cost'])

        self.print_results(root)
        return []



    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
