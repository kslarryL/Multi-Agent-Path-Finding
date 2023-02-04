import time as timer

from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        #constraints.append({'agent': 0, 'loc': [(1, 2)], 'time_step': 1})
        #constraints.append({'agent': 0, 'loc': [(1, 5)], 'time_step': 10})

        #task 1.5
        #constraints.append({'agent': 1, 'loc': [(1, 3), (1, 2)], 'time_step': 2})
        #constraints.append({'agent': 1, 'loc': [(1, 3), (1, 4)], 'time_step': 2})
        #constraints.append({'agent': 1, 'loc': [(1, 3)], 'time_step': 2})
        #constraints.append({'agent': 1, 'loc': [(1, 4)], 'time_step': 3})
        #constraints.append({'agent': 1, 'loc': [(1, 2)], 'time_step': 2})
        #print("test is ", constraints)

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            #print("map      ", self.my_map)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
            #print("after i is ", path)
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            for k in range(self.num_of_agents):
                for j in range(len(path)):
                    if k != i:
                        if j == 0:
                            #constraint = {'agent': k, 'loc': [path[j]], 'time_step': j}
                            #constraints.append(constraint)
                            continue
                        constraint = {'agent': k, 'loc': [path[j]], 'time_step': j}
                        constraints.append(constraint)
                        #print("after k is ", constraint)
                        if j != 0:
                            constraint = {'agent': k, 'loc': [path[j], path[j - 1]], 'time_step': j}
                            constraints.append(constraint)
                            #print("after j is ", constraint)
                    #adding constraints after one previous agent reach their goal
                    elif k == i:
                        if j == 0:
                            #constraint = {'agent': k+1, 'loc': [path][j], [path][j -1], 'time_step': j}
                            index = len(self.my_map[0])
                            while index > 0:
                                constraint = {'agent': k+1, 'loc': [path[len(path)-1]], 'time_step': len(path)+index-1}
                                #print("adding constraint after previous agent reach their goal", constraint)
                                constraints.append(constraint)
                                index -= 1
            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
