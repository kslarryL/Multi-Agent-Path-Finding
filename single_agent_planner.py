import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    edge_constraint_table = dict()
    vertex_constraint_table = dict()
    for temp in constraints:
        if len(temp['loc']) == 1:
            # print("timestep", temp['time_step'])
            if temp['agent'] == agent:
                if not temp['time_step'] in vertex_constraint_table:
                    key = temp['time_step']
                    vertex_constraint_table[key] = temp['loc']
                else:
                    key = temp['time_step']
                    vertex_constraint_table[key].append(temp['loc'][0])
        else:
            if temp['agent'] == agent:
                if not temp['time_step'] in edge_constraint_table:
                    key = temp['time_step']
                    edge_constraint_table[key] = temp['loc']
                else:
                    key = temp['time_step']
                    edge_constraint_table[key].append(temp['loc'][0])
                    edge_constraint_table[key].append(temp['loc'][1])
    # print("vertex is ", vertex_constraint_table)
    # print("edge is ", edge_constraint_table)

    # for temp in constraints:
    #   if temp['agent'] == agent:
    #      key = temp['time_step']
    #     edge_constraint_table[key] = temp['loc']
    return edge_constraint_table, vertex_constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time in constraint_table:
        # print("loc in vertex is ", curr_loc, next_loc, constraint_table[next_time])
        for index in constraint_table[next_time]:
            if next_loc == index:
                return True
        # else:
        #    if constraint_table[next_time][index] == curr_loc and constraint_table[next_time][1+index] == next_loc:
        #        return True
    return False


def is_edge_constrained(curr_loc, next_loc, next_time, constraint_table):
    if next_time in constraint_table:
        for index in range(len(constraint_table[next_time]) - 1):  # index is tuple, location
            # print("index    ", index)
            # print("curr    ", curr_loc)
            if (curr_loc, next_loc) == (constraint_table[next_time][index], constraint_table[next_time][index + 1]):
                return True
            index += 1
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    # 1.2
    edge_constraint_table, vertex_constraint_table = build_constraint_table(constraints, agent)
    # print("vertex is      ", vertex_constraint_table)
    # print("edge is      ", edge_constraint_table)
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    if len(vertex_constraint_table.keys()) != 0:
        for i in vertex_constraint_table:
            earliest_goal_timestep = i

    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
    push_node(open_list, root)
    closed_list[(root['loc']), (root['time_step'])] = root

    # h, l = len(my_map), len(my_map[0])
    upperBoundTimestep = 0
    for i in my_map:
        for j in i:
            if j == False:
                upperBoundTimestep += 1
    # upperBoundTimestep = h+l-2
    # print(my_map, "up   ")
    # print(upperBoundTimestep, "upper   ")
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # task 2.4
        # setting up a maximum time for searching path, if time is over return no solution.
        if curr['time_step'] > upperBoundTimestep:
            print("Hit upper bound:", upperBoundTimestep, "time_steps")
            return None

        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['time_step'] >= earliest_goal_timestep:
            # print("earliest_goal_timestep is", earliest_goal_timestep)
            return get_path(curr)

        # 1.1 adding one from range 4 to 5 in order to let it stay at the same point.
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            h, l = len(my_map), len(my_map[0])
            if child_loc[0] < 0 or child_loc[1] < 0:
                continue
            if child_loc[0] >= h or child_loc[1] >= l:
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            # Task1.2 define whether next child_loc is constrained

            if is_constrained(curr['loc'], child_loc, curr['time_step'] + 1, vertex_constraint_table):
                continue
            if is_edge_constrained(curr['loc'], child_loc, curr['time_step'] + 1, edge_constraint_table):
                continue
            #### Changing all the cell to cell,time_step in task 1.1.
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'time_step': curr['time_step'] + 1}
            if (child['loc'], child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc']), (child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc']), (child['time_step'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc']), (child['time_step'])] = child
                push_node(open_list, child)
            ### end of task 1.1 editing.
    return None  # Failed to find solutions
