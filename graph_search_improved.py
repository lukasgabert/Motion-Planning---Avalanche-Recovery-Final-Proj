#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import heapq
import matplotlib.pyplot as plotter
from math import hypot

_DEBUG = False
_DEBUG_END = True
_ACTIONS = ['u','d','l','r']
_ACTIONS_REVERSED = ['r','l','d','u']
_ACTIONS_2 = ['u','d','l','r','ur','ul','dr','dl']
_X = 1
_Y = 0
_GOAL_COLOR = 0.75
_INIT_COLOR = 0.25
_PATH_COLOR_RANGE = _GOAL_COLOR-_INIT_COLOR
_VISITED_COLOR = 0.9


class GridMap:
    '''
    Class to hold a grid map for navigation. Reads in a map.txt file of the format
    0 - free cell, x - occupied cell, g - goal location, i - initial location.
    Additionally provides a simple transition model for grid maps and a convience function
    for displaying maps.
    '''
    def __init__(self, map_path=None, cliff_heightt=2.0):
        '''
        Constructor. Makes the necessary class variables. Optionally reads in a provided map
        file given by map_path.

        map_path (optional) - a string of the path to the file on disk
        '''
        self.rows = None
        self.cols = None
        self.height_dict = {}
        self.height = None
        self.cliff_height = cliff_heightt
        self.cell_num_list = []
        self.adjacency_graph = []
        if map_path is not None:
            self.read_map(map_path)


    def read_map(self, map_path):
        '''
        Read in a specified map file of the format described in the class doc string.

        map_path - a string of the path to the file on disk
        '''
        map_file = file(map_path,'r')
        lines = [l.rstrip().lower() for l in map_file.readlines()]
        map_file.close()
        self.rows = len(lines)
        self.cols = max([len(l) for l in lines])
        self.init_pos = (self.rows, self.cols)
        if _DEBUG:
            print 'rows', self.rows
            print 'cols', self.cols
            print lines
        self.decomp_grid = np.zeros((self.rows, self.cols))
        self.occupancy_grid = np.zeros((self.rows, self.cols), dtype=np.bool)
        for r in xrange(self.rows):
            for c in xrange(self.cols):
                self.height_dict[(r, c)] = float(lines[r][c])

        # Perform Cell decomposition based on height
        # Run through states left to right down columns, if diff > cliff height, make it a "cliff"
        is_now_next_row = False
        need_new_cell_next_row = False
        is_new_cell = False
        for r in xrange(self.rows):
            if need_new_cell_next_row:
                is_now_next_row = True
                need_new_cell_next_row = False
            for c in xrange(self.cols):

                # Set value of cell to decomp grid
                if is_now_next_row:
                    is_now_next_row = False
                    is_new_cell = True
                    print is_new_cell
                # look for a vertical discontinuity
                # elif r > 0 and \
                #         abs(self.height_dict[(r, c)] - self.height_dict[(r-1, c)]) > self.cliff_height:
                #     if c > 0:
                #         if self.decomp_grid[r - 1, c - 1] == self.decomp_grid[r, c - 1]:
                #             is_new_cell = True
                #     else:
                #         is_new_cell = True
                #     print is_new_cell
                # look for a horizontal discontinuity
                elif c > 0 and \
                       abs(self.height_dict[(r, c)] - self.height_dict[(r, c-1)]) > self.cliff_height:
                    if c > 0:
                        if self.decomp_grid[r - 1, c - 1] == self.decomp_grid[r - 1, c]:
                            is_new_cell = True
                    else:
                        is_new_cell = True
                    print is_new_cell
                else:
                    is_new_cell = False
                    print is_new_cell
                self.decomp_grid[r][c] = self.find_cell_num(r, c, is_new_cell)

                # CHECK TO SEE IF VERTEX IN NEXT COLUMN, and set value to create new set of cells next column
                if r < self.rows - 1 and not need_new_cell_next_row:
                    # Horizontal vertex
                    if c > 0:
                        if abs(self.height_dict[(r + 1, c - 1)] - self.height_dict[(r + 1, c)]) > self.cliff_height:
                            if self.decomp_grid[r, c - 1] == self.decomp_grid[r, c]:
                                need_new_cell_next_row = True
                                print "Horizontal Vertex"
                    # Vertical vertex
                        if abs(self.height_dict[(r, c)] - self.height_dict[(r + 1, c)]) > self.cliff_height:
                            if r > 1 and self.decomp_grid[r - 1, c - 1] == self.decomp_grid[r - 1, c]:
                                need_new_cell_next_row = True
                                print "Vertical Vertex"
                            else:
                                need_new_cell_next_row = True
                                print "Vertical Vertex Else"
        print self.decomp_grid

    def find_cell_num(self, row, col, is_new):
        '''
        Input a state and say if you want to define a new cell or keep using the old one.
        '''
        cell_number = None
        if len(self.cell_num_list) < 1:
            cell_number = 0
            self.cell_num_list.append(cell_number)
        elif is_new:
            # Find biggest cell num, use 1 more than that
            cell_number = max(self.cell_num_list) + 1
            self.cell_num_list.append(cell_number)
            # ADD TO ADJACENCY GRAPH
            # Find left cell, add to graph
            if col > 0:
                self.adjacency_graph.append((cell_number,
                                             self.decomp_grid[row, col-1]))
            # Find Upper cell, add to graph
            if row > 0:
                self.adjacency_graph.append((cell_number, self.decomp_grid[row-1, col]))
        else:
            # Find cell to left/up and use that cell
            # If wall to left, use up val
            if ((col > 0) and (row > 0)) and \
                    (abs(self.height_dict[(row, col)] - self.height_dict[(row, col - 1)]) > self.cliff_height):
                cell_number = self.decomp_grid[row - 1, col]
            elif col is 0:
                cell_number = self.decomp_grid[row - 1, col]
            # else, use upper val
            else:
                cell_number = self.decomp_grid[row, col - 1]
        return cell_number

    def is_goal(self,s):
        '''
        Test if a specifid state is the goal state

        s - tuple describing the state as (row, col) position on the grid.

        Returns - True if s is the goal. False otherwise.
        '''
        return (s[_X] == self.goal[_X] and
                s[_Y] == self.goal[_Y])

    def transition(self, s, a):
        '''
        Transition function for the current grid map.

        s - tuple describing the state as (row, col) position on the grid.
        a - the action to be performed from state s

        returns - s_prime, the state transitioned to by taking action a in state s.
        If the action is not valid (e.g. moves off the grid or into an obstacle)
        returns the current state.
        '''
        new_pos = list(s[:])
        # Ensure action stays on the board
        if a == 'u':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
        elif a == 'd':
            if s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
        elif a == 'l':
            if s[_X] > 0:
                new_pos[_X] -= 1
        elif a == 'r':
            if s[_X] < self.cols - 1:
                new_pos[_X] += 1
        elif a == 'ur':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
            if s[_X] < self.cols - 1:
                new_pos[_X] += 1
        elif a == 'ul':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
            if s[_X] > 0:
                new_pos[_X] -= 1
        elif a == 'dr':
            if s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
            if s[_X] < self.cols - 1:
                new_pos[_X] += 1
        elif a == 'dl':
            if s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
            if s[_X] > 0:
                new_pos[_X] -= 1
        else:
            print 'Unknown action:', str(a)

        # Test if new position is clear
        if self.occupancy_grid[new_pos[0], new_pos[1]]:
            s_prime = tuple(s)
        else:
            s_prime = tuple(new_pos)
        return s_prime

    def display_map(self, path=[], visited={}):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        '''
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)

        # Color all visited nodes if requested
        for v in visited:
            display_grid[v] = _VISITED_COLOR
        # Color path in increasing color from init to goal
        for i, p in enumerate(path):
            disp_col = _INIT_COLOR + _PATH_COLOR_RANGE*(i+1)/len(path)
            display_grid[p] = disp_col

        display_grid[self.init_pos] = _INIT_COLOR
        display_grid[self.goal] = _GOAL_COLOR

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('spectral')
        plotter.show()

    def display_cell_values(self):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        '''
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)

        # Color all visited nodes if requested
        # for v in self.occupancy_grid:
        # display_grid[v] = _VISITED_COLOR
        # Color path in increasing color from init to goal
        for r in xrange(self.rows):
            for c in xrange(self.cols):
                plotter.text(c, r, self.decomp_grid[(r, c)],
                             ha='center', va='center', fontsize=16, color='gray')

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('spectral')
        plotter.show()

    def display_cell_height(self):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        '''
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)

        print self.decomp_grid

        # Color all visited nodes if requested
        # for v in self.occupancy_grid:
        # display_grid[v] = _VISITED_COLOR
        # Color path in increasing color from init to goal
        for r in xrange(self.rows):
            for c in xrange(self.cols):
                plotter.text(c, r, self.height_dict[r, c],
                             ha='center', va='center', fontsize=16, color='gray')

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('spectral')
        plotter.show()

    def uninformed_heuristic(self, s):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.
        s - tuple describing the state as (row, col) position on the grid.
        returns - floating point estimate of the cost to the goal from state s
        '''
        return 0.0

    def euclidean_heuristic(self, s):
        '''
        s - tuple describing the state as (row, col) position on the grid.
        returns - floating point estimate of the cost to the goal from state s
        '''
        gx = self.goal[_X]  #  define the goal x and y locations
        gy = self.goal[_Y]
        return np.hypot(s[_X]-gx,s[_Y]-gy)

    def manhattan_heuristic(self, s):
        '''
        s - tuple describing the state as (row, col) position on the grid.
        returns - floating point estimate of the cost to the goal from state s
        '''
        gx = self.goal[_X]  #  define the goal x and y locations
        gy = self.goal[_Y]
        return abs(s[_X]-gx)+abs(s[_Y]-gy)


class SearchNode:
    def __init__(self, s, A, parent=None, parent_action=None, cost=0):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = cost
        self.parent_action = parent_action
        self.state = s[:]
        self.actions = A[:]

    def __str__(self):
        '''
        Return a human readable description of the node
        '''
        return str(self.state) + ' ' + str(self.actions)+' '+str(self.parent)+' '+str(self.parent_action)

    def path(self):
        plan = []
        totpath = []
        s = self
        while s is not None:
            plan.insert(0, s.parent_action)
            totpath.insert(0, s.state)
            s = s.parent
        return plan, totpath


class PriorityQ:
    '''
    Priority queue implementation with quick access for membership testing
    Setup currently to only with the SearchNode class
    '''
    def __init__(self):
        '''
        Initialize an empty priority queue
        '''
        self.l = [] # list storing the priority q
        self.s = set() # set for fast membership testing

    def __contains__(self, x):
        '''
        Test if x is in the queue
        '''
        return x in self.s

    def push(self, x, cost):
        '''
        Adds an element to the priority queue.
        If the state already exists, we update the cost
        '''
        if x.state in self.s:
            return self.replace(x, cost)
        heapq.heappush(self.l, (cost, x))
        self.s.add(x.state)

    def pop(self):
        '''
        Get the value and remove the lowest cost element from the queue
        '''
        x = heapq.heappop(self.l)
        self.s.remove(x[1].state)
        return x[1]

    def peak(self):
        '''
        Get the value of the lowest cost element in the priority queue
        '''
        x = self.l[0]
        return x[1]

    def __len__(self):
        '''
        Return the number of elements in the queue
        '''
        return len(self.l)

    def replace(self, x, new_cost):
        '''
        Removes element x from the q and replaces it with x with the new_cost
        '''
        if x.cost < new_cost:
            for y in self.l:
                if x.state == y[1].state:
                    self.l.remove(y)
                    self.s.remove(y[1].state)
                    break
            heapq.heapify(self.l)
            self.push(x, new_cost)

    def get_cost(self, x):
        '''
        Return the cost for the search node with state x.state
        '''
        for y in self.l:
            if x.state == y[1].state:
                return y[0]

    def __str__(self):
        '''
        Return a string of the contents of the list
        '''
        return str(self.l)


def path_coverage(init_state, f, is_goal, actions):
    '''
    Perform depth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    # Perform Cell decomposition based on height
    # Run through states left to right up columns, if diff >1, make it a "cliff"

    # Find order of states to visit with Traveling Salesman

    # Run "lawnmower" search on each state, move to next state with A*?


    frontier = [] # Search Stack
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.append(n0)
    while len(frontier) > 0:
        # Peak last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return n_i.path(), visited, 1
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode(s_prime, actions, n_i, a)
                    frontier.append(n_prime)
    return [], visited, None


def iter_dfs(init_state, f, actions):
    '''
    Perform depth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    frontier = []  # Search Stack
    depth_frontier = []
    max_depth = 4
    n0 = SearchNode(init_state, actions)
    frontier.append(n0)
    tot_visited = {}
    # CREATE ORDER BETWEEN ALL CELLS IN CELL DECOMP USING ADJACENCY MATRIX
    # DO BOUSTROPHEDON MOVEMENT IN A CELL
    # TRAVEL TO NEXT CELL IN THE ORDER THEN REPEAT
    while len(frontier) > 0:
        # Peak last element
        n0 = SearchNode(init_state, actions)
        depth_frontier.append(n0)
        frontier = []
        visited = []
        while len(depth_frontier) > 0:
            n_i = depth_frontier.pop()
            if (n_i.state not in visited) or (len(n_i.path()[1]) < tot_visited[n_i.state]):
                tot_visited[n_i.state] = len(n_i.path()[1])
                visited.append(n_i.state)
                if is_goal(n_i.state):
                    return n_i.path(), tot_visited, 1
                else:
                    for a in actions:
                        s_prime = f(n_i.state, a)
                        n_prime = SearchNode(s_prime, actions, n_i, a)
                        if len(n_prime.path()[1]) <= max_depth:
                            depth_frontier.append(n_prime)
                        else:
                            frontier.append(n_prime)
        max_depth += 1
    return [], tot_visited, None


def bfs(init_state, f, is_goal, actions):
    '''
    Perform breadth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    frontier = [] # Search Stack
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.insert(0,n0)
    while len(frontier) > 0:
        # Peak last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.insert(0,n_i.state)
            if is_goal(n_i.state):
                return n_i.path(), visited, 1
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode(s_prime, actions, n_i, a)
                    frontier.insert(0,n_prime)
    return [], visited, None


def uniform_cost_search(init_state, f, is_goal, actions):
    '''
    This does uniform cost search
    '''
    frontier = PriorityQ()  # Search Stack
    n0 = SearchNode(init_state, actions)
    visited = {}
    frontier.push(n0, 1)
    while frontier.__len__() > 0:
        # Peak last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited[n_i.state] = n_i.cost
            if is_goal(n_i.state):
                return n_i.path(), visited, 1
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    if a in ['u', 'd', 'l', 'r']:
                            n_prime = SearchNode(s_prime, actions, n_i, a,  1 + n_i.cost)
                            frontier.push(n_prime, 1 + n_i.cost)
                    else:
                            n_prime = SearchNode(s_prime, actions, n_i, a, 1.5 + n_i.cost)
                            frontier.push(n_prime, 1.5 + n_i.cost)
    return [], visited, None


def a_star_search(init_state, f, is_goal, actions, h):
    '''
    This does a star search
    '''
    frontier = PriorityQ()  # Search Stack
    n0 = SearchNode(init_state, actions)
    visited = {}
    frontier.push(n0, 1)
    while frontier.__len__() > 0:
        # Peak last element
        n_i = frontier.pop()
        if (n_i.state not in visited) or (n_i.cost < visited[n_i.state]):
            visited[n_i.state] = n_i.cost
            if is_goal(n_i.state):
                return n_i.path(), visited, 1
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    heur = h(s_prime)
                    if a in ['u', 'd', 'l', 'r']:
                        n_prime = SearchNode(s_prime, actions, n_i, a,  1 + n_i.cost)
                        if (s_prime in visited and visited[s_prime] > n_prime.cost) or \
                                (s_prime not in visited and s_prime not in frontier):
                            frontier.push(n_prime, 1 + n_i.cost + heur)
                        elif s_prime in frontier and n_prime.cost + heur < frontier.get_cost(n_prime):
                            frontier.replace(n_prime,n_prime.cost + heur)
                    else:
                        n_prime = SearchNode(s_prime, actions, n_i, a, 1.5 + n_i.cost)
                        if (s_prime in visited and visited[s_prime] > n_prime.cost) or \
                                (s_prime not in visited and s_prime not in frontier):
                            frontier.push(n_prime, 1.5 + n_i.cost + heur)
                        elif s_prime in frontier and n_prime.cost + heur < frontier.get_cost(n_prime):
                            frontier.replace(n_prime,n_prime.cost + heur)
    return [], visited, None
