#!usr/bin/python
"""
File:           astar.py

Author:         Alexander Adranly

Description:    Implementation of the Search class for the A* Algorithm

"""


class Search(object):

    def __init__(self, init_state, environment):
        """
        A* SEARCH ALGORITHM
            
            A popular Informed Search Algorithm that is both OPTIMAL and COMPLETE
        
        --- INSTANCE VARIABLES ---
        self.frontier: ([State, ...]): list of unexplored states
        
            The frontier must maintain several qualities:
            Quality 1: the lowest A* value should be in the back of the frontier so it can be popped from the back
            
            Quality 2: Consider directions N before E before S before W, meaning that if an agent has
            multiple options for states that all have the same A* value, pick N before S before E before W
            
            Quality 3: Two states with the same position should never be in the frontier at the same time
            Remove the one with the higher A* value.
            If they have the same A* value, discard the newer one
            
            Quality 4: When two states have the same A* but not the same position, prefer the state that has been
            in the frontier the longest for expansion
        
        self.visited: ([State, ...]): list of explored states
            
            States are ordered in the order that they were explored in the frontier
            ex: [first_explored, second_explored, ...]
            
        self.environment: (Environment): reference to the Environment class
            
            The environment class has all the information about the environment/map that the algorithm needs to 
            search it.
            
        self.heuristics: (int[][]): Two dimensional array containing the heuristic values for each state
            
            In order to reduce function calls throughout the program and slightly increase performance,
            I decided to pre-compute all the heuristic values
            
        self.current_state: (State): The state that is currently selected for expansion
            
        """
        self.frontier = [init_state]
        self.visited = []
        self.environment = environment

        # STATIC ENVIRONMENT
        # Pre-compute Heuristics
        self.heuristics = []
        for r in range(0, self.environment.width):
            self.heuristics.append([])

        for r in range(0, self.environment.width):
            for c in range(0, self.environment.height):
                x_goal, y_goal = self.environment.end_x, self.environment.end_y

                goal_elevation = self.environment.elevation(x_goal, y_goal)
                current_elevation = self.environment.elevation(r, c)

                self.heuristics[r].append(abs(x_goal-r) + abs(y_goal-c) + abs(goal_elevation-current_elevation))

        self.frontier[0].a_star = self.a_star(self.frontier[0])
        self.current_state = self.frontier[0]

    def search(self):
        """
        Function driver for the A* search algorithm
        
        :return: 
        solution ([char, ...]): an array of moves in order (e.g.,['N', 'E', 'E']), or None
        frontier ([State, ...]): an array of states which are in the frontier at the end of the search
        visited ([State, ...]): an array of states that have been expanded during the search
        """

        # Loop
        # while there are still states to explore
        # and the cost of the solution is less than or equal to the energy budget
        while len(self.frontier) != 0 and self.current_state.cost_so_far <= self.environment.energy_budget:

            self.current_state = self.frontier.pop()                              # pick lowest a* state to explore

            # check first if the state we are exploring is a goal state
            if self.environment.is_goal_state(self.current_state):
                # found a goal state
                self.visited.append(self.current_state)                           # note we 'explored' state in a sense
                return self.current_state, self.frontier, self.visited

            self.visited.append(self.current_state)                               # note we actually explored the state
            self.explore()

        return None, self.frontier, self.visited

    def explore(self):
        """
        Expand Upon a Frontier Node and Retrieve the possible states the agent can transition to
        Organize those states in the frontier so that the most favored one ends up in the back
        
        NOTE:
            self.current_state: is where we receive the state we are supposed to expand
            
        :return: None
        """
        frontier_node = self.current_state

        # get the unvisited neighbors of the frontier node
        # AKA: Get all possible moves
        moves = self.environment.get_available_moves(frontier_node)

        # !!! Inserted in NESW Order to maintain direction preference
        for move in moves:
            # For each possible action the agent does, we want to be able to:
            #   1. Calculate the cost and A* value for each new neighbor state
            #   2. Insert or not insert the state into the frontier to help the driver
            #      easily pop the best value to search from the back of the list. By
            #      following the self.frontier requirements defined above

            # 1. Calculate the Cost and A* value
            move.cost_so_far += self.cost(frontier_node, move)
            # print move.cost_so_far
            move.a_star = self.a_star(move)

            # 2. Insert or not insert the state into the frontier to help the driver search
            if not self.has_been_visited(move) and move.cost_so_far <= self.environment.energy_budget:
                # CONDITION: 2
                # check and see if there are any other values in the frontier with A* values
                # If there are: remove the one with the higher A* value (trying to find fastest way to tile)
                # If they both have the same A* value: discard the newer one
                remove_marker = []
                move_rejected = False

                for i in range(0, len(self.frontier)):
                    # check and see if two nodes have the same position, if they do then thats when change happens
                    if move.position == self.frontier[i].position:
                        # two states have the same position, check their A* values
                        if move.a_star < self.frontier[i].a_star:
                            # new state A* is smaller so we take that one
                            remove_marker.append(self.frontier[i])
                        else:
                            # if they both have the same A* value: discard the newer one
                            # remove the one with the higher A* value
                            # reject this new neighbor state
                            move_rejected = True

                # remove all older nodes that have position collisions
                for marker in remove_marker:
                    self.frontier.remove(marker)

                # if the new state was not rejected based on a frontier clash
                # insert accordingly into the frontier
                if not move_rejected:
                    self.frontier.append(move)

                    # LOOP to place new state accordingly
                    select = len(self.frontier)-1  # (int) tracks newly added state index
                    while select > 0:
                        # shuffle to the back of the list as far as you can
                        # swap back if your a star is higher than the guy behind you
                        if self.frontier[select].a_star > self.frontier[select-1].a_star:
                            self.frontier[select], self.frontier[select-1] = self.frontier[select-1], self.frontier[select]
                            select -= 1

                        elif self.frontier[select].a_star == self.frontier[select-1].a_star:
                            # if the two states compared are the same A* value
                            # they are older then you (waiting in frontier longer)
                            # swap behind it
                            self.frontier[select], self.frontier[select-1] = self.frontier[select-1], self.frontier[select]
                            select -= 1

                        else:
                            # select has not decreased this round
                            # select is at the correct place in the frontier
                            # no need to iterate anymore
                            break

    # STATE CALCULATIONS
    def a_star(self, neighbor_state):
        """
        Calculates the A* value of an unexplored state
        
        f(x) = g(x) + h(x)
        
        h(x) = the cost to get from the neighbor node to the goal
        g(x) = the cost to get from the current node to the neighbor node
        
        NOTE:
            
            function 'a_star' works under the assumption that the state has already
            calculated the cost so far to calculate the value
        
        :param neighbor_state: (State): state of which we want to find the A* value
        :return: (int): A* value of the neighbor_state -- aka f(x)
        """
        cost = neighbor_state.cost_so_far

        # heuristic = self.heuristic(neighbor_state, self.environment.get_goal_state())
        heuristic = self.heuristics[neighbor_state.position[0]][neighbor_state.position[1]]
        return cost+heuristic

    def cost(self, src_state, dest_state):
        """
        Calculates the cost to transition from the source state to the destination state
        
        COST EQUATION:
        
            move is Downhill
                1 + (elevation(x_old, y_old) - elevation(x_new, y_new))
                
            move is Uphill
                cost: 1 + (elevation(x_new, y_new) - elevation(x_old, y_old))^2
                
            move is Flat
                cost: 1
        
        :param src_state: (State) the state the agent is currently at
        :param dest_state: (State) a transitional state for which we want to calculate the resulting cost 
        :return: (int) resultant cost to travel from one state to another state
        """
        src_elevation = self.environment.elevation(src_state.position[0], src_state.position[1])
        dest_elevation = self.environment.elevation(dest_state.position[0], dest_state.position[1])

        # determine if the move is uphill or downhill and return the according cost calculation
        if src_elevation > dest_elevation:
            # move is Downhill
            return 1 + (src_elevation - dest_elevation)

        elif src_elevation < dest_elevation:
            # move is Uphill
            return 1 + (dest_elevation - src_elevation)**2

        else:
            # move is Flat
            return 1

    # HELPER METHOD
    def has_been_visited(self, current_state):
        """
        Check and see if the state we are looking at has the same coordinates
        as a state we have already looked at
        
        :param current_state: (State): queried state to check if it has been visited 
        :return: (bool): True if state visited, False if otherwise
        """
        for node in self.visited:
            if current_state.position == node.position:
                return True
        return False
