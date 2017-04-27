#!usr/bin/python
"""
File:           astar.py

Author:         Alexander Adranly

Description:    Implementation of the Search class for the A* Algorithm

"""
import state


class Search(object):

    def __init__(self, init_state, environment):
        """
        frontier needs to be sorted in descending a* value
        pull from the back, insert into the frontier based on direction preference 
        """
        # FRONTIER
        """
            The frontier is a priority queue that keeps the lowest A* values at the front
            When taking out values, prioritize taking N over E over S over W if they have the same value
        """
        self.frontier = [init_state]            # ([state]) list of states not visited yet

        self.visited = []                       # ([state]) list of visited states
        self.environment = environment          # (Environment) pointer to environment variable
        self.frontier[0].a_star = self.a_star(self.frontier[0], self.frontier[0])
        self.current_state = self.frontier[0]   # (State) current state

    def search(self):
        """
        A* SEARCH ALGORITHM
        This function drives the A* search algorithm
        
        :return: 
        solution ([char, ...]): an array of moves in order (e.g.,['N', 'E', 'E']), or None
        frontier ([State, ...]): an array of states which are in the frontier at the end of the search
        visited ([State, ...]): an array of states that have been expanded during the search
        """
        """
        loop do
            if EMPTY?(frontier) then return failure
            node <- POP(frontier) // chooses lowest cost node in frontier
            if problem.GOAL-TEST(node.STATE) then return SOLUTION(node)
            add node.STATE to explored
            for each action in problem.ACTION(node.STATE) do
                child <- CHILD-NODE(problem, node, action)
                if child.STATE is not in explored or frontier then
                    frontier <- inser(child, frontier)
                else if child.STATE in frontier with higher PATH-COST then
                    replace that frontier node with child
        """

        while len(self.frontier) != 0 and self.current_state.cost_so_far <= self.environment.energy_budget:
            # pick lowest a*, state to explore
            self.current_state = self.frontier.pop()
            # add state to visited
            self.visited.append(self.current_state)

            # expand state
            self.explore(self.current_state)

            # check if goal state
            if self.environment.is_goal_state(self.current_state):
                # print "found goal state!"
                # self.display_search_state()
                # print self.current_state
                return self.current_state, self.frontier, self.visited

        # for i in range(0, 15):
        #     # pick lowest a*, state to explore
        #     # self.display_search_state()
        #     self.current_state = self.frontier.pop()
        #     # add state to visited
        #     self.visited.append(self.current_state)
        #
        #     # expand state
        #     self.explore(self.current_state)
        #
        #     self.display_search_state()

        # print "not found"
        return None, self.frontier, self.visited

    def explore(self, node):
        """
        Explore a frontier node by:
            getting its available neighbor states and calculate their costs
            place these neighbor states in the frontier
            
        :param frontier_node: current node to be explored
        :return: (bool) have we found a solution
        """
        frontier_node = node

        # get the unvisited neighbors of the frontier node and add them to the frontier
        moves = self.environment.get_available_moves(frontier_node)

        # adding values in NESW order
        # !!! update the cost of each move to be accurate
        for move in moves:
            move.cost_so_far += self.cost(frontier_node, move)
            # print move.cost_so_far
            move.a_star = self.a_star(frontier_node, move)

            # adding it into the frontier
            # frontier is a priority queue
            if not self.has_been_visited(move):
                for item in self.frontier:
                    if move.position == item.position:
                        # there is a position clash, one with higher a* leaves
                        if move.a_star > item.a_star or move.a_star == item.a_star:
                            # exit out of loop
                            # do not add move
                            break
                        else:
                            item = move

                else:
                    self.frontier.append(move)

                    i = len(self.frontier)-1
                    # organize like priority queue
                    remove_list = []
                    while i != 0:
                        current_node = self.frontier[i]
                        next_node = self.frontier[i-1]

                        # swap larger ones to the back
                        if current_node.a_star > next_node.a_star:
                            self.frontier[i], self.frontier[i-1] = self.frontier[i-1], self.frontier[i]

                        elif current_node.a_star == next_node.a_star:
                            if len(current_node.moves_so_far) > len(next_node.moves_so_far):
                                # want to prefer states that have been in the frontier for a while
                                self.frontier[i], self.frontier[i - 1] = self.frontier[i - 1], self.frontier[i]

                            elif len(current_node.moves_so_far) == len(next_node.moves_so_far):
                                # if they are the same length, lets have a preference for direction
                                if self.direct_priority(current_node, next_node) == -1:
                                    # current node has inferior direction
                                    self.frontier[i], self.frontier[i - 1] = self.frontier[i - 1], self.frontier[i]

                        i -= 1

    # STATE CALCULATIONS
    def a_star(self, src_state, neighbor_state):
        """
        Calculates the A* value of an unexplored state
        
        f(x) = g(x) + h(x)
        
        h(x) = the cost to get from the neighbor node to the goal
        g(x) = the cost to get from the current node to the neighbor node
        
        :param state: (State): state of which we want to find the A* value
        :return: (int) A* value f(x)
        """
        # cost = 0
        # if src_state.position != neighbor_state.position:
        #     cost = self.cost(src_state, neighbor_state)
        cost = neighbor_state.cost_so_far

        heuristic = self.heuristic(neighbor_state, self.environment.get_goal_state())
        # print str(neighbor_state.position), "cost: ", str(cost), " h: ", str(heuristic), "= ", cost+heuristic
        return cost+heuristic

    def heuristic(self, current_state, goal_state):
        """
        A* heuristic to determine how close the agent is from its current to the goal state
        the heuristic is the MANHATTAN DISTANCE from the current state to the goal state
        
        :tested: TRUE
        :param current_state: (State) current state of the agent
        :param goal_state: (State) final state that determines the current cost
        :return: (int) predicted cost from current state to goal state
        """
        # variables
        x_goal, x_current = goal_state.position[0], current_state.position[0]
        y_goal, y_current = goal_state.position[1], current_state.position[1]

        goal_elevation = self.environment.elevation(x_goal, y_goal)

        current_elevation = self.environment.elevation(x_current, y_current)
        heuristic = abs(x_goal-x_current) + abs(y_goal-y_current) + abs(goal_elevation-current_elevation)
        # print "heuristic from ", str(current_state.position), " to ", str(goal_state.position)," is ", heuristic
        return heuristic

    def cost(self, src_state, dest_state):
        """
        Calculates the cost to transition from the source state to the destination state
        
        :tested: TRUE
        :param src_state: (State) the state the agent is currently at
        :param dest_state: (State) a transitional state for which we want to calculate the resulting cost 
        :return: (int) resultant cost to travel from one state to another state
        """
        src_elevation = self.environment.elevation(src_state.position[0], src_state.position[1])
        dest_elevation = self.environment.elevation(dest_state.position[0], dest_state.position[1])

        # determine if the move is uphill or downhill
        if src_elevation > dest_elevation:
            # move is Downhill
            # 1 + (elevation(x_old, y_old) - elevation(x_new, y_new))
            return 1 + (src_elevation - dest_elevation)

        elif src_elevation < dest_elevation:
            # move is Uphill
            # cost: 1 + (elevation(x_new, y_new) - elevation(x_old, y_old))^2
            return 1 + (dest_elevation - src_elevation)**2

        else:
            # move is Flat
            # cost: 1
            return 1

    # HELPER METHOD
    @staticmethod
    def compare(one_dir, two_dir):
        if one_dir == 'N':
            if two_dir == 'N':
                return 0
            elif two_dir == 'E':
                return 1
            elif two_dir == 'S':
                return 1
            elif two_dir == 'W':
                return 1

        elif one_dir == 'E':
            if two_dir == 'N':
                return -1
            elif two_dir == 'E':
                return 0
            elif two_dir == 'S':
                return 1
            elif two_dir == 'W':
                return 1

        elif one_dir == 'S':
            if two_dir == 'N':
                return -1
            elif two_dir == 'E':
                return -1
            elif two_dir == 'S':
                return 0
            elif two_dir == 'W':
                return 1

        elif one_dir == 'W':
            if two_dir == 'N':
                return -1
            elif two_dir == 'E':
                return -1
            elif two_dir == 'S':
                return -1
            elif two_dir == 'W':
                return 0

    def direct_priority(self, state1, state2):
        """
        The two state move lists should be equal

        :param state1: 
        :param state2: 
        :return: 
        """
        if not len(state1.moves_so_far) == len(state2.moves_so_far):
            # print "error: direct_priority args are not same length"
            return None

        one_dir = state1.moves_so_far
        two_dir = state2.moves_so_far

        i = len(one_dir) - 1
        # return self.compare(one_dir[i], two_dir[i])
        while i >= 0:
            if self.compare(one_dir[i], two_dir[i]) == -1:
                return -1
            elif self.compare(one_dir[i], two_dir[i]) == 1:
                return 1
            else:
                i -= 1
        return 0

    # CHECKING METHODS
    def has_been_visited(self, current_state):
        """
        Check and see if the state we are looking at has the same coordinates
        as a state we have already looked at
        
        :param current_state: 
        :return: 
        """
        for node in self.visited:
            if current_state.position == node.position:
                return True
        return False

    # DEBUG METHODS #
    def display_search_state(self, frontier=True, visited=True):
        """
        Displays all resources used for searching
        
        :tested: TRUE
        :return: 
        """
        print '-'*50
        if frontier:
            print "\nFrontier:"
            for item in self.frontier:
                    print "\t"+str(item), " *: ", item.a_star
        if visited:
            print "\nVisited:"
            for item in self.visited:
                    print "\t"+str(item), " *: ", item.a_star

        print '-' * 50

    def state_calculation_test(self):
        """
        Tests out the functionality of heuristic and cost calculation along with state displays
        
        :tested: TRUE
        :return: 
        """

        # current state diagnostics
        # what is the elevation of the position
        current_state = self.frontier[0]
        print "elevation: "+str(self.environment.elevation(current_state.position[0], current_state.position[1]))

        print "available moves: "
        print str(self.environment.get_available_moves(current_state))

        print "goal state?: " + str(self.environment.is_goal_state(current_state))

        # Neighbor state function diagnostics
        flat_state = state.State(1, 0)
        uphill_state = state.State(0, 1)
        goal_state = state.State(4, 4)

        print "cost from current (0,0) to neighbor (0,1) flat is: " + str(self.cost(current_state, flat_state)) # flat
        print "cost from current (0,0) to neighbor (1,0) uphill is: " + str(self.cost(current_state, uphill_state)) # up
        print "cost from current (1,0) to neighbor (0,0) downhill is: " + str(self.cost(uphill_state, current_state)) # down
        print "\n"
        # testing heuristic
        print "heuristic value from current (0,0) to neighbor (0,1) is: "+str(self.heuristic(current_state, flat_state))
        print "heuristic value from current (0,0) to neighbor (1,0) is: "+str(self.heuristic(current_state, uphill_state))
        print "heuristic value from current (0,0) to neighbor (4,4) is: "+str(self.heuristic(current_state, goal_state))