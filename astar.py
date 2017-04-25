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

        """
        self.frontier = [init_state]       # ([state]) list of states not visited yet
        self.visited = []                  # ([state]) list of visited states
        self.environment = environment     # (Environment) pointer to environment variable
        self.current_state = None          # (State) current state

    def search(self):
        """
        A* SEARCH ALGORITHM
        This function drives the A* search algorithm
        
        :return: 
        solution ([char, ...]): an array of moves in order (e.g.,['N', 'E', 'E']), or None
        frontier ([State, ...]): an array of states which are in the frontier at the end of the search
        visited ([State, ...]): an array of states that have been expanded during the search
        """
        solution = []
        print "start: a* search"
        # self.test_expansion()

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

        while True:
            if len(self.frontier) == 0:
                return None, self.frontier, self.visited

            self.current_state = self.pick_a_star()

            # if not, explore the node
            self.explore(self.current_state)

            # check if the node we are expanding is a goal state
            if self.environment.is_goal_state(self.current_state):
                print self.current_state.moves_so_far
                return self.current_state, self.frontier, self.visited

        # re-iterate
        print "end: a* search"

    # TRAVERSAL CALCULATION
    def calc_a_star(self):
        for node in self.frontier:
            if node.a_star is None:
                node.a_star = self.a_star(self.current_state, node)

    def pick_a_star(self):
        """
        Pick and return the node with the lowest A* value to be explored
        
        :return: (State) node with lowest A* value
        """
        if self.current_state is None:
            return self.frontier[0]

        lowest_astar = None
        for node in self.frontier:
            # print node
            # print self.a_star(self.current_state, node)
            # print ""
            if lowest_astar is None:
                lowest_astar = node

            elif node.a_star < lowest_astar.a_star:
                lowest_astar = node

        return lowest_astar

    def explore(self, node):
        """
        Explore a frontier node by:
            getting its available neighbor states and calculate their costs
            place these neighbor states in the frontier
            
        :param frontier_node: current node to be explored
        :return: (bool) have we found a solution
        """
        frontier_node = node

        # # check and see if the node has already been visited
        # if self.has_been_visited(frontier_node):
        #     # if visited, just remove from frontier
        #     self.frontier.remove(frontier_node)
        #     return

        # remove from frontier
        self.frontier.remove(frontier_node)

        # add to visited
        self.visited.append(frontier_node)

        """
        add node.STATE to explored
            for each action in problem.ACTION(node.STATE) do
                child <- CHILD-NODE(problem, node, action)
                if child.STATE is not in explored or frontier then
                    frontier <- inser(child, frontier)
                else if child.STATE in frontier with higher PATH-COST then
                    replace that frontier node with child
        """
        # get the unvisited neighbors of the frontier node and add them to the frontier
        moves = self.environment.get_available_moves(frontier_node)

        # !!! update the cost of each move to be accurate
        for move in moves:
            if not self.has_been_visited(move):
                move.cost_so_far += self.cost(frontier_node, move)
                self.frontier.append(move)

            else:
                for node in self.frontier:
                    if node.position == move.position:
                        if self.a_star(node) > self.a_star(move):
                            self.frontier.remove(node)
                            move.cost_so_far += self.cost(frontier_node, move)
                            self.frontier.append(move)

        self.calc_a_star()


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
        return self.cost(src_state, neighbor_state)+self.heuristic(neighbor_state, self.environment.get_goal_state())

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

        return abs(x_goal-x_current) + abs(y_goal-y_current) + abs(goal_elevation-current_elevation)

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
    def display_search_state(self, solution, frontier=True, visited=True):
        """
        Displays all resources used for searching
        
        :tested: TRUE
        :return: 
        """
        print '-'*50
        print "Solution:"
        print str(solution)
        if frontier:
            print "\nFrontier:"
            for item in self.frontier:
                    print "\t"+str(item)
        if visited:
            print "\nVisited:"
            for item in self.visited:
                    print "\t"+str(item)

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

    def test_expansion(self):
        """
        Testing the exploration of the frontier
        :return: 
        """
        solution = []
        # one iteration
        print "First Iteration:"
        self.current_state = self.pick_a_star()
        self.explore(self.current_state)
        self.display_search_state(solution)
        self.calc_a_star()

        # pick another to do a second iteration
        print "Second Iteration:"
        self.current_state = self.pick_a_star()
        self.explore(self.current_state)
        self.display_search_state(solution)
        self.calc_a_star()

        # pick another to do a second iteration
        print "third Iteration:"
        self.current_state = self.pick_a_star()
        self.explore(self.current_state)
        self.display_search_state(solution)

        # # pick another to do a second iteration
        # print "fourth Iteration:"
        # self.current_state = self.pick_a_star()
        # self.explore(self.current_state)
        # self.display_search_state(solution)