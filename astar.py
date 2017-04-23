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

        # in the frontier pick the value with the lowest a* value

        # explore that frontier node, putting its children it the frontier and place itself in the visited list

        # re-iterate
        print "end: a* search"

    # TRAVERSAL CALCULATION
    def explore(self, frontier_node):
        """
        Explore a frontier node by:
            getting its available neighbor states and calculate their costs
            place these neighbor states in the frontier
            
        :param frontier_node: 
        :return: 
        """
        pass

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
                    print "\t"+str(item),
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