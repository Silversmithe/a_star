#!usr/bin/python
"""
File:           bbfs.py

Author:         Alexander Adranly

Description:    Implementation of the Search class for the Bidirectional BFS-Algorithm

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
        self.environment = environment                          # (Environment) pointer to environment variable
        self.frontier_start = [init_state]                      # ([state]) list of states not visited yet
        self.frontier_end = [self.environment.get_goal_state()]
        self.expanded = []

    def search(self):
        """
        Bidirectional Depth-First Search Algorithm
        
        :return: 
        """

        for i in range(0, 14):
            # a loop to control how far we search and we can the adjust its quality
            current_start = self.frontier_start.pop()
            self.expanded.append(current_start)
            self.expand_front(current_start)

            current_end = self.frontier_end.pop()
            self.expanded.append(current_end)
            self.expand_back(current_end)

        self.display_state()

    def expand_front(self, node):
        """
        
        :param node: 
        :return: 
        """
        # add new moves onto the front of the list so that the current ones to pop are on the back
        moves = self.environment.get_available_moves(node)
        result = []
        for move in moves:
            if not self.has_been_visited(move):
                move.cost_so_far += self.cost(node, move)
                result.append(move)

        result.extend(self.frontier_start)
        self.frontier_start = result

    def expand_back(self, node):
        """

        :param: frontier:
        :param node: 
        :return: 
        """
        # add new moves onto the front of the list so that the current ones to pop are on the back
        moves = self.environment.get_available_moves(node)
        result = []
        for move in moves:
            if not self.has_been_visited(move):
                move.cost_so_far += self.cost(node, move)
                result.append(move)

        result.extend(self.frontier_end)
        self.frontier_end = result

    # CHECKING METHODS
    def has_been_visited(self, current_state):
        """
        Check and see if the state we are looking at has the same coordinates
        as a state we have already looked at

        :param current_state: 
        :return: 
        """
        for node in self.expanded:
            if current_state.position == node.position:
                return True
        return False

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
            return 1 + (dest_elevation - src_elevation) ** 2

        else:
            # move is Flat
            # cost: 1
            return 1

    def display_state(self):
        """
        
        :return: 
        """
        print '-'*50
        print "start frontier:"
        for node in self.frontier_start:
            print "\t", str(node)

        print "\nend frontier:"
        for node in self.frontier_end:
            print "\t", str(node)

        print "Intersections:"
        for node in self.environment.frontier_overlap(self.frontier_start, self.frontier_end):
            print "\t", str(node)
        print '-'*50
