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
        self.front_frontier = [init_state]                      # ([state]) list of states not visited yet
        self.back_frontier = [self.environment.get_goal_state()]
        self.explored = []

    def search(self):
        """
        Bidirectional Depth-First Search Algorithm
        
        :return: 
        solution ([char, ...]): an array of moves in order (e.g.,['N', 'E', 'E']), or None
        frontier ([State, ...]): an array of states which are in the frontier at the end of the search
        visited ([State, ...]): an array of states that have been expanded during the search
        """
        """
        BREADTH FIRST SEARCH
        
        function bfs(problem) returns a solution, or failure
        
            node <-- a node with State = problem.initial_state, path-cost =0
            if problem.goal_test(node.State) then return SOLUTION(node)
            frontier <- a FIFO queue with node as the only element
            explored <- an empty set
            
            loop do
                if empty?(frontier) then return failure
                node <- pop(frontier) // chooses the shallowest node in frontier
                add node.state to explored
                for each action in problem.actions(node.state) do
                    child <- child-node(problem, node, action)
                    if child.state is not explored in frontier then
                        if problem.goal-test(child.state) then return solution(child)
                        frontier <- insert(child, frontier)
        
        """

        while True:
            # if they both go through their entire frontiers and find nothing, return
            # FAILURE
            if len(self.front_frontier) == 0 and len(self.back_frontier) == 0:
                final_frontier = self.front_frontier
                final_frontier.extend(self.back_frontier)
                return None, final_frontier, self.explored

            # if there are more states, pop from both frontiers
            current_front = None if len(self.front_frontier) == 0 else self.front_frontier.pop(0)
            current_back = None if len(self.back_frontier) == 0 else self.back_frontier.pop(0)

            # add states to explored
            if current_front is not None:
                # if there is a new state to explore, add to explored states and insert
                # its neighboring moves into the frontier
                self.explored.append(current_front)

                # get the next moves
                moves = self.environment.get_available_moves(current_front)
                for move in moves:
                    # calculate move cost
                    move.cost_so_far += self.cost(current_front, move)
                    # if the state already exists in the frontier, take the one with the
                    # lowest cost
                    move_rejected = False
                    remove_list = []
                    for i in range(0, len(self.front_frontier)):
                        if move.position == self.front_frontier[i].position:
                            # take the one with the lowest cost
                            if move.cost_so_far < self.front_frontier[i].cost_so_far:
                                remove_list.append(self.front_frontier[i])
                            elif move.cost_so_far > self.front_frontier[i].cost_so_far:
                                # new move has greater cost, dont  keep it
                                move_rejected = True
                            else:
                                # if they both have the same cost, keep the older one
                                move_rejected = True

                    # remove all discarded nodes from frontier
                    for node in remove_list:
                        self.front_frontier.remove(node)

                    if not move_rejected:
                        if not self.has_been_visited(move) and move.cost_so_far <= self.environment.energy_budget:
                            self.front_frontier.append(move)

            solution_test = self.test_for_solution()
            if solution_test is not None:
                return solution_test

            if current_back is not None:
                # if there is a new state to explore, add to explored states and insert
                # its neighboring moves into the frontier
                self.explored.append(current_back)

                # get the next moves
                moves = self.environment.get_available_moves(current_back)
                for move in moves:
                    # calculate move cost
                    move.cost_so_far += self.cost(current_back, move)
                    # if the state already exists in the frontier, take the one with the
                    # lowest cost
                    move_rejected = False
                    remove_list = []
                    for i in range(0, len(self.back_frontier)):
                        if move.position == self.back_frontier[i].position:
                            # take the one with the lowest cost
                            if move.cost_so_far < self.back_frontier[i].cost_so_far:
                                # remove the current node
                                # add the new node later
                                remove_list.append(self.back_frontier[i])
                            elif move.cost_so_far > self.back_frontier[i].cost_so_far:
                                # new move has greater cost, dont  keep it
                                move_rejected = True
                            else:
                                # if they both have the same cost, keep the older one
                                move_rejected = True

                    # remove all discarded nodes from frontier
                    for node in remove_list:
                        self.back_frontier.remove(node)

                    if not move_rejected:
                        if not self.has_been_visited(move) and move.cost_so_far <= self.environment.energy_budget:
                            self.back_frontier.append(move)

            solution_test = self.test_for_solution()
            if solution_test is not None:
                return solution_test

    # CHECKING METHODS
    def test_for_solution(self):
        """
        
        :return: 
        """
        # check to see if there is any intersection
        overlap_test = self.environment.frontier_overlap(self.front_frontier, self.back_frontier)
        if len(overlap_test) > 0:
            # if there is an intersection, then there is a solution
            final_frontier = self.front_frontier
            final_frontier.extend(self.back_frontier)
            return overlap_test[0], final_frontier, self.explored

        return None

    def has_been_visited(self, current_state):
        """
        Check and see if the state we are looking at has the same coordinates
        as a state we have already looked at

        :param current_state: 
        :return: 
        """
        for node in self.explored:
            if current_state.position == node.position:
                return True
        return False

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
            return 1 + (dest_elevation - src_elevation) ** 2

        else:
            # move is Flat
            return 1

    def display_state(self):
        """
        
        :return: 
        """
        print '-'*50
        print "start frontier:"
        for node in self.front_frontier:
            print "\t", str(node)

        print "\nend frontier:"
        for node in self.back_frontier:
            print "\t", str(node)

        print "\nexplored:"
        for node in self.explored:
            print "\t", str(node)

        print "\nIntersections:"
        for node in self.environment.frontier_overlap(self.front_frontier, self.back_frontier):
            print "\t", str(node)
        print '-'*50
