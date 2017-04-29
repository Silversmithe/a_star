#!usr/bin/python
"""
File:           bbfs.py

Author:         Alexander Adranly

Description:    Implementation of the Search class for the Bidirectional BFS-Algorithm

"""


class Search(object):

    def __init__(self, init_state, environment):
        """
        Bidirectional Breadth First SEARCH ALGORITHM

            Breadth-First Search that searches from the start state and from the goal state
            The algorithm tries to look at the intersection of the two frontiers to find a solution

        --- INSTANCE VARIABLES ---
        self.front_frontier: [State, ...]: list of unexplored states starting from the START STATE
        self.back_frontier: [State, ...]: list of unexplored states starting from the END STATE

            The frontier must maintain several qualities:
            FIFO Queue
            
            The states with the highest priority to expand are the earliest ones inserted
            The expanded states are inserted in the back of the queue

        self.explored: [State, ...]: list of explored states

            States are ordered in the order that they were explored in the frontier
            ex: [first_explored, second_explored, ...]

        self.environment: Environment: reference to the Environment class

            The environment class has all the information about the environment/map that the algorithm needs to 
            search it.

        """
        self.environment = environment
        self.front_frontier = [init_state]
        self.back_frontier = [self.environment.get_goal_state()]
        self.explored = []

    def search(self):
        """
        Bidirectional Depth-First Search Algorithm
        Driver for the search algorithm
        
        :return: 
        solution ([char, ...]): an array of moves in order (e.g.,['N', 'E', 'E']), or None
        frontier ([State, ...]): an array of states which are in the frontier at the end of the search
        visited ([State, ...]): an array of states that have been expanded during the search
        """

        while True:
            # FAILURE CASE
            # if they both go through their entire frontiers and find nothing, return
            if len(self.front_frontier) == 0 and len(self.back_frontier) == 0:
                final_frontier = self.front_frontier
                final_frontier.extend(self.back_frontier)
                return None, final_frontier, self.explored

            # STEP: PICK STATES TO EXPAND FROM FRONT AND END FRONTIERS
            # if there are more states, pop from both frontiers
            current_front = None if len(self.front_frontier) == 0 else self.front_frontier.pop(0)
            current_back = None if len(self.back_frontier) == 0 else self.back_frontier.pop(0)

            # STEP: EXPLORE FRONT STATE
            if current_front is not None:
                # if there is a new state to explore, add to explored states and insert
                # its neighboring moves into the frontier
                self.explored.append(current_front)

                # get the next moves
                moves = self.environment.get_available_moves(current_front)
                for move in moves:
                    # calculate move cost
                    move.cost_so_far += self.cost(current_front, move)
                    # REPETITIVE POSITION FILTER
                    # if the state already exists in the frontier, take the one with the lowest cost
                    if not self.filter(self.front_frontier, move):
                        if not self.has_been_visited(move) and move.cost_so_far <= self.environment.energy_budget:
                            # add move if it is within our budget and hasnt been explored yet
                            self.front_frontier.append(move)

            # SOLUTION TEST
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
                    # REPETITIVE POSITION FILTER
                    # if the state already exists in the frontier, take the one with the lowest cost
                    if not self.filter(self.back_frontier, move):
                        if not self.has_been_visited(move) and move.cost_so_far <= self.environment.energy_budget:
                            # add move if it is within our budget and hasn't been explored yet
                            self.back_frontier.append(move)

            # SOLUTION TEST
            solution_test = self.test_for_solution()
            if solution_test is not None:
                return solution_test

    @staticmethod
    def filter(frontier, node):
        """
        Checks to see if the frontier has any nodes with a similar position to the node
        in question
        The filter will then throw out one of the duplicate nodes based on their comparative costs
        
        :param frontier: [State, ...]: list of unexplored states that possibly share the same position as node
        :param node: State: Node in question that may be a duplicate position with one of the frontier nodes
        :return: (bool): was the node rejected or not?
        """
        move_rejected = False
        remove_list = []
        for i in range(0, len(frontier)):
            if node.position == frontier[i].position:
                # take the one with the lowest cost
                if node.cost_so_far < frontier[i].cost_so_far:
                    # remove the current node
                    # add the new node later
                    remove_list.append(frontier[i])
                elif node.cost_so_far > frontier[i].cost_so_far:
                    # new move has greater cost, dont  keep it
                    move_rejected = True
                else:
                    # if they both have the same cost, keep the older one
                    move_rejected = True

        # remove all discarded nodes from frontier
        for n in remove_list:
            frontier.remove(n)

        return move_rejected

    # CHECKING METHODS
    def test_for_solution(self):
        """
        Tests both of the frontiers to see if there is an existing solution
        If there is a solution, the function will return the necessary information
        If there is not a solution, the function will return None
        
        :return: 
        solution ([char, ...]): an array of moves in order (e.g.,['N', 'E', 'E'])
        frontier ([State, ...]): an array of states which are in the frontier at the end of the search
        visited ([State, ...]): an array of states that have been expanded during the search
        
        or
        
        None
        """
        # check to see if there is any intersection
        overlap_test = self.environment.frontier_overlap(self.front_frontier, self.back_frontier)
        if len(overlap_test) > 0:
            # the solution is in the first index
            # if there is an intersection, then there is a solution
            final_frontier = self.front_frontier
            final_frontier.extend(self.back_frontier)
            return overlap_test[0], final_frontier, self.explored

        return None

    def has_been_visited(self, current_state):
        """
        Check and see if the state we are looking at has the same coordinates
        as a state we have already looked at

        :param current_state: State: state in question that we want to know if we have visited already
        :return: (bool):
        True: the state is question has been visited
        False: the state in question has NOT been visited
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

