#!usr/bin/python
"""
File:           astar.py

Author:         Alexander Adranly

Description:    Implementation of the Search class for the A* Algorithm

"""


class Search(object):

    def __init__(self):
        """

        """
        self.frontier = []          # ([state]) list of states not visited yet
        self.visited = []           # ([state]) list of visited states
        self.environment = None     # (Environment) pointer to environment variable

    def search(self):
        """
        A* SEARCH ALGORITHM
        This function drives the A* search algorithm
        
        :return: 
        solution ([char, ...]): an array of moves in order (e.g.,['N', 'E', 'E']), or None
        frontier ([State, ...]): an array of states which are in the frontier at the end of the search
        visited ([State, ...]): an array of states that have been expanded during the search
        """
        # variables
        solution = []
        frontier = []
        visited = []

    def heuristic(self, current_state, goal_state):
        """
        A* heuristic to determine how close the agent is from its current to the goal state
        the heuristic is the MANHATTAN DISTANCE from the current state to the goal state
        
        :param current_state: (State) current state of the agent
        :param goal_state: (State) final state that determines the current cost
        :return: (int) predicted cost from current state to goal state
        """
        pass

    def cost(self, src_state, dest_state):
        """
        Calculates the cost to transition from the source state to the destination state
        
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
