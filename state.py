#!usr/bin/python
"""
File:           state.py

Author:         Alexander Adranly

Description:    Implementation of the state representation of the given environment

"""


class State(object):

    def __init__(self, x_pos, y_pos):
        """
        current x, y position
        moves so far
        cost so far
        """
        self.position = (x_pos, y_pos)      # a tuple of the current position of the agent
        self.moves_so_far = []              # a list representing all the moves the agent has done until now
        # when is cost activated?
        self.cost_so_far = 0                # an integer representing the cost that the agent has used so far
        self.a_star = 0                     # A* value of the state

    def __str__(self):
        """
        Expected form:
        Pos=(2, 3) Moves=['N', 'N', 'E', 'E'] Cost=4
        
        :tested: TRUE
        :return: (str) representation of the state instance as a string
        """
        return "Pos=" + str(self.position) + " Moves=" + str(self.moves_so_far) + " Cost=" + str(self.cost_so_far)
