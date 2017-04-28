#!usr/bin/python
"""
File:           state.py

Author:         Alexander Adranly

Description:    Implementation of the state representation of the given environment

"""


class State(object):

    def __init__(self, x_pos, y_pos):
        """
        State Object
            
            Node Representation of a state
        
        --- INSTANCE VARIABLES ---
        self.position: (x_pos, y_pos): tuple holds the x/y coordinate position of the state in the environment
        
        self.moves_so_far: [char, ...] : all past moves used by the agent to reach this state
        
        self.cost_so_var: int: total cost required for the agent to reach this state
        
        self.a_star: int: A* value of the state --> heuristic + cost
        """
        self.position = (x_pos, y_pos)
        self.moves_so_far = []
        self.cost_so_far = 0
        self.a_star = 0

    def __str__(self):
        """
        --- TO STRING OVERRIDE ---
        
        Expected form:
        Pos=(2, 3) Moves=['N', 'N', 'E', 'E'] Cost=4
        
        :return: (str) representation of the state instance as a string
        """
        return "Pos=" + str(self.position) + " Moves=" + str(self.moves_so_far) + " Cost=" + str(self.cost_so_far)
