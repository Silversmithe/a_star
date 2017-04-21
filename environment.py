#!/usr/bin/python

# import state
from state import State
import sys


class Environment:
    'Map-based environment'

    # Member data
    # elevations: raw data for each position, stored in a list of lists
    #             (each outer list represents a single row)
    # height: number of rows
    # width: number of elements in each row
    # end_x, end_y: location of goal

    def __init__(self, mapfile, energy_budget, end_coords):
        self.elevations = []
        self.height = 0
        self.width = -1
        self.end_x, self.end_y = end_coords
        self.energy_budget = energy_budget
        # Read in the data
        for line in mapfile:
            nextline = [ int(x) for x in line.split()]
            if self.width == -1:
                self.width = len(nextline)
            elif len(nextline) == 0:
                sys.stderr.write("No data (or parse error) on line %d\n"
                                 % (len(self.elevations) + 1))
                sys.exit(1)
            elif self.width != len(nextline):
                sys.stderr.write("Inconsistent map width in row %d\n"
                                 % (len(self.elevations) + 1))
                sys.stderr.write("Expected %d elements, saw %d\n"
                                 % (self.width, len(nextline)))
                sys.exit(1)
            self.elevations.insert(0, nextline)
        self.height = len(self.elevations)
        if self.end_x == -1:
            self.end_x = self.width - 1
        if self.end_y == -1:
            self.end_y = self.height - 1

    def is_valid_position(self, x_pos, y_pos):
        """
        Identifies if given coordinates are valid positions
        
        :param x_pos: (int) x coordinate position
        :param y_pos: (int) y coordinate position
        :return: (bool) if the coordinate is valid in this elevation map
        """
        x_valid = (0 <= x_pos <= self.width-1)
        y_valid = (0 <= y_pos <= self.height-1)
        return x_valid and y_valid

    def elevation(self, x_pos, y_pos):
        """
        Return the elevation at a given state
        :param x_pos: 
        :param y_pos: 
        :return: 
        """
        if self.is_valid_position(x_pos=x_pos, y_pos=y_pos):
            return self.elevations[y_pos][x_pos]                # column-major ordering of elevations

        # not a valid position, raise an error (something must be wrong with code)
        raise Exception("error: invalid position (" + str(x_pos) + ", " + str(y_pos) + ")")

    def is_goal_state(self, state):
        """
        GOAL EVALUATION
        Given a state, determine if it is a goal state or not
        
        :param state: (State) state object in question to determine if it is a goal state
        :return: (bool) 
        """
        goal = (self.width-1, self.height-1)
        x_align = state.position[0] and goal[0]
        y_align = state.position[1] and goal[1]

        return x_align and y_align

    def get_available_moves(self, state):
        """
        TRANSITION MODEL
        Given a state, determine the states it can transition to
        
        From a given state, the agent can move:
        N(y++), S(y--), E(x++), W(x--)
        
           ^N
           |
        W<-*->E
           |
           vS
        
        Consider N before E before S before W
        
        :param state: (State) state object in question to find tran
        :return: [State,...] list of states are ordered how they should be considered
        """
        possible_moves = []
        position = state.position

        # Check N Movement Possible
        if position[0] < self.height-1:
            # y position is lower than highest y pos possible
            possible_moves.append('N')

        # Check E Movement Possible
        if position[1] < self.width-1:
            # x position is lower than highest x pos possible
            possible_moves.append('E')

        # Check S Movement Possible
        if position[0] > 0:
            # y position is greater than lowest y pos possible
            possible_moves.append('S')

        # Check W Movement Possible
        if position[1] > 0:
            # x pos is greater than lowest x pos possible
            possible_moves.append('W')

        return possible_moves
