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
        
        :param x_pos: int: x coordinate of the point you want the elevation of
        :param y_pos: int: y coordinate of the point you want the elevation of
        :return: int: elevation of coordinate space
        """
        if self.is_valid_position(x_pos=x_pos, y_pos=y_pos):
            return self.elevations[y_pos][x_pos]                # column-major ordering of elevations

        # not a valid position, raise an error (something must be wrong with code)
        raise Exception("error: invalid position (" + str(x_pos) + ", " + str(y_pos) + ")")

    def is_goal_state(self, state):
        """
        --- GOAL EVALUATION ---
        Given a state, determine if it is a goal state or not
        
        :param state: State: state object in question to determine if it is a goal state
        :return: bool: is this state a goal 
        """
        goal = (self.end_x, self.end_y)
        x_align = state.position[0] == goal[0]
        y_align = state.position[1] == goal[1]

        return x_align and y_align

    def frontier_overlap(self, front, back):
        """
        Tests to see if there is an intersection between the two frontiers
        
        :param front: [State,...]: frontier that starts from the starting position
        :param back: [State, ...]: frontier that starts from the ending position
        :return: [State, ...]: list of states that have paths that are solutions, ordered from
                                smallest cost to greatest cost
        """
        overlap = []
        for start in front:
            for end in back:
                if start.position == end.position:
                    sol = State(start.position[0], start.position[1])
                    sol.cost_so_far = start.cost_so_far + end.cost_so_far
                    sol.moves_so_far.extend(start.moves_so_far)

                    end.moves_so_far.reverse()
                    for move in end.moves_so_far:
                        opp = ''
                        if move == 'N':
                            opp = 'S'
                        elif move == 'S':
                            opp = 'N'
                        elif move == 'E':
                            opp = 'W'
                        else:
                            opp = 'E'

                        sol.moves_so_far.append(opp)
                    overlap.append(sol)
                    end.moves_so_far.reverse()

        # get the solution with the smallest state first
        overlap.sort(key=lambda node: node.cost_so_far)
        return overlap

    def get_goal_state(self):
        """
        Returns a state with the position of the goal
        
        :return: State: state that just holds the position of the goal
        """
        return State(self.end_x, self.end_y)

    def get_available_moves(self, state):
        """
        --- TRANSITION MODEL ---
        Given a state, determine the states it can transition to
        
        From a given state, the agent can move:
        N(y++), S(y--), E(x++), W(x--)
        
           ^N
           |
        W<-*->E
           |
           vS
        
        Consider N before E before S before W
        
        :param state: State: state object in question to find tran
        :return: [State,...]: list of states are ordered how they should be considered
        """
        possible_moves = []
        position = state.position

        # Check N Movement Possible
        if position[1] < self.height-1:
            # NORTH
            # y position is lower than highest y pos possible
            # make new frontier node
            north_state = State(position[0], position[1]+1)

            # update the solution if you take this path
            north_state.moves_so_far.extend(state.moves_so_far)
            north_state.moves_so_far.append('N')

            # get cost so far from current state ** will get updated to cost so far in search function
            # not complete cost so far! finished by search.expand()
            north_state.cost_so_far = state.cost_so_far

            # add to possible moves
            possible_moves.append(north_state)

        # Check E Movement Possible
        if position[0] < self.width-1:
            # EAST
            # x position is lower than highest x pos possible
            east_state = State(position[0]+1, position[1])

            # update the solution if you take this path
            east_state.moves_so_far.extend(state.moves_so_far)
            east_state.moves_so_far.append('E')

            # get cost so far from current state ** will get updated to cost so far in search function
            # not complete cost so far! finished by search.expand()
            east_state.cost_so_far = state.cost_so_far

            # add to possible moves
            possible_moves.append(east_state)

        # Check S Movement Possible
        if position[1] > 0:
            # SOUTH
            # y position is greater than lowest y pos possible
            south_state = State(position[0], position[1]-1)

            # update the solution if you take this path
            south_state.moves_so_far.extend(state.moves_so_far)
            south_state.moves_so_far.append('S')

            # get cost so far from current state ** will get updated to cost so far in search function
            # not complete cost so far! finished by search.expand()
            south_state.cost_so_far = state.cost_so_far

            # add to possible moves
            possible_moves.append(south_state)

        # Check W Movement Possible
        if position[0] > 0:
            # WEST
            # x pos is greater than lowest x pos possible
            west_state = State(position[0]-1, position[1])

            # update the solution if you take this path
            west_state.moves_so_far.extend(state.moves_so_far)
            west_state.moves_so_far.append('W')

            # get cost so far from current state ** will get updated to cost so far in search function
            # not complete cost so far! finished by search.expand()
            west_state.cost_so_far = state.cost_so_far

            # add to possible moves
            possible_moves.append(west_state)

        return possible_moves
