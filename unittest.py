#!/usr/bin/python
"""
File:           unittest.py

Author:         Alexander Adranly

"""

import argparse
import environment
import state
import sys

parser = argparse.ArgumentParser(description='Run a single search')
parser.add_argument('search_module', metavar='search-module',
                    help='Search algorithm module name')
parser.add_argument('map_name', metavar='map-name',
                    type=argparse.FileType('r'),
                    help='Name of map file')
parser.add_argument('--energy', type=int, default=100,
                    help='Starting energy level, default is 100')
parser.add_argument('--start-x', type=int, default=0,
                    help='Starting X position, default is 0')
parser.add_argument('--start-y', type=int, default=0,
                    help='Starting Y position, default is 0')
parser.add_argument('--end-x', type=int, default=-1,
                    help='Ending X position, default is last column')
parser.add_argument('--end-y', type=int, default=-1,
                    help='Ending Y position, default is last row')
parser.add_argument('--minimal-display', action='store_true',
                    help='Hide display of things that may change with a different heuristic.\n' + \
                         'Useful for alternate-heuristic tests.')
args = parser.parse_args()

# Import the search algo module, removing the .py extension if found.
if args.search_module.endswith('.py') and len(args.search_module) > 3:
    search_pkg = __import__(args.search_module[:-3])
else:
    search_pkg = __import__(args.search_module)

env = environment.Environment(args.map_name, args.energy, (args.end_x, args.end_y))
initial_state = state.State(args.start_x, args.start_y)
search = search_pkg.Search(initial_state, env)

# testing area
search.search()


