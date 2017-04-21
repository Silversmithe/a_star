# a_star
For this homework assignment, you will be implementing an A* graph search algorithm.

## Agent
The agent can move a single location per turn. Each turn it will choose from the following actions:

N(y++)
<br>
S(y--)
<br>
E(x++)
<br>
W(x--)

In addition, the agent will have an "energy bucket" to use, which is the maximum amount of energy
the agent can use to reach the goal location. The default amount of energy is 100. The default starting
location for the agent is 0,0 , although the defaults can be changed through the command line arguments 
in main.py


## Cost Function
### Uphill Move Cost
1 + (elevation(x_new, y_new) - elevation(x_old, y_old))^2

### Downhill Move Cost
1 + (elevation(x_old, y_old) - elevation(x_new, y_new))

### Flat Move Cost
1

## Heuristic Function
|x_goal - x_current| + |y_goal - y_current| + elevation(x_goal, y_goal) - elevation(x_curr, y_curr)|

You must also consider:

1. Consider ‘N’ before ‘E’ before ‘S’ before ‘W’. This means that if the agent has two move options, ‘S’ and ‘W’, both of which have the same A* value, it must first expand the ‘S’ option before it expands the ‘W’ option.
2. Two states should never be in the frontier with the same x,y coordinates. If there are, you should remove the one with the higher A* value. If they both have the same A* value, you should discard the newer one.

It is likely significantly easier to enforce these constraints when adding states to the frontier than to try and enforce them when selecting a state to expand.

## Requirements
* implement the class <i>Search</i> , which should be in a file that you create named <b>astar.py</b>
