## Requirements:
	- python3.
	- pygame.

```pip install pygame```

#### There are 2 main programs in this folder:

The other files in this folder contain either variables, functions or classes imported into these 2.

#### 1. Data and Plots for Final Report.

	Python Script (Data and Plots for Final Report.py)

Python Script to compare and plot the performance of the 2 algorithms. I have reduced the number of iterations for the first 2 tests (D* Incremental Repairing and A* Brute-Force Replanning) from 100 to 1 mainly because the A* test took close to 21 hours so the data is more noisy but the trends are still similar. The grid used is still the same only the averaging was changed.

#### 2. A* vs D* With Visual

	Python Script ( A* vs D* With Visual.py)

1. Input prompt options:

	1. Replicate the grid used in the report (only the first 100 squares. 1000 results in squares too small to see) 
	2. User inputs: grid obstacle density (between 0 and 1) and grid dimensions x and y (I typically use 100 by 100)
	
2. Check for Nodes -> Create them if they don't exist
3. Set the Start and End point of the path planning
4. A* plans followed by D*
5. Set new points or quit the program

Colours Legend:

- Maroon: The path followed by the rover using a D* search.
- Pink: The path followed by the rover using an A* search.
- Red: The start point.
- Blue: The goal or end point
- Green: The current best path given the known information using D*
- Light Blue: The current best path given the known information using A*
- Yellow: The Robots Frame of view/known area.
- White: Empty Space
- Black: Obstacle

### dstar.py 

	module containing Dstar() path planning object
Can be called as follows:
```python
from dstar import Dstar
import makenodes
nodes = makenodes.makenodes(x,y,connections,directory,save,progress_bar) #making nodes
planner = Dstar() 
path = planner.plan(grid,nodes,start,goal) #This gives the initial path. planner.repair is used to repair rhe path.
nodes = planner.reset(nodes) #Resets node values to avoid conflicts when planning a different path
```
### astar.py

	module containing Astar() path planning function
```python
from astar import astar4connected as astar
P = astar(maze,start,goal) #initial path. called again after maze is updated with obstacles
```

###### Plots of Results:

###### D* Repair Time vs Distace to Goal
![](https://github.com/kalebakeits/Pathfinding-Astar-and-Dstar/blob/main/Images/D*%20Repair%20Time%20vs%20Distace%20to%20Goal.png?raw=true)
###### BFR Repair Time vs Distace to Goal
![](https://github.com/kalebakeits/Pathfinding-Astar-and-Dstar/blob/main/Images/BFR%20Time%20vs%20Distance%20to%20Goal.png?raw=true)
###### D* Repair Time from Start to Goal
![](https://github.com/kalebakeits/Pathfinding-Astar-and-Dstar/blob/main/Images/D*%20Repair%20Time%20from%20Start%20to%20Goal.png?raw=true )
###### BFR Time from Start to Goal
![](https://github.com/kalebakeits/Pathfinding-Astar-and-Dstar/blob/main/Images/BFR%20Time%20from%20Start%20to%20Goal.png.png?raw=true)
###### Cum Sum D* Repair Times Start to Goal
![](https://github.com/kalebakeits/Pathfinding-Astar-and-Dstar/blob/main/Images/Cum%20Sum%20D*%20Repair%20Times%20Start%20to%20Goal.png?raw=true)
###### Cum Sum BFR Start to Goal
![](https://github.com/kalebakeits/Pathfinding-Astar-and-Dstar/blob/main/Images/Cum%20Sum%20BFR%20Start%20to%20Goal.png?raw=true)
###### D* Time Making Nodes
![](https://github.com/kalebakeits/Pathfinding-Astar-and-Dstar/blob/main/Images/D*%20Time%20Making%20Nodes.png?raw=true)
###### A* Performance Increasing Grid
![](https://github.com/kalebakeits/Pathfinding-Astar-and-Dstar/blob/main/Images/A*%20Performance%20Increasing%20Grid.png?raw=true)

