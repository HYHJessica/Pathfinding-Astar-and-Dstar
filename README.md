## Requirements:
- python3.
- pygame.

#### There are 2 main programs in this folder:

The other files in this folder contain either variables, functions or classes imported into these 2.

#### 1. Data and Plots for Final Report.

It is available in 3 different formats.

	1. Jupyter Notebook (.ipynb)
	2. HTML (.html)
	3. Python Script (.py)

The Notebook contains all the code and figures used to generate the data and plots of the Final Report. If you can't run the Notebook, I've included the HTMl file so that you can view it in a browser to see the code and the outputs. The Python Script contains the same code as the Notebook and produces the same plots. I have reduced the number of iterations for the first 2 tests (D* Incremental Repairing and A* Brute-Force Replanning) from 100 to 1 mainly because the A* test took close to 21 hours so the data is more noisy but the trends are still similar. The grid used is still the same only the averaging was changed

#### 2. A* vs D* With Visual

```pip install pygame```

It is available as a Python Script (.py)

The first 2 prompts that appear give you the option to either 1. replicate the grid used in the report (only the first 100 squares. 1000 results in squares too small to see) or to 2. initialise a grid of x by y size where x and y are user defined (I typically use 100 by 100). Option 2 also requires you to specify an obstacle density (I typically use a value between 0.1 and 0.3) where the obstacles are placed randomly. To try and accommodate for a wider range of nodes (x by y), the window size will scale the size of the squares to fit on the screen however at a certain dimensions, the squares will become too small too see.

After specifying these parameters, the program checks if a file containing Nodes for the specified grid size already exists and loads them if they do or begins creating them if they don't. The reason for loading Nodes vs creating them each time is discussed in the log book.

The next thing that will appear is the pygame window where you will be required to click on a location in the window to set the start and end points for the path planning (you can only place these on white squares or place the end on the start).

The first plan is an A* plan which is then followed by a D* plan the colours are explained further down.

After the plan is complete, press anywhere on the screen to reset the pygame window.

Set new start and end points as before and the process will repeat.

To terminate the program just close the pygame window at any point.

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
