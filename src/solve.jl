"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("problems/planning_problem.jl")
include("tasks/task_handler.jl")

# Usings



"""
Simulation 
"""
function solve(problem::PlanningProblem)
    # Setup and Set IDs 
    for (i, robot) ∈ enumerate(problem.workspace.robots)
        robot.Id = i
    end

    # Break down into sub-problems
    problem_queue = task_handler(problem)

    # Simulation loop
    max_iters = 42
    for k ∈ 1:max_iters in 
        # Solve each task 

        # Store the results
    end

    return 42 # Return a Solution object

end