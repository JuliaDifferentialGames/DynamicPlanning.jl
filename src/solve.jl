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
include("utils.jl")

# Usings
using DataStructures



"""
Simulation 
"""
function solve!(problem::PlanningProblem)
    # Setup and Set IDs 
    for (i, robot) ∈ enumerate(problem.workspace.robots)
        robot.Id = i
    end

    # Break down into sub-problems
    problem_queue = task_handler(problem)

    # println(problem_queue.queue[1])

    prob_id = first(problem_queue.queue[1])[1]
    prob_info = problem_queue.problem_dict[1][prob_id]
    sol = prob_info.solver(prob_info.problem)

    # Update the robots 
    problem.workspace.robots[1].X = sol.X
    problem.workspace.robots[1].U = sol.Y # TODO change these valiable names
    problem.workspace.robots[1].shape = circle(sol.X[end][1:2], 0.1) # TODO get rid of the magic number and get problem dimension

    return sol # Return a Solution object

end