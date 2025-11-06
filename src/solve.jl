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

    # Create a global solution object 
    solution = Solution([], [], [], [])

    for i ∈ eachindex(problem.workspace.robots)
        # Solve the problem
        prob_id = first(problem_queue.queue[i])[1]

        println("Solving Robot ", i, " Task: ", prob_id )

        prob_info = problem_queue.problem_dict[i][prob_id]
        sol = prob_info.solver.base.planner(prob_info.problem, prob_info.solver)

        # Remove the completed task from the queue and from problem object
        delete!(problem_queue.queue[i], prob_id)
        deleteat!(problem.workspace.robots[i].tasks, prob_id) # TODO fix this for multiple objective of different priorities

        # Update the global solution 
        push!(solution.complete, true)
        push!(solution.X, sol.X)
        push!(solution.X, sol.Y)

        # Update the robots 
        push!(problem.workspace.robots[i].X, sol.X...)
        push!(problem.workspace.robots[i].U, sol.Y...) # TODO change these valiable names
        problem.workspace.robots[i].shape = circle(sol.X[end][1:2], 0.1) # TODO get rid of the magic number and get problem dimension
    end

    return solution # Return a Solution object

end