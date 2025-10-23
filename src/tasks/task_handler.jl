"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("navigation.jl")
include("../problems/tpbvp.jl")

# Usings 
using DataStructures



mutable struct PlanningSubProblem
    problem::Any # TODO give this a proper type 
    solver::Any  # TODO give this a proper type 
end



mutable struct ProblemQueue
    queue::Vector{PriorityQueue}
    problem_dict::Vector{Dict}
end


"""
Method for determining task priorites, creating problem definitions for tasks, andd pairing up problems and solvers
"""
function task_handler(problem)
    # Problem queue and dict object 
    problem_queue = Vector{PriorityQueue{Int, Float64}}()
    problem_dict = Vector{Dict{Int, PlanningSubProblem}}()

    # Loop through each robot
    for robot ∈ problem.workspace.robots
        # Create a task priority queue and subproblem dictionary
        pq = PriorityQueue()
        sub_prob_dict = Dict()

        # Populate the queue and create problems
        for (task_id, task) ∈ enumerate(robot.tasks)
            println("Populating Robot ", 1, " Task: ", task_id )
            # Add the task to the queue via an id
            push!(pq, task_id => task.base.priority)

            # Create a problem from the task
            if task isa NavigationTask
                # Determine the starting location 
                x_init = robot.x0 
                if length(robot.X) != 0
                    x_init = robot.X[end]
                end

                # Make a TPBVP 
                prob = TPBVP(robot.n, robot.m, robot.dynamics, x_init, task.goal, (0.0, task.base.deadline), 10) # Might need to change x0, timespan, and N

                # Map the problem to a solver as a subproblem and put into a dictionary
                sub_prob_dict[task_id] = PlanningSubProblem(prob, robot.planners["TPBVP"])
            end

        end

        # Push to the global objects 
        push!(problem_queue, pq)
        push!(problem_dict, sub_prob_dict)
    end

    # Create a problem queue object 
    return ProblemQueue(problem_queue, problem_dict)

end