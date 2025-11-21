"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
- https://github.com/schmrlng/MotionPlanning.jl
- https://arxiv.org/pdf/1403.2483
- Claude for programming aid
"""


# Usings 
using DifferentialEquations
using Interpolations 
using LinearAlgebra 
using Optimization
using NearestNeighbors
using Random
using Base.Threads
# using Distributed 
# addprocs(4) 


# Base Includes 
include("../../tasks/navigation.jl")
include("../planning.jl")
include("../../problems/planning_problem.jl")
include("../../constraints.jl")

# Algorithm Includes
include("nodes.jl")
include("sampling.jl")
include("helper.jl")
include("steering.jl")


# ============================================================================ #
# Kinodynamic FMT* Strucutre
# ============================================================================ #


"""
A very dumb algorithm that goes to the goal and ignores obstacles
"""
mutable struct KinoFMTStar <: PlanningAlgorithm 
    # Base Planner
    base::BasePlanner
    type::String

    # Sampling Params 
    n_samples::Int64 
    goal_bias::Float64

    # Connection 
    r::Float64     
    γ::Float64 
    distance::Function
    goal_tolerance::Float64 

    # Cost and Heuristic 
    cost_function::Function 
    heuristic::Function
end

"""
PassThrough Constructor
"""
function KinoFMTStar(; n_samples=1000, goal_bias=0.05, r=3.0, γ=6.0, distance=(x1, x2)->norm(x1-x2), goal_tolerance=1.0, cost_function=x->norm(x), heuristic=x->0)
    # Make the base 
    base = base_planner(:KinoFMTStar, KinoFMTStarPlanner, [])

    return KinoFMTStar(base, "TPBVP", n_samples, goal_bias, r, γ, distance, goal_tolerance, cost_function, heuristic)
end



# ============================================================================ #
# Main Planner
# ============================================================================ #

"""
    KinoFMTStarPlanner(problem, params)

Execute Kinodynamic FMT* planning with proper algorithm structure.

Key Algorithm Steps:
1. Sample N states uniformly in free space
2. Initialize OPEN = {x_start}, CLOSED = {}, UNVISITED = {all other samples}
3. While OPEN not empty:
   a. Select z = argmin_{x ∈ OPEN} cost(x)
   b. Find X_near ⊂ (OPEN ∪ UNVISITED) within radius r
   c. CLOSED ← CLOSED ∪ {z}, OPEN ← OPEN / {z}
   d. For each x ∈ X_near:
      - Find N_x = neighbors of x in CLOSED
      - Solve TPBVP from best parent in N_x to x
      - If successful connection, update parent and move to OPEN
4. Return path to goal if reached
"""
function KinoFMTStarPlanner(problem::TPBVP, params::PlanningAlgorithm)

    Random.seed!()


    T = typeof(problem.x0)
    U = typeof(zeros(problem.m))
    
    # Create start node
    start_node = KinoNode(problem.x0, zeros(problem.m), 0.0)
    start_node.cost = 0.0
    start_node.in_open = true
    
    # Control and state setup
    ctrl_lower, ctrl_upper = get_bounds(problem, :u)
    state_lower, state_upper = get_bounds(problem, :x)
    
    # Goal checking
    goal_region(state) = params.distance(state, problem.xT) <= params.goal_tolerance
    
    # Validation
    valid(traj) = trajectory_validation(traj, problem, state_lower, state_upper)
    
    # Generate uniform samples
    control_duration = 0.25  # TODO: parameterize 0.5
    V_unvisited = generate_kinodynamic_samples(problem, params, control_duration)


    goal_samples = 0
    for z in V_unvisited
        if goal_region(z.state)
            goal_samples += 1
        end
    end

    @info "Samples near goal region: $goal_samples"
    if goal_samples == 0
        @warn "No samples in goal region! Consider increasing n_samples or goal_tolerance"
    end

    @info "State bounds: lower=$state_lower, upper=$state_upper"
    @info "Workspace bounds: $(problem.workspace.bounds)"
    @info "Number of obstacles: $(length(problem.workspace.obstacles))"
    @info "Start state: $(problem.x0)"
    
    # Initialize sets
    V_open = Vector{KinoNode{T, U}}([start_node])
    V_closed = KinoNode{T, U}[]
    
    # Compute connection radius (FMT* formula)
    d = problem.n  # state dimension
    n = length(V_unvisited) + 1
    r_n = min(params.γ * (log(n) / n)^(1/d), params.r)
    
    @info "Connection radius: $r_n"
    
    goal_node = nothing
    iterations = 0
    max_iterations = 2000
    verbose = true
    
    while !isempty(V_open)
        if iterations >= max_iterations
            @warn "Max iterations reached"
            break
        end
        iterations += 1
        
        if verbose && iterations % 100 == 0
            println("Iteration $iterations: |OPEN|=$(length(V_open)), |CLOSED|=$(length(V_closed)), |UNVISITED|=$(length(V_unvisited))")
        end
        
        # Select minimum cost node from OPEN
        z = find_min_cost_node(V_open) # TODO switch to KD or Ball tree
        
        # Check if goal reached
        if goal_region(z.state)
            goal_node = z
            @info "Goal reached at iteration $iterations"
            break
        end
        
        # Find nearby nodes in OPEN ∪ UNVISITED
        X_near = find_nearby_nodes(params, z, V_open, V_unvisited, r_n) # TODO switch to KD or Ball tree
        # num_neighbors = length(X_near)
        # @info "Iteration $iterations: Neighbors: $num_neighbors"
        
        # Move z: OPEN → CLOSED
        filter!(n -> n !== z, V_open)
        push!(V_closed, z)
        z.in_open = false
        z.in_closed = true
        
        # Try to connect nearby nodes
        for x in X_near
            # Find closed neighbors of x
            N_x = find_closed_neighbors(params, x, V_closed, r_n) # TODO switch to KD or Ball tree
            
            if isempty(N_x)
                continue
            end
            
            # Find best parent via TPBVP solving TODO thread canidates
            best_parent = find_best_kinodynamic_parent_proper(
                problem, params, x, N_x, control_duration, valid
            )
            
            if best_parent !== nothing
                # Update x with new parent connection
                x.parent = best_parent.parent_node
                x.cost = best_parent.total_cost
                x.control = best_parent.control
                x.duration = best_parent.duration
                x.trajectory = best_parent.trajectory
                
                # Move x: UNVISITED → OPEN
                if x in V_unvisited
                    filter!(n -> n !== x, V_unvisited)
                    push!(V_open, x)
                    x.in_open = true
                end
            end
        end
    end
    
    # Extract solution
    if goal_node !== nothing
        return extract_kinodynamic_solution(goal_node)
        #return Solution([true], [problem.x0, problem.xT], [zeros(2), zeros(2)], [[]])
    else
        @warn "No path found after $iterations iterations"
        return Solution([false], [problem.x0], [zeros(problem.m)], [[]])
    end
end


"""
    find_best_kinodynamic_parent_proper(problem, params, target, candidates, duration, valid)

Find best parent by solving TPBVP from each candidate to target.

This is the CORRECT implementation for Kinodynamic FMT*.
"""
function find_best_kinodynamic_parent_proper(
    problem::TPBVP,
    params::PlanningAlgorithm,
    target::KinoNode{T, U},
    candidates::Vector{KinoNode{T, U}},
    control_duration::Float64,
    valid::Function
) where {T, U}
    
    best_connection = nothing
    best_cost = Inf
    
    for parent in candidates
        # Solve TPBVP: parent.state → target.state TODO parallelize this
        connection = solve_tpbvp_informed(  # or use solve_tpbvp_sampling
            parent.state,
            target.state,
            problem,
            params,
            control_duration,
            n_attempts = 30,
            distance_threshold = params.goal_tolerance
        )
        
        # Check if connection was successful
        if connection !== nothing && connection.success
            # Compute total cost
            total_cost = parent.cost + connection.cost
            
            # Update best
            if total_cost < best_cost
                best_cost = total_cost
                best_connection = (
                    parent_node = parent,
                    total_cost = total_cost,
                    control = connection.control,
                    duration = control_duration,
                    trajectory = connection.trajectory
                )
            end
        end
    end
    
    return best_connection
end