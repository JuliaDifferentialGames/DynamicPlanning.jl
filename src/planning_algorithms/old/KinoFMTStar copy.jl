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

# Includes 
include("../tasks/navigation.jl")
include("planning.jl")
include("../problems/planning_problem.jl")
include("../constraints.jl")

# Usings 

# Usings 
using DifferentialEquations
using Interpolations 
using LinearAlgebra 
using Optimization
using NearestNeighbors
using Random


# ============================================================================ #
# Kinodynamic Node Structure
# ============================================================================ #

"""
    KinoNode{T, U}

Node for kinodynamic planning containing state and control trajectory.

# Fields
- `state::T`: State at this node (position, velocity, etc.)
- `control::Union{U, Nothing}`: Control input that reached this state
- `duration::Float64`: Duration of control application
- `cost::Float64`: Cost-to-come from start
- `parent::Union{KinoNode{T,U}, Nothing}`: Parent node
- `trajectory::Vector{T}`: State trajectory from parent (for collision checking)
- `in_open::Bool`: Node in OPEN set flag
- `in_closed::Bool`: Node in CLOSED set flag
"""
mutable struct KinoNode{T, U}
    state::T
    control::Union{U, Nothing}
    duration::Float64
    cost::Float64
    parent::Union{KinoNode{T, U}, Nothing}
    trajectory::Vector{T}
    in_open::Bool
    in_closed::Bool
end

function KinoNode(state::T, control::Union{U, Nothing}=nothing, 
                  duration::Float64=0.0) where {T, U}
    KinoNode{T, U}(state, control, duration, Inf, nothing, T[], false, false)
end



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
function KinoFMTStar(; n_samples=50000, goal_bias=0.05, r=1.0, γ=1.0, distance=(x1, x2)->norm(x1-x2), goal_tolerance=0.1, cost_function=x->norm(x), heuristic=x->0)
    # Make the base 
    base = base_planner(:KinoFMTStar, KinoFMTStarPlanner, [])

    return KinoFMTStar(base, "TPBVP", n_samples, goal_bias, r, γ, distance, goal_tolerance, cost_function, heuristic)
end


# ============================================================================ #
# Sample Generation
# ============================================================================ #


function generate_kinodynamic_samples(
    problem::TPBVP, params::PlanningAlgorithm, control_duration::Float64
)
    # Extract types
    T = typeof(problem.x0)
    U = typeof(zeros(problem.m)) 

    # Control sampling
    ctrl_lower, ctrl_upper = get_bounds(problem, :u)
    gen_ctrl() = sample_control(ctrl_lower, ctrl_upper)

    # State Sampling 
    state_lower, state_upper = get_bounds(problem, :x)
    gen_state() = sample_SE2(10.0, state_lower, state_upper) # TODO: magic numbered the radius for the workspace

    # Define goal region check and sampling
    goal_region(state) = params.distance(state, problem.xT) <= params.goal_tolerance 
    gen_goal() = sample_SE2(params.goal_tolerance , state_lower, state_upper)

    # Trajectory validation 
    valid(traj) = trajectory_validation(traj, problem, state_lower, state_upper)
    
    # Sampling settings
    sample_count = 1
    attempts = 0
    max_attempts = params.n_samples * 100  # More generous limit
    
    # TODO: GPT code that needs dome more validation

    # 1. Always include start
    samples = KinoNode{T,U}[]
    start_node = KinoNode(problem.x0, [0.0])
    start_node.cost = 0.0
    push!(samples, start_node)
    sample_count = 1

    # 2. Generate N-1 kinodynamic samples
    while sample_count < params.n_samples && attempts < max_attempts
        attempts += 1

        # Pick a node to extend from
        parent = samples[rand(1:length(samples))]

        # Sample control
        control = gen_ctrl()

        # Simulate forward
        traj, final_state = simulate_forward(parent.state, control, problem.f, control_duration)

        # Only accept collision-free trajectories
        if !valid(traj)
            continue
        end

        node = KinoNode(final_state, control, control_duration)
        node.trajectory = traj
        node.cost = parent.cost + params.cost_function(final_state - problem.xT)  # if you have a cost function
        push!(samples, node)
        sample_count += 1
    end

    # 3. Ensure goal coverage
    goal_added = 0
    ensure_goal_ct = 1
    while goal_added < ensure_goal_ct && sample_count < params.n_samples
        # Pick a node to extend from
        parent = samples[rand(1:length(samples))]

        # Sample a control that moves toward goal
        control = control_toward(parent.state, problem.xT, problem.f, control_duration)

        traj, final_state = simulate_forward(parent.state, control, problem.f, control_duration)

        if valid(traj) && goal_region(final_state)
            node = KinoNode(final_state, control, control_duration)
            node.trajectory = traj
            push!(samples, node)
            sample_count += 1
            goal_added += 1
        end
    end

    
    @info "Generated $sample_count kinodynamic samples in $attempts attempts"
    return samples
end


"""
Collision and bound checking
"""
function trajectory_validation(traj::Vector, problem::TPBVP, state_lower::Vector, state_upper::Vector)

    for state ∈ traj 
        # Check if within state constraints 
        if (any(state < state_lower) || any(state > state_upper))
            return false 
        end

        # Check if in the workspace 
        if (!(state[1:2] ∈ problem.workspace.bounds))
            return false 
        end

        # Check if not hitting obstacles
        for obs ∈ problem.workspace.obstacles
            if (state[1:2] ∈ obs)
                return false
            end
        end
    end

    return true
end



function simulate_forward(x0::Vector,
    control,
    dynamics::Function,
    duration::Float64;
    n_steps::Int = 20)

    # Create control interp object 
    t = collect(0.0:duration/(n_steps-1):duration)
    ctrl = [linear_interpolation(t, control[i]*ones(size(t))) for i ∈ 1:2]

    prob = ODEProblem(dynamics, x0, (0.0, duration), ctrl)
    sol = solve(prob, Tsit5(), adaptive=false, dt = duration / (n_steps - 1))

    trajectory = [sol.u[i] for i in 1:length(sol.u)]
    final_state = sol.u[end]

    return trajectory, final_state
    
end


function get_bounds(problem::TPBVP, type::Symbol)
    contraints = problem.robot.constraints 

    for con in contraints 
        if con isa BoxConstraint && con.type == type
            return con.lower, con.upper
        end
    end
end


function sample_control(lower::Vector, upper::Vector)
    return lower .+ rand(length(lower)) .* (upper .- lower)
end


function sample_SE2(r::Float64, lower::Vector, upper::Vector; center::Tuple{<:Real,<:Real}=(0, 0))
    # Position
    u = rand()
    v = rand()
    R = r .* sqrt.(u)
    ψ = 2π .* v
    x = R .* cos.(ψ) .+ center[1]
    y = R .* sin.(ψ) .+ center[2]

    # Angle
    θ = lower[3] + rand() * (upper[3] - lower[3])

    return [x, y, θ]
end




# ============================================================================ #
# Main Planner
# ============================================================================ #

"""
    KinoFMTStarPlanner(problem::TPBVP)

Execute kinodynamic FMT* planning.

Returns (path, controls, durations) if successful, otherwise nothing.
"""
function KinoFMTStarPlanner(problem::TPBVP, params::PlanningAlgorithm)

    # Extract types
    T = typeof(problem.x0)
    U = typeof(zeros(problem.m)) 
    
    # Create start node
    start_node = KinoNode(problem.x0, zeros(problem.m), 0.0)
    start_node.cost = 0.0
    start_node.in_open = true

    # Control sampling
    ctrl_lower, ctrl_upper = get_bounds(problem, :u)
    gen_ctrl() = sample_control(ctrl_lower, ctrl_upper)

    # State Sampling 
    state_lower, state_upper = get_bounds(problem, :x)
    gen_state() = sample_SE2(10.0, state_lower, state_upper) # TODO: magic numbered the radius for the workspace

    # Define goal region check and sampling
    goal_region(state) = params.distance(state, problem.xT) <= params.goal_tolerance 
    gen_goal() = sample_SE2(params.goal_tolerance , state_lower, state_upper) 

    # Collision checking 
    state_lower, state_upper = get_bounds(problem, :x)
    valid(traj) = trajectory_validation(traj, problem, state_lower, state_upper)

    # Define the node sets
    V_open = Vector{KinoNode{T, U}}([start_node])
    V_closed = KinoNode{T, U}[]
    control_duration = 1.0  # TODO make a param
    V_unvisited = generate_kinodynamic_samples(problem, params, control_duration) # TODO: check against paper repo code to make sure this is done correctly. Also make it parallel. 

    
    # Compute connection radius 
    d = problem.n
    n = params.n_samples + 1
    r_n = min(params.γ * (log(n) / n)^(1/d), Inf)
    
    goal_node = nothing
    iterations = 0
    max_iterations = 20000 # TODO make a param
    verbose = true # TODO make a param
    
    while !isempty(V_open)
        if max_iterations !== nothing && iterations >= max_iterations
            break
        end
        iterations += 1
        
        if verbose && iterations % 100 == 0
            println("Iteration $iterations, |OPEN| = $(length(V_open))")
        end
        
        # Select minimum cost node from OPEN
        z = find_min_cost_node(V_open)
        
        # Check goal
        if goal_region(z.state)
            goal_node = z
            if verbose
                println("Goal reached at iteration $iterations!")
            end
            break
        end
        
        # Find nearby nodes
        X_near = find_nearby_nodes(params, z, V_open, V_unvisited, r_n)
        
        # Move z to CLOSED
        filter!(n -> n !== z, V_open)
        push!(V_closed, z)
        z.in_open = false
        z.in_closed = true
        
        # Extend to nearby nodes
        for x in X_near
            N_x = find_closed_neighbors(params, x, V_closed, r_n)
            
            # Find best kinodynamic connection
            best_parent = find_best_kinodynamic_parent(problem, params, gen_ctrl, valid, x, N_x, control_duration) # TODO: look into how good of a job solving the TPBVP is 
            
            if best_parent !== nothing
                # Update x with connection from best_parent
                x.parent = best_parent.parent_node
                x.cost = best_parent.cost
                x.control = best_parent.control
                x.duration = best_parent.duration
                x.trajectory = best_parent.trajectory
                
                # Move to OPEN
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
    else
        if verbose
            println("No path found after $iterations iterations.")
        end
        return Solution([true], [problem.x0], [zeros(2)], [[]])
    end


    # Bypass for testing
    sol = Solution([true], [problem.x0, problem.xT], [zeros(2), zeros(2)], [[]])

    return sol
end



# ============================================================================
# Helper Functions
# ============================================================================

"""Compute connection radius based on FMT* formula."""
function compute_connection_radius(planner::KinoFMTStar, d::Int, n::Int)
    γ = planner.γ
    # FMT* radius shrinks with more samples
    return min(γ * (log(n) / n)^(1/d), Inf)
end

"""Find minimum cost node in set."""
function find_min_cost_node(nodes::Vector{KinoNode{T, U}}) where {T, U}
    return argmin(n -> n.cost, nodes)
end

"""Find nodes near given node."""
function find_nearby_nodes(
    params::PlanningAlgorithm,
    node::KinoNode{T, U},
    V_open::Vector{<:KinoNode},
    V_unvisited::Vector{<:KinoNode},
    r::Float64
) where {T, U}
    
    nearby = KinoNode{T, U}[]
    
    for x in V_open
        if x !== node && params.distance(node.state, x.state) <= r
            push!(nearby, x)
        end
    end
    
    for x in V_unvisited
        if params.distance(node.state, x.state) <= r
            push!(nearby, x)
        end
    end
    
    return nearby
end

"""Find closed neighbors of node."""
function find_closed_neighbors(
    params::PlanningAlgorithm,
    node::KinoNode{T, U},
    V_closed::Vector{KinoNode{T, U}},
    radius::Float64
) where {T, U}
    
    return filter(y -> params.distance(node.state, y.state) <= radius, V_closed)
end

"""
Find best kinodynamic parent by simulating controls.

Returns NamedTuple with (parent_node, cost, control, duration, trajectory) or nothing.
"""
function find_best_kinodynamic_parent(
    problem::TPBVP, params::PlanningAlgorithm,
    control_sampler::Function,
    valid::Function,
    target::KinoNode{T, U},
    candidates::Vector{KinoNode{T, U}},
    control_duration::Float64;
    num_control_attempts::Int = 20
) where {T, U}
    
    best_connection = nothing
    best_cost = Inf
    
    for parent in candidates
        # Try multiple controls to reach target
        for _ in 1:num_control_attempts
            control = control_sampler()
            
            # Simulate
            traj, final_state = simulate_forward(
                parent.state, control, problem.f, control_duration
            )
            
            # Check if we got close to target
            final_dist = params.distance(problem.xT, target.state)
            
            # Compute cost
            edge_cost = params.cost_function(control) # planner.control_cost(control, control_duration)
            total_cost = parent.cost + edge_cost
            
            # Check if better and collision-free
            if total_cost < best_cost && final_dist < 1.0  # Threshold for "reaching"
                if valid(traj)
                    best_cost = total_cost
                    best_connection = (
                        parent_node = parent,
                        cost = total_cost,
                        control = control,
                        duration = control_duration,
                        trajectory = traj
                    )
                end
            end
        end
    end
    
    return best_connection
end

"""Extract full kinodynamic solution."""
function extract_kinodynamic_solution(goal_node::KinoNode{T, U}) where {T, U}
    states = T[]
    controls = U[]
    durations = Float64[]
    trajectories = Vector{T}[]
    
    current = goal_node
    while current !== nothing
        pushfirst!(states, current.state)
        if current.control !== nothing
            pushfirst!(controls, current.control)
            pushfirst!(durations, current.duration)
            pushfirst!(trajectories, current.trajectory)
        end
        current = current.parent
    end
    
    return (states=states, controls=controls, durations=durations, trajectories=trajectories)
end


# # ============================================================================
# # Performance Optimizations
# # ============================================================================

# """
# Optimized version using spatial hashing for neighbor queries.
# """
# mutable struct SpatialHashGrid{T, U}
#     grid::Dict{Tuple{Int, Int}, Vector{KinoNode{T, U}}}
#     cell_size::Float64
# end

# function SpatialHashGrid{T, U}(cell_size::Float64) where {T, U}
#     return SpatialHashGrid{T, U}(Dict{Tuple{Int, Int}, Vector{KinoNode{T, U}}}(), cell_size)
# end

# """Hash state to grid cell."""
# function hash_state(grid::SpatialHashGrid, state)
#     i = floor(Int, state[1] / grid.cell_size)
#     j = floor(Int, state[2] / grid.cell_size)
#     return (i, j)
# end

# """Insert node into spatial hash."""
# function insert!(grid::SpatialHashGrid{T, U}, node::KinoNode{T, U}) where {T, U}
#     cell = hash_state(grid, node.state)
#     if !haskey(grid.grid, cell)
#         grid.grid[cell] = KinoNode{T, U}[]
#     end
#     push!(grid.grid[cell], node)
# end

# """Query nodes near a state within radius."""
# function query_radius(grid::SpatialHashGrid{T, U}, state, radius::Float64) where {T, U}
#     center_cell = hash_state(grid, state)
#     i0, j0 = center_cell
    
#     # Determine search range
#     cell_range = ceil(Int, radius / grid.cell_size)
    
#     nearby = KinoNode{T, U}[]
#     for di in -cell_range:cell_range
#         for dj in -cell_range:cell_range
#             cell = (i0 + di, j0 + dj)
#             if haskey(grid.grid, cell)
#                 for node in grid.grid[cell]
#                     if norm(node.state[1:2] .- state[1:2]) <= radius
#                         push!(nearby, node)
#                     end
#                 end
#             end
#         end
#     end
    
#     return nearby
# end


# # ============================================================================
# # Anytime FMT* Variant
# # ============================================================================

# """
#     AnytimeFMTStar

# Anytime version that improves solution quality over time.
# """
# mutable struct AnytimeFMTStar{T, U, D}
#     planner::KinoFMTStar{T, U, D}
#     best_solution::Union{Nothing, NamedTuple}
#     best_cost::Float64
# end

# function AnytimeFMTStar(planner::KinoFMTStar{T, U, D}) where {T, U, D}
#     return AnytimeFMTStar{T, U, D}(planner, nothing, Inf)
# end

# """
#     plan_anytime(anytime_planner; time_limit, iteration_limit)

# Run anytime FMT* with time or iteration limit.
# """
# function plan_anytime(
#     anytime::AnytimeFMTStar;
#     time_limit::Float64 = 10.0,
#     iteration_limit::Int = 1000
# )
    
#     start_time = time()
#     iterations = 0
    
#     while time() - start_time < time_limit && iterations < iteration_limit
#         # Run planning iteration
#         solution = plan(anytime.planner, max_iterations=100)
        
#         if solution !== nothing
#             # Compute solution cost
#             cost = compute_solution_cost(anytime.planner, solution)
            
#             if cost < anytime.best_cost
#                 anytime.best_solution = solution
#                 anytime.best_cost = cost
#                 println("Found improved solution with cost: $cost")
#             end
            
#             # Add more samples around solution path for refinement
#             add_samples_around_path(anytime.planner, solution, 50)
#         end
        
#         iterations += 100
#     end
    
#     return anytime.best_solution
# end

# """Compute total cost of solution."""
# function compute_solution_cost(planner::KinoFMTStar, solution)
#     total_cost = 0.0
#     for i in 1:length(solution.controls)
#         total_cost += planner.control_cost(solution.controls[i], solution.durations[i])
#     end
#     return total_cost
# end

# """Add samples around existing path for refinement."""
# function add_samples_around_path(
#     planner::KinoFMTStar{T, U, D},
#     solution,
#     n_new::Int
# ) where {T, U, D}
    
#     # Sample states near path
#     for _ in 1:n_new
#         idx = rand(1:length(solution.states))
#         base_state = solution.states[idx]
        
#         # Perturb state
#         noise = 0.5 * randn(length(base_state))
#         new_state = base_state .+ noise
        
#         # Try random control
#         control = planner.control_sampler(new_state)
#         traj, final_state = simulate_forward(
#             new_state, control, planner.dynamics, planner.control_duration
#         )
        
#         if planner.is_collision_free(traj)
#             node = KinoNode(final_state, control, planner.control_duration)
#             node.trajectory = traj
#             push!(planner.samples, node)
#         end
#     end
# end