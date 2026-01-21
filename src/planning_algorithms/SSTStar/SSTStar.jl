"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Kinodynamic SST* Implementation
Based on: Li, Littlefield, Bekris (2016) "Asymptotically optimal sampling-based kinodynamic planning"

Resources Used:
- Original SST* paper (IJRR 2016)
- Your existing FMT* implementation
- Claude for programming aid
"""

# Base Includes 
include("../../tasks/navigation.jl")
include("../planning.jl")
include("../../problems/planning_problem.jl")
include("../../constraints.jl")

# Algorithm Includes
include("../../helper/nodes.jl")
include("../../helper/sampling.jl")
include("../../helper/helper.jl")
include("../../helper/steering.jl")

using Random
using NearestNeighbors
using LinearAlgebra
using .Threads: @threads

# ============================================================================ #
# Kinodynamic SST* Structure
# ============================================================================ #

"""
Stable Sparse RRT* (SST*) for kinodynamic planning without steering functions.

Key Properties:
- Asymptotically optimal
- Maintains sparse tree via pruning
- Uses best-first selection strategy
- Only requires forward propagation (no BVP solver)

Parameters:
- n_samples: Number of samples per sprint
- δ_s: Pruning radius (witness radius)
- δ_BN: Selection radius (BestNear radius)
- ξ: Shrinking factor for radii between sprints
- max_sprints: Maximum number of optimization sprints
"""
mutable struct KinoSSTStar <: PlanningAlgorithm 
    # Base Planner
    base::BasePlanner
    
    # Sampling Parameters
    n_samples::Int64
    goal_bias::Float64
    
    # SST* Specific Parameters
    δ_s::Float64          # Pruning radius (witness sample radius)
    δ_BN::Float64         # Selection radius (BestNear radius)
    ξ::Float64            # Shrinking factor (0 < ξ < 1)
    max_sprints::Int64    # Number of optimization sprints
    
    # Distance and Cost
    distance::Function
    goal_tolerance::Float64
    cost_function::Function
    heuristic::Function
    
    # Control duration
    control_duration::Float64
end

"""
Constructor for SST*
"""
function KinoSSTStar(;
    n_samples=1000,
    goal_bias=0.05,
    δ_s=0.5,
    δ_BN=1.0,
    ξ=0.9,
    max_sprints=5,
    distance=(x1, x2)->norm(x1-x2),
    goal_tolerance=1.0,
    cost_function=x->norm(x),
    heuristic=x->0.0,
    control_duration=0.25
)
    # Verify parameter relationship: δ_BN + 2*δ_s < δ (robust clearance)
    # For practical purposes, we check a relaxed version
    if δ_BN + 2*δ_s > 3.0  # Assume reasonable clearance
        @warn "Parameters may violate δ_BN + 2*δ_s < δ. Consider reducing δ_s or δ_BN"
    end
    
    base = base_planner(:KinoSSTStar, KinoSSTStarPlanner, [])
    
    return KinoSSTStar(
        base, n_samples, goal_bias,
        δ_s, δ_BN, ξ, max_sprints,
        distance, goal_tolerance,
        cost_function, heuristic,
        control_duration
    )
end

# ============================================================================ #
# Witness Structure for Sparse Representation
# ============================================================================ #

"""
Witness point for maintaining sparse tree structure.
Each witness represents a local neighborhood of radius δ_s.
"""
mutable struct Witness{T}
    location::T           # Witness location in state space
    representative::Union{KinoNode{T}, Nothing}  # Best node in this region
    
    Witness(loc::T) where T = new{T}(loc, nothing)
end

# ============================================================================ #
# SST Data Structures
# ============================================================================ #

"""
SST-specific data structures for managing active/inactive nodes and witnesses
"""
mutable struct SSTData{T, U}
    V_active::Vector{KinoNode{T, U}}      # Nodes available for selection
    V_inactive::Vector{KinoNode{T, U}}    # Pruned nodes kept for tree connectivity
    witnesses::Vector{Witness{T}}          # Witness set S
    
    # Nearest neighbor structures
    active_tree::Union{KDTree, Nothing}
    witness_tree::Union{KDTree, Nothing}
    
    SSTData{T, U}() where {T, U} = new{T, U}(
        KinoNode{T, U}[],
        KinoNode{T, U}[],
        Witness{T}[],
        nothing,
        nothing
    )
end

# ============================================================================ #
# Main SST* Planner
# ============================================================================ #

"""
    KinoSSTStarPlanner(problem, params)

Execute SST* planning with multiple optimization sprints.

Algorithm Overview:
1. Run SST for n_samples iterations with current δ_s, δ_BN
2. Shrink radii by factor ξ
3. Repeat for max_sprints sprints
4. As δ_s, δ_BN → 0, achieves asymptotic optimality

Each sprint improves solution quality while maintaining computational efficiency.
"""
function KinoSSTStarPlanner(problem::TPBVP, params::KinoSSTStar)
    Random.seed!()
    
    T = typeof(problem.x0)
    U = typeof(zeros(problem.m))
    
    # Initialize SST data structures
    sst_data = SSTData{T, U}()
    
    # Create and add start node
    start_node = KinoNode(problem.x0, zeros(problem.m), 0.0)
    start_node.cost = 0.0
    start_node.in_open = true
    push!(sst_data.V_active, start_node)
    
    # Create initial witness at start
    start_witness = Witness(problem.x0)
    start_witness.representative = start_node
    push!(sst_data.witnesses, start_witness)
    
    # Setup
    ctrl_lower, ctrl_upper = get_bounds(problem, :u)
    state_lower, state_upper = get_bounds(problem, :x)
    goal_region(state) = params.distance(state, problem.xT) <= params.goal_tolerance
    valid(traj) = trajectory_validation(traj, problem, state_lower, state_upper)
    
    # Current parameters (will shrink over sprints)
    current_δ_s = params.δ_s
    current_δ_BN = params.δ_BN
    current_n = params.n_samples
    
    best_goal_node = nothing
    
    @info "Starting SST* with $(params.max_sprints) sprints"
    
    # Multi-sprint optimization
    for sprint in 1:params.max_sprints
        @info "Sprint $sprint: δ_s=$(round(current_δ_s, digits=3)), δ_BN=$(round(current_δ_BN, digits=3)), iterations=$current_n"
        
        # Run SST for current sprint
        goal_node = run_sst_sprint!(
            problem, params, sst_data,
            current_δ_s, current_δ_BN, current_n,
            goal_region, valid
        )
        
        if goal_node !== nothing
            if best_goal_node === nothing || goal_node.cost < best_goal_node.cost
                best_goal_node = goal_node
                @info "  New best solution: cost=$(round(goal_node.cost, digits=3))"
            end
        end
        
        # Shrink radii for next sprint (approaching optimality)
        current_δ_s *= params.ξ
        current_δ_BN *= params.ξ
        
        # Increase iterations (as per SST* schedule)
        current_n = Int(ceil((1 + log(sprint)) * params.ξ^(-(problem.n + problem.m + 1) * sprint) * params.n_samples))
        
        @info "  Active nodes: $(length(sst_data.V_active)), Witnesses: $(length(sst_data.witnesses))"
    end
    
    # Extract solution
    if best_goal_node !== nothing
        @info "SST* found solution with cost $(round(best_goal_node.cost, digits=3))"
        return extract_kinodynamic_solution(best_goal_node)
    else
        @warn "SST* failed to find solution after $(params.max_sprints) sprints"
        return Solution([false], [problem.x0], [zeros(problem.m)], [[]])
    end
end

# ============================================================================ #
# Single SST Sprint
# ============================================================================ #

"""
    run_sst_sprint!(problem, params, sst_data, δ_s, δ_BN, n_iterations, goal_region, valid)

Execute one sprint of SST algorithm.

SST Algorithm per iteration:
1. BestNear Selection: Sample random state, select best-cost node within δ_BN
2. Monte Carlo Propagation: Random control, random duration
3. Local Best Check: Is new node best in its δ_s neighborhood?
4. Pruning: Remove dominated nodes, maintain sparse tree
"""
function run_sst_sprint!(
    problem::TPBVP,
    params::KinoSSTStar,
    sst_data::SSTData{T, U},
    δ_s::Float64,
    δ_BN::Float64,
    n_iterations::Int,
    goal_region::Function,
    valid::Function
) where {T, U}
    
    goal_node = nothing
    
    for iter in 1:n_iterations
        # 1. SELECTION: BestNear strategy
        x_selected = best_near_selection(sst_data, problem, params, δ_BN)
        
        if x_selected === nothing
            continue  # No nodes to select (shouldn't happen after initialization)
        end
        
        # 2. PROPAGATION: Monte Carlo random control and duration
        x_new = monte_carlo_propagation(
            x_selected, problem, params.control_duration
        )
        
        if x_new === nothing
            continue  # Propagation failed
        end
        
        # Check collision
        if !valid(x_new.trajectory)
            continue
        end
        
        # 3. LOCAL BEST CHECK: Is x_new locally the best?
        if !is_node_locally_best(x_new, sst_data, δ_s, params)
            continue  # Node is dominated in its local region
        end
        
        # 4. ADD NODE and PRUNE
        push!(sst_data.V_active, x_new)
        x_new.in_open = true
        
        prune_dominated_nodes!(x_new, sst_data, δ_s, params)
        
        # Rebuild nearest neighbor structures periodically
        if iter % 100 == 0
            rebuild_nn_structures!(sst_data)
        end
        
        # Check for goal
        if goal_region(x_new.state)
            if goal_node === nothing || x_new.cost < goal_node.cost
                goal_node = x_new
            end
        end
        
        # Progress logging
        if iter % 500 == 0
            best_cost = goal_node !== nothing ? round(goal_node.cost, digits=2) : "none"
            @info "  Iteration $iter: Active=$(length(sst_data.V_active)), Best=$best_cost"
        end
    end
    
    return goal_node
end

# ============================================================================ #
# Selection Strategy: BestNear
# ============================================================================ #

"""
    best_near_selection(sst_data, problem, params, δ_BN)

BestNear selection strategy (Algorithm 6 from paper):
1. Sample random state x_rand
2. Find all nodes X_near within δ_BN of x_rand
3. Return node with minimum cost from root

This biases selection toward good-quality paths while maintaining exploration.
"""
function best_near_selection(
    sst_data::SSTData{T, U},
    problem::TPBVP,
    params::KinoSSTStar,
    δ_BN::Float64
) where {T, U}
    
    if isempty(sst_data.V_active)
        return nothing
    end
    
    # Sample random state
    state_lower, state_upper = get_bounds(problem, :x)
    x_rand = sample_state_uniform(state_lower, state_upper)
    
    # Find near neighbors within δ_BN
    X_near = find_near_active_nodes(sst_data, x_rand, δ_BN, params.distance)
    
    # If no near neighbors, return nearest node (fallback)
    if isempty(X_near)
        return find_nearest_active_node(sst_data, x_rand, params.distance)
    end
    
    # Return node with minimum cost
    return argmin(node -> node.cost, X_near)
end

"""
Find nodes within radius of target state
"""
function find_near_active_nodes(
    sst_data::SSTData{T, U},
    target::T,
    radius::Float64,
    distance::Function
) where {T, U}
    
    return filter(node -> distance(node.state, target) <= radius, sst_data.V_active)
end

"""
Find nearest active node
"""
function find_nearest_active_node(
    sst_data::SSTData{T, U},
    target::T,
    distance::Function
) where {T, U}
    
    if isempty(sst_data.V_active)
        return nothing
    end
    
    return argmin(node -> distance(node.state, target), sst_data.V_active)
end

# ============================================================================ #
# Propagation: Monte Carlo
# ============================================================================ #

"""
    monte_carlo_propagation(parent, problem, max_duration)

Monte Carlo propagation (Algorithm 3 from paper):
1. Sample random duration t ∈ [0, max_duration]
2. Sample random control u(t)
3. Forward integrate dynamics

This random propagation is key to achieving optimality without BVP solver.
"""
function monte_carlo_propagation(
    parent::KinoNode{T, U},
    problem::TPBVP,
    max_duration::Float64
) where {T, U}
    
    # 1. Sample random duration
    duration = rand() * max_duration
    
    if duration < 1e-6
        return nothing
    end
    
    # 2. Sample random piecewise constant control
    ctrl_lower, ctrl_upper = get_bounds(problem, :u)
    n_steps = max(1, Int(ceil(duration / 0.05)))  # ~20Hz control
    dt = duration / n_steps
    
    controls = []
    for _ in 1:n_steps
        u = sample_control_uniform(ctrl_lower, ctrl_upper)
        push!(controls, u)
    end
    
    # 3. Forward integrate
    try
        x_current = copy(parent.state)
        trajectory = [copy(x_current)]
        
        for u in controls
            # Simple Euler integration (replace with your dynamics)
            x_next = forward_propagate(problem, x_current, u, dt)
            push!(trajectory, copy(x_next))
            x_current = x_next
        end
        
        # Create new node
        x_new = KinoNode(x_current, controls[end], duration)
        x_new.parent = parent
        x_new.cost = parent.cost + compute_trajectory_cost(trajectory, controls, duration)
        x_new.trajectory = trajectory
        x_new.control = controls[end]  # Store final control
        x_new.duration = duration
        
        return x_new
        
    catch e
        @debug "Propagation failed: $e"
        return nothing
    end
end

"""
Simple forward propagation (replace with your actual dynamics)
"""
function forward_propagate(problem::TPBVP, x::T, u::U, dt::Float64) where {T, U}
    # This should call your actual dynamics function
    # For now, simple Euler: x_next = x + f(x,u)*dt
    x_dot = problem.f(x, u)  # Assuming problem has dynamics function
    return x + x_dot * dt
end

"""
Compute trajectory cost
"""
function compute_trajectory_cost(trajectory, controls, duration)
    # Simple time-based cost (replace with your cost function)
    return duration
end

# ============================================================================ #
# Pruning Operations
# ============================================================================ #

"""
    is_node_locally_best(x_new, sst_data, δ_s, params)

Check if x_new is locally the best node (Algorithm 7):
1. Find nearest witness s_new
2. If no witness within δ_s, create new witness
3. Compare with current representative of s_new
4. Return true if x_new has better cost

Maintains invariant: each witness has one best representative within δ_s.
"""
function is_node_locally_best(
    x_new::KinoNode{T, U},
    sst_data::SSTData{T, U},
    δ_s::Float64,
    params::KinoSSTStar
) where {T, U}
    
    # Find nearest witness
    s_new, dist = find_nearest_witness(sst_data, x_new.state, params.distance)
    
    # If no witness within δ_s, create new witness
    if s_new === nothing || dist > δ_s
        new_witness = Witness(x_new.state)
        new_witness.representative = x_new
        push!(sst_data.witnesses, new_witness)
        return true
    end
    
    # Compare with current representative
    x_peer = s_new.representative
    
    if x_peer === nothing || x_new.cost < x_peer.cost
        return true
    end
    
    return false
end

"""
    prune_dominated_nodes!(x_new, sst_data, δ_s, params)

Prune nodes dominated by x_new (Algorithm 8):
1. Find witness s_new of x_new and its old representative x_peer
2. Move x_peer: V_active → V_inactive
3. Update s_new.representative = x_new
4. If x_peer is leaf, remove it and cascade upward through inactive parents
"""
function prune_dominated_nodes!(
    x_new::KinoNode{T, U},
    sst_data::SSTData{T, U},
    δ_s::Float64,
    params::KinoSSTStar
) where {T, U}
    
    # Find witness and old representative
    s_new, _ = find_nearest_witness(sst_data, x_new.state, params.distance)
    
    if s_new === nothing
        return  # No witness (shouldn't happen)
    end
    
    x_peer = s_new.representative
    
    if x_peer === nothing || x_peer === x_new
        s_new.representative = x_new
        return
    end
    
    # Move x_peer to inactive
    filter!(n -> n !== x_peer, sst_data.V_active)
    push!(sst_data.V_inactive, x_peer)
    x_peer.in_open = false
    
    # Update representative
    s_new.representative = x_new
    
    # Cascade removal of inactive leaf nodes
    current = x_peer
    while is_leaf(current) && current in sst_data.V_inactive
        parent = current.parent
        if parent === nothing
            break
        end
        
        # Remove current from inactive
        filter!(n -> n !== current, sst_data.V_inactive)
        
        current = parent
    end
end

"""
Check if node is a leaf (has no children)
"""
function is_leaf(node::KinoNode{T, U}) where {T, U}
    # Simple check: a node is a leaf if it's not anyone's parent
    # This could be optimized with explicit child tracking
    return true  # Placeholder - implement proper check
end

"""
Find nearest witness to target state
"""
function find_nearest_witness(
    sst_data::SSTData{T, U},
    target::T,
    distance::Function
) where {T, U}
    
    if isempty(sst_data.witnesses)
        return nothing, Inf
    end
    
    min_dist = Inf
    nearest = nothing
    
    for witness in sst_data.witnesses
        d = distance(witness.location, target)
        if d < min_dist
            min_dist = d
            nearest = witness
        end
    end
    
    return nearest, min_dist
end

# ============================================================================ #
# Helper Functions
# ============================================================================ #

"""
Sample uniform state
"""
function sample_state_uniform(lower::AbstractVector, upper::AbstractVector)
    return lower .+ rand(length(lower)) .* (upper .- lower)
end

"""
Sample uniform control
"""
function sample_control_uniform(lower::AbstractVector, upper::AbstractVector)
    return lower .+ rand(length(lower)) .* (upper .- lower)
end

"""
Rebuild nearest neighbor structures for efficiency
"""
function rebuild_nn_structures!(sst_data::SSTData{T, U}) where {T, U}
    # Build KDTree for active nodes if enough nodes exist
    if length(sst_data.V_active) > 10
        active_states = reduce(hcat, [node.state for node in sst_data.V_active])
        sst_data.active_tree = KDTree(active_states)
    end
    
    # Build KDTree for witnesses
    if length(sst_data.witnesses) > 10
        witness_locs = reduce(hcat, [w.location for w in sst_data.witnesses])
        sst_data.witness_tree = KDTree(witness_locs)
    end
end

# ============================================================================ #
# Solution Extraction (reuse from FMT)
# ============================================================================ #

"""
Extract solution from goal node
"""
function extract_kinodynamic_solution(goal_node::KinoNode{T, U}) where {T, U}
    path = T[]
    controls = U[]
    trajectories = Vector{Vector{T}}()
    
    current = goal_node
    while current !== nothing
        pushfirst!(path, current.state)
        if current.parent !== nothing
            pushfirst!(controls, current.control)
            pushfirst!(trajectories, current.trajectory)
        end
        current = current.parent
    end
    
    return Solution([true], path, controls, trajectories)
end