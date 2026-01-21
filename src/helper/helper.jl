# ============================================================================
# Helper Functions
# ============================================================================

"""Compute connection radius based on FMT* formula."""
function compute_connection_radius(planner::PlanningAlgorithm, d::Int, n::Int) # KinoFMTStar
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

    sol = Solution([true], [states...], [controls...], [[]])
    
    return sol 
end