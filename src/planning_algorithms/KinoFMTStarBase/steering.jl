



# ============================================================================ #
# Sampling-Based TPBVP Solver (Steering Function)
# ============================================================================ #

"""
    solve_tpbvp_sampling(x_start, x_goal, problem, params, control_duration; n_attempts)

Solve Two-Point Boundary Value Problem using sampling-based approach.

This is the steering function for kinodynamic planning:
- Samples multiple control sequences
- Simulates forward from x_start
- Returns best control that gets closest to x_goal
- Checks collision along trajectory

Returns: NamedTuple with (control, trajectory, final_state, cost, success) or nothing
"""
function solve_tpbvp_sampling(
    x_start::Vector,
    x_goal::Vector,
    problem::TPBVP,
    params::PlanningAlgorithm,
    control_duration::Float64;
    n_attempts::Int = 50,
    distance_threshold::Float64 = 1.0  # How close is "close enough"
)
    # Get bounds and validation
    ctrl_lower, ctrl_upper = get_bounds(problem, :u)
    state_lower, state_upper = get_bounds(problem, :x)
    valid(traj) = trajectory_validation(traj, problem, state_lower, state_upper)
    
    best_connection = nothing
    best_distance = Inf
    
    for attempt in 1:n_attempts
        # Sample random control
        control = sample_control(ctrl_lower, ctrl_upper)
        
        # Simulate forward
        traj, final_state = simulate_forward(
            x_start, control, problem.f, control_duration
        )
        
        # Check collision
        if !valid(traj)
            continue
        end
        
        # Compute distance to goal
        dist = params.distance(final_state, x_goal)
        
        # Compute trajectory cost (time-based for now)
        traj_cost = norm(control) #control_duration  # Could also use ∫||u||²dt
        
        # Update best if closer
        if dist < best_distance
            best_distance = dist
            best_connection = (
                control = control,
                trajectory = traj,
                final_state = final_state,
                cost = traj_cost,
                success = dist <= distance_threshold,
                distance = dist
            )
            
            # Early exit if good enough
            if dist <= distance_threshold
                break
            end
        end
    end
    
    return best_connection
end


"""
    solve_tpbvp_informed(x_start, x_goal, problem, params, control_duration; n_attempts)

Improved TPBVP solver with informed sampling (optional enhancement).

Uses simple heuristic to bias control samples toward goal direction.
"""
function solve_tpbvp_informed(
    x_start::Vector,
    x_goal::Vector,
    problem::TPBVP,
    params::PlanningAlgorithm,
    control_duration::Float64;
    n_attempts::Int = 50,
    distance_threshold::Float64 = 1.0,
    informed_ratio::Float64 = 0.3  # 30% informed, 70% random
)
    ctrl_lower, ctrl_upper = get_bounds(problem, :u)
    state_lower, state_upper = get_bounds(problem, :x)
    valid(traj) = trajectory_validation(traj, problem, state_lower, state_upper)
    
    best_connection = nothing
    best_distance = Inf
    
    # Compute direction to goal
    Δpos = x_goal[1:2] - x_start[1:2]
    desired_direction = atan(Δpos[2], Δpos[1])
    
    for attempt in 1:n_attempts
        # Mix informed and random sampling
        if rand() < informed_ratio
            # Informed control: bias toward goal
            v = rand() * (ctrl_upper[1] - ctrl_lower[1]) + ctrl_lower[1]
            
            # Angle error
            θ_error = desired_direction - x_start[3]
            θ_error = atan(sin(θ_error), cos(θ_error))  # Wrap to [-π, π]
            
            # Proportional control + noise
            ω = clamp(2.0 * θ_error, ctrl_lower[2], ctrl_upper[2]) + 0.3 * randn()
            control = [v, ω]
        else
            # Random control
            control = sample_control(ctrl_lower, ctrl_upper)
        end
        
        # Simulate and check
        traj, final_state = simulate_forward(
            x_start, control, problem.f, control_duration
        )
        
        if !valid(traj)
            continue
        end
        
        dist = params.distance(final_state, x_goal)
        traj_cost = control_duration
        
        if dist < best_distance
            best_distance = dist
            best_connection = (
                control = control,
                trajectory = traj,
                final_state = final_state,
                cost = traj_cost,
                success = dist <= distance_threshold,
                distance = dist
            )
            
            if dist <= distance_threshold
                break
            end
        end
    end
    
    return best_connection
end
