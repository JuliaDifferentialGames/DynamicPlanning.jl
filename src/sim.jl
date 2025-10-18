"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 


# Usings


"""
Base Simulation type
"""
struct Simulation
    workspace::Workspace
    robots::Vector{Robot}
    planning_alg::Vector{PlanningAlgorithm}
    T::Float64
    dt::Float64
end

"""
Results
"""
struct Results
    complete::Bool   # Whether all tasks were completed
    X::Vector        # Collection of robot states
    Y::Vector        # Collection of robot control
    metrics::Vector  # Custom metrics
end

"""
Simulation 
"""
function simulate(simulation::Simulation; max_iters = Inf)
    # Setup 
    # perform task allocation

    # Simulation loop
    while k <= max_iters


    end

    return 42 # Return a Results object

end