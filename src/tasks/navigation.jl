"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("task.jl")

# Usings

"""
Navigation task object
"""
mutable struct NavigationTask <: AbstractTask
    base::BaseTask
    goal::AbstractVector 
    tolerance::Float64
end

"""
Navigation object constructor
"""
function navigation_task(goal; tolerance=1e-2, priority=1, deadline=-1.0)
    # Initialize the base task 
    base = base_task(priority=priority, deadline=deadline)

    # Create the nav task 
    task = NavigationTask(base, goal, tolerance)

    return task
end