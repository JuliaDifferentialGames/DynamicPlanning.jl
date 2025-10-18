"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("../tasks/navigation.jl")
include("planning.jl")

# Usings 

"""
A very dumb algorithm that goes to the goal and ignores obstacles
"""
mutable struct DumbBug
    base::PlanningAlgorithm
    task::NavigationTask
    dx::Vector 
end

"""
DumbBug Constructor
"""
function dumb_bug(task; dx=0.05)
    # Make the base 
    base = base_planner(:DumbBug, dumb_bug_planner, [])

    return DumbBug(base, task, dx)
end


function dumb_bug_planner()
    return 42
end