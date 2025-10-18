"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

"""
Abstract task type
"""
abstract type AbstractTask end

"""
Base task type
"""
mutable struct BaseTask <: AbstractTask
    id::Int 
    priority::Int 
    deadline::Float64 
    status::Symbol
end

"""
Base task constructor
"""
function base_task(; priority=1, deadline=-1, status=:NotStarted)
    # id set to -1 since it needs to be assigned by the handler
    return BaseTask(-1, priority, deadline, status)
end