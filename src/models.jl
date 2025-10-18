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
using DifferentialEquations
using Interpolations 


"""
Unicycle model 
"""
function unicycle!(du, u, p, t)    
    # Velocities
    du[1] = p[1](t) * cos(u[3])
    du[2] = p[1](t) * sin(u[3])
    du[3] = p[2](t) 
    
    return du
end

