"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Includes 
include("../src/workspace.jl")
include("problems/planning_problem.jl")

# Usings 
using Plots 
using LaTeXStrings




subscripts = Dict(
    '0'=>'₀', '1'=>'₁', '2'=>'₂', '3'=>'₃', '4'=>'₄',
    '5'=>'₅', '6'=>'₆', '7'=>'₇', '8'=>'₈', '9'=>'₉'
)

function subscript_digits(n::Integer)
    return join(subscripts[d] for d in string(n))
end

"""
Workspace plotting function
"""
function plot(workspace::Workspace, label=true, text_sizes=[7, 5])
    # Default plot function
    p = Plots.plot(aspectratio=1)

    # Plot the workspace
    plot!(p, workspace.bounds, c=:white)

    # Plot the obstacles 
    for (i, obstacle) ∈ enumerate(workspace.obstacles)
        plot!(p, obstacle, c=:grey)
        # if label
        #     center = chebyshevcenter(obstacle)#LazySets.center(obstacle)
        #     annotate!(p, center[1], center[2], text(L"\mathcal{WO}" * LaTeXString(subscript_digits(i)), text_sizes[1]))
        # end
    end

    # Plot robot related stuff
    for (i, robot) ∈ enumerate(workspace.robots)
        # Nav goals 
        

        # Terminal state
        plot!(p, robot.shape)
        if label
            center = robot.X[end]
            annotate!(p, center[1], center[2], text(LaTeXString("$i"), text_sizes[2]))
        end
    end

    # Get bounds 
    box = box_approximation(workspace.bounds)
    l = low(box)
    h = high(box)

    plot!(p, xlims=[l[1], h[1]], ylims=[l[2], h[2]])


    return p

end



"""
Solution plotting function
"""
function plot(problem::PlanningProblem, solution::Solution, label=true, text_sizes=[7, 5])
    # Default plot function
    p = Plots.plot(aspectratio=1)

    # Get the workspace 
    workspace = problem.workspace

    # Plot the workspace
    #plot!(p, workspace.bounds, c=:white)

    # Plot the obstacles 
    for (i, obstacle) ∈ enumerate(workspace.obstacles)
        plot!(p, obstacle, c=:grey)
        # if label
        #     center = LazySets.center(obstacle)
        #     annotate!(p, center[1], center[2], text(L"\mathcal{WO}" * LaTeXString(subscript_digits(i)), text_sizes[1]))
        # end
    end

    # Plot robot related stuff
    for (i, robot) ∈ enumerate(workspace.robots)
        # Nav goals 
        # Get xs and ys 
        xs = [robot.X[i][1] for i ∈ eachindex(robot.X)]
        ys = [robot.X[i][2] for i ∈ eachindex(robot.X)]
        
        scatter!([robot.X[1][1]], [robot.X[1][2]], label=nothing, color=i)
        plot!(xs, ys, label="Robot $i Path", color=i) 
        

        # Terminal state
        plot!(p, robot.shape, color=i)
        if label
            center = problem.workspace.robots[i].X[end]
            annotate!(p, center[1], center[2], text(LaTeXString("$i"), text_sizes[2]))
        end
    end


    plot!(p, xlims=:auto, ylims=:auto)


    return p

end