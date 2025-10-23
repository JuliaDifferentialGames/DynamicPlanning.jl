"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT

Resources Used:
- Principles of Robotic Motion by Choset et al
"""

# Imports 

# Usings 
using LazySets
using Polyhedra
using LinearAlgebra
using Random
using Statistics


"""
Defines an n dimensional ball/circle
"""
function circle(center, radius)
    n = length(center)
    Ellipsoid(Vector{Float64}(center), Matrix((radius^2) * I, n, n))
end



function rand_in_set(S::LazySet; max_tries::Int=1000)
    low, high = extrema(S)
    for _ in 1:max_tries
        x = low .+ rand(2) .* (high .- low)  # random point in bounding box
        in(x, S) && return x
    end
    error("Failed to sample point inside set after $max_tries tries.")
end

"""
    random_polygon(max_sides::Int=10; max_area::Float64=10.0, bounds::AbstractPolyhedron, max_tries::Int=100, seed::Int=0)

Generate a random convex polygon with up to `max_sides` vertices,
scaled so that its area is ≤ `max_area`,
and ensure it lies entirely within the given `bounds` LazySet.
"""
function random_polygon(workspace; max_sides::Int=10, max_area::Float64=10.0,
                        max_tries::Int=100, seed::Int=0)

    seed != 0 && Random.seed!(seed)
    bounds = workspace.bounds

    for attempt in 1:max_tries
        # 1. Randomly choose number of vertices between 3 and max_sides
        n = rand(3:max_sides)

        # 2. Generate random convex polygon (points in polar coordinates)
        angles = sort(rand(n) .* 2π)
        radii = 0.5 .+ rand(n)
        points = [r .* [cos(a), sin(a)] for (r, a) in zip(radii, angles)]
        P = VPolygon(points)

        # 3. Scale area if necessary
        A = area(P)
        if A > max_area
            scale_factor = sqrt(max_area / A)
            P = scale(P, scale_factor)
            A = area(P)
        end

        # 4. Randomly translate polygon within bounds
        # Compute bounding box center of P and of bounds
        cP = mean(vertices_list(P))
        # Random translation vector within bounds
        for _ in 1:20
            # sample a random point inside bounds
            xrand = rand_in_set(bounds)
            P_shifted = LazySets.translate(P, xrand - cP)

            # 5. Check if all vertices are within bounds
            if all(in(v, bounds) for v in vertices_list(P_shifted))
                #println("Accepted polygon with $n sides, area = $(round(A, digits=3)) after $attempt attempts")
                return P_shifted
            end
        end
    end
end