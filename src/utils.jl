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
function random_polygon(bounds; max_sides::Int=10, max_area::Float64=10.0,
                        max_tries::Int=100, seed::Int=0)

    seed != 0 && Random.seed!(seed)
    #bounds = workspace.bounds

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


"""
    create_pursuit_evasion_obstacles(;
        workspace_radius = 10.0,
        num_obstacles = 8,
        obstacle_max_sides = 6,
        obstacle_min_area = 1.0,
        obstacle_max_area = 5.0,
        min_obstacle_separation = 2.0,
        seed = 42
    )

Create polygonal obstacles for pursuit-evasion scenarios within a circular workspace.
Uses fixed seed for consistency across runs.

# Arguments
- `workspace_radius`: Radius of circular workspace centered at origin
- `num_obstacles`: Number of random polygonal obstacles to generate
- `obstacle_max_sides`: Maximum number of sides per obstacle polygon
- `obstacle_min_area`: Minimum area for each obstacle
- `obstacle_max_area`: Maximum area for each obstacle
- `min_obstacle_separation`: Minimum distance between obstacle centers
- `seed`: Random seed for reproducibility (default: 42)

# Returns
Vector of convex polygonal obstacles (VPolygon)
"""
function create_pursuit_evasion_obstacles(workspace;
    workspace_radius = 10.0,
    num_obstacles = 15,
    obstacle_max_sides = 10,
    obstacle_min_area = 1.0,
    obstacle_max_area = 5.0,
    min_obstacle_separation = 2.0,
    seed = 42
)
    
    # Set seed for consistency
    Random.seed!(seed)
    
    # Generate obstacles in inner region (leave boundary clear)
    obstacle_region = circle(zeros(2), 0.9 * workspace_radius)
    
    obstacles = []
    obstacle_centers = []
    
    for i in 1:num_obstacles
        max_attempts = 100
        placed = false
        
        for attempt in 1:max_attempts
            # Generate random polygon
            poly = random_polygon(obstacle_region; 
                                 max_sides=obstacle_max_sides,
                                 max_area=obstacle_max_area,
                                 max_tries=100,
                                 seed=0)  # Don't reset seed inside
            
            # Check area constraint
            poly_area = area(poly)
            if poly_area < obstacle_min_area
                continue  # Reject if too small
            end
            
            # Check separation from existing obstacles
            poly_center = mean(vertices_list(poly))
            
            if isempty(obstacle_centers) || 
               all(norm(poly_center - c) >= min_obstacle_separation for c in obstacle_centers)
                push!(obstacles, poly)
                push!(obstacle_centers, poly_center)
                placed = true
                break
            end
        end
        
        if !placed
            @warn "Could not place obstacle $i with required separation after $max_attempts attempts"
        end
    end
    
    return obstacles
end