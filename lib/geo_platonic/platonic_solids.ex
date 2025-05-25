# lib/geo_platonic/platonic_solids.ex
defmodule GeoPlatonic.PlatonicSolids do
  @moduledoc """
  Module for generating mathematically accurate platonic solids on a sphere based on a chosen point.

  This module provides functions to create the five platonic solids:
  - Tetrahedron (4 vertices)
  - Cube/Hexahedron (8 vertices)
  - Octahedron (6 vertices)
  - Dodecahedron (20 vertices)
  - Icosahedron (12 vertices)

  Each solid is generated so that one of its vertices is at the selected point,
  with the remaining vertices positioned to create a mathematically precise regular solid.
  """

  @golden_ratio (1 + :math.sqrt(5)) / 2

defp is_at_pole(lat) do
  abs(abs(lat) - 90) < 0.001
end


@doc """
Modifies the tetrahedron generation to handle pole cases, using longitude as a rotation angle.
"""
def generate_tetrahedron(lat, lng) do
  if is_at_pole(lat) do
    # For poles, we'll create a standard tetrahedron and then rotate it around the pole axis
    # First, determine if we're at North or South pole
    pole_direction = if lat > 0, do: {0, 0, 1}, else: {0, 0, -1}

    # Create a standard tetrahedron with one vertex at the pole
    # For a tetrahedron, we can start with vertices at:
    # - The pole
    # - Three points equally spaced on a circle below/above the pole

    # Convert longitude to radians for the rotation
    lng_rad = lng * :math.pi() / 180

    # Create the pole vertex
    v1 = pole_direction

    # Create the other three vertices equidistant from the pole
    # The angle between any two vertices from the center is arccos(-1/3) ≈ 109.47°
    tetra_angle = :math.acos(-1/3)

    # Distance from the center to the equator (perpendicular to pole axis)
    equator_dist = :math.sin(tetra_angle)

    # Distance along the pole axis
    pole_dist = :math.cos(tetra_angle) * elem(pole_direction, 2)

    # Generate the three vertices with 120° spacing around the pole axis
    # but include the longitude as a starting angle
    v2 = {
      equator_dist * :math.cos(lng_rad),
      equator_dist * :math.sin(lng_rad),
      pole_dist
    }

    v3 = {
      equator_dist * :math.cos(lng_rad + 2 * :math.pi() / 3),
      equator_dist * :math.sin(lng_rad + 2 * :math.pi() / 3),
      pole_dist
    }

    v4 = {
      equator_dist * :math.cos(lng_rad + 4 * :math.pi() / 3),
      equator_dist * :math.sin(lng_rad + 4 * :math.pi() / 3),
      pole_dist
    }

    # Normalize all vertices to ensure they're exactly on the unit sphere
    v1 = normalize(v1)
    v2 = normalize(v2)
    v3 = normalize(v3)
    v4 = normalize(v4)

    # Convert to latitude/longitude
    [
      cartesian_to_lat_lng(v1),
      cartesian_to_lat_lng(v2),
      cartesian_to_lat_lng(v3),
      cartesian_to_lat_lng(v4)
    ]
  else
    # For non-pole locations, use the original function implementation
    # [original function body here]
    # First vertex at the specified point
    v1_lat_rad = lat * :math.pi() / 180
    v1_lng_rad = lng * :math.pi() / 180
    v1 = {
      :math.cos(v1_lat_rad) * :math.cos(v1_lng_rad),
      :math.cos(v1_lat_rad) * :math.sin(v1_lng_rad),
      :math.sin(v1_lat_rad)
    }

    # To ensure an edge is pointing up, we'll create a local coordinate system
    # with v1 as one axis, and determine the other three vertices

    # For consistent orientation, find a vector that's pointing "up" relative to v1
    # We can use the north pole (0, 0, 1) and project it to be perpendicular to v1
    north = {0, 0, 1}

    # Remove the component of north that's parallel to v1
    dot_v1_north = dot_product(v1, north)
    up_direction = {
      elem(north, 0) - dot_v1_north * elem(v1, 0),
      elem(north, 1) - dot_v1_north * elem(v1, 1),
      elem(north, 2) - dot_v1_north * elem(v1, 2)
    }

    # If v1 is at the north or south pole, use a different reference vector
    up_length = :math.sqrt(
      elem(up_direction, 0) * elem(up_direction, 0) +
      elem(up_direction, 1) * elem(up_direction, 1) +
      elem(up_direction, 2) * elem(up_direction, 2)
    )

    up_direction = if up_length < 0.000001 do
      # If v1 is at/near a pole, use the east direction instead
      east = {1, 0, 0}
      {
        elem(east, 0) - dot_product(v1, east) * elem(v1, 0),
        elem(east, 1) - dot_product(v1, east) * elem(v1, 1),
        elem(east, 2) - dot_product(v1, east) * elem(v1, 2)
      }
    else
      up_direction
    end

    up_direction = normalize(up_direction)

    # Get a third perpendicular vector to complete our local coordinate system
    # right_direction = cross_product(v1, up_direction)
    # right_direction = normalize(right_direction)

    # In a regular tetrahedron, the angle between any two vertices from the center is
    # approximately 109.47 degrees (arccos(-1/3))
    tetra_angle = :math.acos(-1/3)

    # The distance from the center to each vertex in a regular tetrahedron
    # inscribed in a unit sphere is 1

    # Now we'll place the remaining 3 vertices to form a regular tetrahedron
    # Calculate the components for the remaining vertices

    # For a regular tetrahedron, we want vertices equally spaced
    # We'll use spherical coordinates to position them

    # The second vertex will be placed in the "up" direction from v1
    # at the correct angle to form a regular tetrahedron

    # Component of v2 in the direction of v1
    v1_component = :math.cos(tetra_angle)

    # Component of v2 perpendicular to v1
    perp_component = :math.sin(tetra_angle)

    # Calculate the second vertex - this will be "up" from v1
    v2 = {
      v1_component * elem(v1, 0) + perp_component * elem(up_direction, 0),
      v1_component * elem(v1, 1) + perp_component * elem(up_direction, 1),
      v1_component * elem(v1, 2) + perp_component * elem(up_direction, 2)
    }

    # For the third and fourth vertices, we need to rotate around the axis from origin to v1
    # They should be 120 degrees apart when viewed from the direction of v1

    # Calculate the rotation for v3 (120 degrees from the up direction around v1)
    cos_120 = -0.5                # cos(120°)
    sin_120 = :math.sqrt(3) / 2   # sin(120°)

    # Rotate up_direction by 120 degrees around v1
    rotated_up_120 = rotate_around_axis(up_direction, v1, cos_120, sin_120)

    # Third vertex
    v3 = {
      v1_component * elem(v1, 0) + perp_component * elem(rotated_up_120, 0),
      v1_component * elem(v1, 1) + perp_component * elem(rotated_up_120, 1),
      v1_component * elem(v1, 2) + perp_component * elem(rotated_up_120, 2)
    }

    # Calculate the rotation for v4 (240 degrees from the up direction around v1)
    cos_240 = -0.5                # cos(240°) = cos(120°)
    sin_240 = -:math.sqrt(3) / 2  # sin(240°) = -sin(120°)

    # Rotate up_direction by 240 degrees around v1
    rotated_up_240 = rotate_around_axis(up_direction, v1, cos_240, sin_240)

    # Fourth vertex
    v4 = {
      v1_component * elem(v1, 0) + perp_component * elem(rotated_up_240, 0),
      v1_component * elem(v1, 1) + perp_component * elem(rotated_up_240, 1),
      v1_component * elem(v1, 2) + perp_component * elem(rotated_up_240, 2)
    }

    # Normalize all vertices to ensure they're exactly on the unit sphere
    v1 = normalize(v1)
    v2 = normalize(v2)
    v3 = normalize(v3)
    v4 = normalize(v4)

    # Convert to latitude/longitude
    [
      cartesian_to_lat_lng(v1),
      cartesian_to_lat_lng(v2),
      cartesian_to_lat_lng(v3),
      cartesian_to_lat_lng(v4)
    ]
  end
end

@doc """
Modifies the cube generation to handle pole cases, using longitude as a rotation angle.
"""
def generate_cube(lat, lng) do
  if is_at_pole(lat) do
    # For poles, generate a cube with one vertex at the pole
    # and edges aligned according to the provided longitude

    # Determine if we're at North or South pole
    is_north_pole = lat > 0
    pole_vertex = if is_north_pole, do: {0, 0, 1}, else: {0, 0, -1}

    # Convert longitude to radians
    lng_rad = lng * :math.pi() / 180

    # For a cube with one vertex at the pole, we need to calculate the other vertices
    # The cube vertices are formed by a central vertex at the pole
    # and three neighboring vertices at the same geodesic distance from the pole

    # The angle between the pole and each neighbor in a cube is arccos(1/sqrt(3)) ≈ 54.7°
    cube_angle = :math.acos(1/:math.sqrt(3))

    # Calculate the height component (distance along pole axis)
    height_component = if is_north_pole do
      :math.cos(cube_angle)
    else
      -:math.cos(cube_angle)
    end

    # Distance from central axis (perpendicular to pole axis)
    radius_component = :math.sin(cube_angle)

    # Generate the three vertices with 120° spacing around the pole axis
    # but include the longitude as a starting angle
    v2 = {
      radius_component * :math.cos(lng_rad),
      radius_component * :math.sin(lng_rad),
      height_component
    }

    v3 = {
      radius_component * :math.cos(lng_rad + 2 * :math.pi() / 3),
      radius_component * :math.sin(lng_rad + 2 * :math.pi() / 3),
      height_component
    }

    v4 = {
      radius_component * :math.cos(lng_rad + 4 * :math.pi() / 3),
      radius_component * :math.sin(lng_rad + 4 * :math.pi() / 3),
      height_component
    }

    # Generate the opposite pole vertex and its three neighbors
    opposite_pole = {-elem(pole_vertex, 0), -elem(pole_vertex, 1), -elem(pole_vertex, 2)}

    v5 = {
      radius_component * :math.cos(lng_rad),
      radius_component * :math.sin(lng_rad),
      -height_component
    }

    v6 = {
      radius_component * :math.cos(lng_rad + 2 * :math.pi() / 3),
      radius_component * :math.sin(lng_rad + 2 * :math.pi() / 3),
      -height_component
    }

    v7 = {
      radius_component * :math.cos(lng_rad + 4 * :math.pi() / 3),
      radius_component * :math.sin(lng_rad + 4 * :math.pi() / 3),
      -height_component
    }

    # Normalize all vertices to ensure they're exactly on the unit sphere
    vertices = [pole_vertex, v2, v3, v4, opposite_pole, v5, v6, v7]
    vertices = Enum.map(vertices, &normalize/1)

    # Convert to latitude/longitude
    Enum.map(vertices, &cartesian_to_lat_lng/1)
  else
    # For non-pole locations, use the original function implementation
    # Get the vertices of a tetrahedron with one vertex at the specified point
    tetrahedron_vertices = generate_tetrahedron(lat, lng)

    # Get the vertices of the facing tetrahedron
    facing_vertices = generate_facing_tetrahedron(lat, lng)

    # Convert the specified point to Cartesian coordinates
    specified_point_lat_rad = lat * :math.pi() / 180
    specified_point_lng_rad = lng * :math.pi() / 180
    specified_point = {
      :math.cos(specified_point_lat_rad) * :math.cos(specified_point_lng_rad),
      :math.cos(specified_point_lat_rad) * :math.sin(specified_point_lng_rad),
      :math.sin(specified_point_lat_rad)
    }

    # Find the antipode in Cartesian coordinates
    antipode = {
      -elem(specified_point, 0),
      -elem(specified_point, 1),
      -elem(specified_point, 2)
    }

    # Define axis from the specified point to its antipode
    axis = normalize(specified_point)

    # Define rotation angle (60 degrees in radians)
    angle = 60 * :math.pi() / 180
    cos_angle = :math.cos(angle)
    sin_angle = :math.sin(angle)

    # Convert all vertices to Cartesian for rotation
    cartesian_tetrahedron = Enum.map(tetrahedron_vertices, fn vertex ->
      lat_rad = vertex.lat * :math.pi() / 180
      lng_rad = vertex.lng * :math.pi() / 180
      {
        :math.cos(lat_rad) * :math.cos(lng_rad),
        :math.cos(lat_rad) * :math.sin(lng_rad),
        :math.sin(lat_rad)
      }
    end)

    cartesian_facing = Enum.map(facing_vertices, fn vertex ->
      lat_rad = vertex.lat * :math.pi() / 180
      lng_rad = vertex.lng * :math.pi() / 180
      {
        :math.cos(lat_rad) * :math.cos(lng_rad),
        :math.cos(lat_rad) * :math.sin(lng_rad),
        :math.sin(lat_rad)
      }
    end)

    # Rotate all vertices except the specified point and its antipode
    rotated_tetrahedron = Enum.map(cartesian_tetrahedron, fn vertex ->
      distance_to_specified = vector_distance(vertex, specified_point)
      distance_to_antipode = vector_distance(vertex, antipode)

      if distance_to_specified < 0.0001 || distance_to_antipode < 0.0001 do
        # Don't rotate the specified point or its antipode
        vertex
      else
        # Rotate around the axis
        rotate_around_axis(vertex, axis, cos_angle, sin_angle)
      end
    end)

    rotated_facing = Enum.map(cartesian_facing, fn vertex ->
      distance_to_specified = vector_distance(vertex, specified_point)
      distance_to_antipode = vector_distance(vertex, antipode)

      if distance_to_specified < 0.0001 || distance_to_antipode < 0.0001 do
        # Don't rotate the specified point or its antipode
        vertex
      else
        # Rotate around the axis
        rotate_around_axis(vertex, axis, cos_angle, sin_angle)
      end
    end)

    # Convert back to lat/lng
    final_tetrahedron = Enum.map(rotated_tetrahedron, &cartesian_to_lat_lng/1)
    final_facing = Enum.map(rotated_facing, &cartesian_to_lat_lng/1)

    # Combine the vertices to form a cube
    final_tetrahedron ++ final_facing
  end
end

@doc """
Modifies the octahedron generation to handle pole cases, using longitude as a rotation angle.
"""
def generate_octahedron(lat, lng) do
  if is_at_pole(lat) do
    # For poles, generate an octahedron with one vertex at the pole
    # and edges aligned according to the provided longitude

    # Determine if we're at North or South pole
    is_north_pole = lat > 0
    pole_vertex = if is_north_pole, do: {0, 0, 1}, else: {0, 0, -1}
    opposite_pole = {-elem(pole_vertex, 0), -elem(pole_vertex, 1), -elem(pole_vertex, 2)}

    # Convert longitude to radians
    lng_rad = lng * :math.pi() / 180

    # For an octahedron with one vertex at the pole, the other 4 vertices
    # lie on the equator at 90° to each other, with longitude as starting angle

    # Generate the four equatorial vertices
    v1 = {
      :math.cos(lng_rad),
      :math.sin(lng_rad),
      0
    }

    v2 = {
      :math.cos(lng_rad + :math.pi()/2),
      :math.sin(lng_rad + :math.pi()/2),
      0
    }

    v3 = {
      :math.cos(lng_rad + :math.pi()),
      :math.sin(lng_rad + :math.pi()),
      0
    }

    v4 = {
      :math.cos(lng_rad + 3 * :math.pi()/2),
      :math.sin(lng_rad + 3 * :math.pi()/2),
      0
    }

    # Normalize all vertices to ensure they're exactly on the unit sphere
    vertices = [pole_vertex, v1, v2, v3, v4, opposite_pole]
    vertices = Enum.map(vertices, &normalize/1)

    # Convert to latitude/longitude
    Enum.map(vertices, &cartesian_to_lat_lng/1)
  else
    # For non-pole locations, use the original function implementation
    # [original octahedron generation code]
    # First vertex at the specified point
    v1_lat_rad = lat * :math.pi() / 180
    v1_lng_rad = lng * :math.pi() / 180
    v1 = {
      :math.cos(v1_lat_rad) * :math.cos(v1_lng_rad),
      :math.cos(v1_lat_rad) * :math.sin(v1_lng_rad),
      :math.sin(v1_lat_rad)
    }

    # For an octahedron, we'll start with the standard form in 3D space
    # with vertices at (±1,0,0), (0,±1,0), (0,0,±1)
    standard_octahedron = [
      {1, 0, 0}, {-1, 0, 0},
      {0, 1, 0}, {0, -1, 0},
      {0, 0, 1}, {0, 0, -1}
    ]

    # We'll rotate this standard octahedron to place one vertex at v1
    # and ensure proper alignment with geographic coordinates

    # Choose the first vertex as our reference to map to v1
    reference_vertex = List.first(standard_octahedron)  # {1, 0, 0}

    # First rotation: align reference_vertex with v1
    rotation_matrix_1 = rotation_matrix_between_vectors(reference_vertex, v1)

    # Apply first rotation to all vertices
    rotated_vertices = Enum.map(standard_octahedron, fn vertex ->
      apply_rotation(rotation_matrix_1, vertex)
    end)

    # Now we need to ensure the "up" direction from v1 aligns with geographic north
    # In the standard octahedron, the "up" neighbor to {1,0,0} would be {0,0,1}

    # Define this edge as the standard "up" direction in the rotated frame
    std_up_vertex_idx = 4  # Index of {0,0,1} in standard_octahedron
    std_up_vertex = Enum.at(rotated_vertices, std_up_vertex_idx)

    # Calculate the direction from v1 to this vertex
    std_up_direction = {
      elem(std_up_vertex, 0) - elem(v1, 0),
      elem(std_up_vertex, 1) - elem(v1, 1),
      elem(std_up_vertex, 2) - elem(v1, 2)
    }
    std_up_direction = normalize(std_up_direction)

    # Define the "up" direction relative to geographic north
    north = {0, 0, 1}
    dot_v1_north = dot_product(v1, north)
    geographic_up = {
      elem(north, 0) - dot_v1_north * elem(v1, 0),
      elem(north, 1) - dot_v1_north * elem(v1, 1),
      elem(north, 2) - dot_v1_north * elem(v1, 2)
    }

    # Handle case where v1 is at a pole
    up_length = :math.sqrt(
      elem(geographic_up, 0) * elem(geographic_up, 0) +
      elem(geographic_up, 1) * elem(geographic_up, 1) +
      elem(geographic_up, 2) * elem(geographic_up, 2)
    )

    geographic_up = if up_length < 0.000001 do
      # If v1 is at/near a pole, use east instead
      east = {1, 0, 0}
      {
        elem(east, 0) - dot_product(v1, east) * elem(v1, 0),
        elem(east, 1) - dot_product(v1, east) * elem(v1, 1),
        elem(east, 2) - dot_product(v1, east) * elem(v1, 2)
      }
    else
      geographic_up
    end

    geographic_up = normalize(geographic_up)

    # Second rotation: around v1 to align std_up_direction with geographic_up
    rotation_matrix_2 = rotation_around_axis_matrix(v1, std_up_direction, geographic_up)

    # Apply second rotation to all vertices
    final_vertices = Enum.map(rotated_vertices, fn vertex ->
      apply_rotation(rotation_matrix_2, vertex)
    end)

    # Ensure all vertices are exactly on the unit sphere
    final_vertices = Enum.map(final_vertices, &normalize/1)

    # Reorder vertices to put v1 first
    # v1_idx = 0  # Index of v1 in final_vertices

    # Convert to latitude/longitude
    vertices = Enum.map(final_vertices, &cartesian_to_lat_lng/1)

    vertices
  end
end

@doc """
Modifies the dodecahedron generation to handle pole cases, using longitude as a rotation angle.
"""
def generate_dodecahedron(lat, lng) do
  if is_at_pole(lat) do
    # For poles, generate a dodecahedron with one vertex at the pole
    # and edges aligned according to the provided longitude

    # Determine if we're at North or South pole
    is_north_pole = lat > 0
    pole_vertex = if is_north_pole, do: {0, 0, 1}, else: {0, 0, -1}

    # The golden ratio
    phi = @golden_ratio

    # Convert longitude to radians
    lng_rad = lng * :math.pi() / 180

    # In a dodecahedron with one vertex at the pole, there are 5 vertices
    # in a pentagonal arrangement below/above the pole

    # The angle between the pole vertex and its neighbors is arccos(1/sqrt(5)) ≈ 63.4°
    dodeca_angle = :math.acos(1/:math.sqrt(5))

    # Calculate the height component (distance along pole axis)
    height_component = if is_north_pole do
      :math.cos(dodeca_angle)
    else
      -:math.cos(dodeca_angle)
    end

    # Distance from central axis (perpendicular to pole axis)
    radius_component = :math.sin(dodeca_angle)

    # Generate the five vertices of the first pentagonal face with the pole vertex
    pentagon_vertices = for i <- 0..4 do
      angle = lng_rad + i * 2 * :math.pi() / 5
      {
        radius_component * :math.cos(angle),
        radius_component * :math.sin(angle),
        height_component
      }
    end

    # The next layer of vertices forms another pentagon, rotated by 36°
    second_height = if is_north_pole do
      height_component * height_component - (1 - height_component * height_component) / phi
    else
      height_component * height_component + (1 - height_component * height_component) / phi
    end

    second_radius = :math.sqrt(1 - second_height * second_height)

    second_pentagon = for i <- 0..4 do
      angle = lng_rad + (i + 0.5) * 2 * :math.pi() / 5
      {
        second_radius * :math.cos(angle),
        second_radius * :math.sin(angle),
        second_height
      }
    end

    # The third layer forms a pentagon at the height of the pole but rotated
    third_height = -height_component
    third_radius = radius_component

    third_pentagon = for i <- 0..4 do
      angle = lng_rad + i * 2 * :math.pi() / 5
      {
        third_radius * :math.cos(angle),
        third_radius * :math.sin(angle),
        third_height
      }
    end

    # The fourth layer is the final pentagon before the opposite pole
    fourth_height = -second_height
    fourth_radius = second_radius

    fourth_pentagon = for i <- 0..4 do
      angle = lng_rad + (i + 0.5) * 2 * :math.pi() / 5
      {
        fourth_radius * :math.cos(angle),
        fourth_radius * :math.sin(angle),
        fourth_height
      }
    end

    # The opposite pole
    opposite_pole = {-elem(pole_vertex, 0), -elem(pole_vertex, 1), -elem(pole_vertex, 2)}

    # Combine all vertices
    vertices = [pole_vertex] ++ pentagon_vertices ++ second_pentagon ++
               third_pentagon ++ fourth_pentagon ++ [opposite_pole]

    # Normalize all vertices to ensure they're exactly on the unit sphere
    vertices = Enum.map(vertices, &normalize/1)

    # Convert to latitude/longitude
    Enum.map(vertices, &cartesian_to_lat_lng/1)
  else
    # For non-pole locations, use the original function implementation
    # [original dodecahedron generation code]
    # First vertex at the specified point
    v1_lat_rad = lat * :math.pi() / 180
    v1_lng_rad = lng * :math.pi() / 180
    v1 = {
      :math.cos(v1_lat_rad) * :math.cos(v1_lng_rad),
      :math.cos(v1_lat_rad) * :math.sin(v1_lng_rad),
      :math.sin(v1_lat_rad)
    }

    # Create a standard dodecahedron in canonical position
    phi = @golden_ratio

    # The 20 vertices of a regular dodecahedron
    # These are the standard coordinates based on the golden ratio
    standard_vertices = [
      # Vertices based on (±1, ±1, ±1)
      {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1},
      {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1},

      # Vertices based on (0, ±1/φ, ±φ)
      {0, 1/phi, phi}, {0, -1/phi, phi}, {0, 1/phi, -phi}, {0, -1/phi, -phi},

      # Vertices based on (±φ, 0, ±1/φ)
      {phi, 0, 1/phi}, {-phi, 0, 1/phi}, {phi, 0, -1/phi}, {-phi, 0, -1/phi},

      # Vertices based on (±1/φ, ±φ, 0)
      {1/phi, phi, 0}, {-1/phi, phi, 0}, {1/phi, -phi, 0}, {-1/phi, -phi, 0}
    ]

    # Normalize all vertices to the unit sphere
    standard_vertices = Enum.map(standard_vertices, &normalize/1)

    # For a dodecahedron, we need a vertex with an adjacent vertex that shares the same longitude
    # Let's examine the structure of our standard dodecahedron to find such a pair

    # Pick a reference vertex
    # Choose one from the (0, ±1/φ, ±φ) group, like {0, 1/phi, phi}
    reference_idx = 8  # Index of {0, 1/phi, phi}
    reference_vertex = Enum.at(standard_vertices, reference_idx)

    # Calculate distances between all vertices
    distances = for i <- 0..(length(standard_vertices) - 1), j <- 0..(length(standard_vertices) - 1) do
      {i, j, vector_distance(Enum.at(standard_vertices, i), Enum.at(standard_vertices, j))}
    end

    # Find the typical edge length of the dodecahedron
    # Sort by distance, skip the self-connections (distance 0), and take the smallest non-zero
    edge_length = distances
      |> Enum.filter(fn {i, j, _d} -> i != j end)
      |> Enum.sort_by(fn {_, _, d} -> d end)
      |> List.first()
      |> elem(2)

    # Find adjacent vertices to the reference vertex
    # These are vertices at approximately the edge length distance
    adjacent_indices = distances
      |> Enum.filter(fn {i, j, d} -> i == reference_idx && j != reference_idx && abs(d - edge_length) < 0.0001 end)
      |> Enum.map(fn {_, j, _} -> j end)

    # Among adjacent vertices, find one that shares the same longitude (x=0)
    # For {0, 1/phi, phi}, we want one with coordinates {0, something, something}
    same_longitude_idx = Enum.find(adjacent_indices, fn idx ->
      v = Enum.at(standard_vertices, idx)
      abs(elem(v, 0) - elem(reference_vertex, 0)) < 0.0001
    end)

    # If no vertex with same longitude found, fallback to the first adjacent vertex
    same_longitude_idx = same_longitude_idx || List.first(adjacent_indices)

    # Get the vertex that shares longitude with our reference vertex
    target_vertex = Enum.at(standard_vertices, same_longitude_idx)

    # Define the edge direction from reference vertex to the target vertex
    std_up_direction = {
      elem(target_vertex, 0) - elem(reference_vertex, 0),
      elem(target_vertex, 1) - elem(reference_vertex, 1),
      elem(target_vertex, 2) - elem(reference_vertex, 2)
    }
    std_up_direction = normalize(std_up_direction)

    # Create the "up" direction for our target point v1
    north = {0, 0, 1}
    dot_v1_north = dot_product(v1, north)
    v1_up_direction = {
      elem(north, 0) - dot_v1_north * elem(v1, 0),
      elem(north, 1) - dot_v1_north * elem(v1, 1),
      elem(north, 2) - dot_v1_north * elem(v1, 2)
    }

    # Handle the case where v1 is at a pole
    up_length = :math.sqrt(
      elem(v1_up_direction, 0) * elem(v1_up_direction, 0) +
      elem(v1_up_direction, 1) * elem(v1_up_direction, 1) +
      elem(v1_up_direction, 2) * elem(v1_up_direction, 2)
    )

    v1_up_direction = if up_length < 0.000001 do
      # If v1 is at/near a pole, use the east direction instead
      east = {1, 0, 0}
      {
        elem(east, 0) - dot_product(v1, east) * elem(v1, 0),
        elem(east, 1) - dot_product(v1, east) * elem(v1, 1),
        elem(east, 2) - dot_product(v1, east) * elem(v1, 2)
      }
    else
      v1_up_direction
    end

    v1_up_direction = normalize(v1_up_direction)

    # Now we'll move the dodecahedron in two steps:
    # 1. Rotate to place reference_vertex at v1
    # 2. Rotate around v1 to align the selected edge with the "up" direction

    # First rotation
    rotation_matrix_1 = rotation_matrix_between_vectors(reference_vertex, v1)

    # Apply the first rotation to all vertices
    rotated_vertices = Enum.map(standard_vertices, fn vertex ->
      apply_rotation(rotation_matrix_1, vertex)
    end)

    # Also rotate the standard up direction
    rotated_up = apply_rotation(rotation_matrix_1, std_up_direction)

    # Second rotation - align the up directions
    rotation_matrix_2 = rotation_around_axis_matrix(v1, rotated_up, v1_up_direction)

    # Apply the second rotation to all vertices
    final_vertices = Enum.map(rotated_vertices, fn vertex ->
      apply_rotation(rotation_matrix_2, vertex)
    end)

    # Normalize to ensure vertices are exactly on the unit sphere
    final_vertices = Enum.map(final_vertices, &normalize/1)

    # Reorder vertices to put v1 first
    # Find which vertex in final_vertices is closest to v1
    # (it should be the transformed reference_vertex, but for numerical stability we check)
    distances_to_v1 = Enum.map(final_vertices, fn vertex ->
      vector_distance(vertex, v1)
    end)

    v1_in_final_idx = Enum.find_index(distances_to_v1, fn d -> d < 0.0001 end)

    # Reorder the vertices to put v1 first
    reordered_vertices = [
      Enum.at(final_vertices, v1_in_final_idx) |
      Enum.take(final_vertices, v1_in_final_idx) ++
      Enum.drop(final_vertices, v1_in_final_idx + 1)
    ]

    # Convert to latitude/longitude
    vertices = Enum.map(reordered_vertices, &cartesian_to_lat_lng/1)

    vertices
  end
end

@doc """
Modifies the icosahedron generation to handle pole cases, using longitude as a rotation angle.
"""
def generate_icosahedron(lat, lng) do
  if is_at_pole(lat) do
    # For poles, generate an icosahedron with one vertex at the pole
    # and edges aligned according to the provided longitude

    # Determine if we're at North or South pole
    is_north_pole = lat > 0
    pole_vertex = if is_north_pole, do: {0, 0, 1}, else: {0, 0, -1}

    # The golden ratio
    phi = @golden_ratio

    # Convert longitude to radians
    lng_rad = lng * :math.pi() / 180

    # For an icosahedron with one vertex at the pole, there are 5 vertices
    # in a pentagonal arrangement below/above the pole

    # The angle between the pole vertex and its 5 neighbors
    icosa_angle = :math.acos(phi/3 * phi/3 - 1)

    # Calculate the height component (distance along pole axis)
    height_component = if is_north_pole do
      :math.cos(icosa_angle)
    else
      -:math.cos(icosa_angle)
    end

    # Distance from central axis (perpendicular to pole axis)
    radius_component = :math.sin(icosa_angle)

    # Generate the five vertices of the first pentagonal ring
    first_pentagon = for i <- 0..4 do
      angle = lng_rad + i * 2 * :math.pi() / 5
      {
        radius_component * :math.cos(angle),
        radius_component * :math.sin(angle),
        height_component
      }
    end

    # The next layer of vertices forms another pentagon, but with a different rotation
    second_height = -height_component

    second_pentagon = for i <- 0..4 do
      angle = lng_rad + (i + 0.5) * 2 * :math.pi() / 5
      {
        radius_component * :math.cos(angle),
        radius_component * :math.sin(angle),
        second_height
      }
    end

    # The opposite pole
    opposite_pole = {-elem(pole_vertex, 0), -elem(pole_vertex, 1), -elem(pole_vertex, 2)}

    # Combine all vertices
    vertices = [pole_vertex] ++ first_pentagon ++ second_pentagon ++ [opposite_pole]

    # Normalize all vertices to ensure they're exactly on the unit sphere
    vertices = Enum.map(vertices, &normalize/1)

    # Convert to latitude/longitude
    Enum.map(vertices, &cartesian_to_lat_lng/1)
  else
    # For non-pole locations, use the original function implementation
    # [original icosahedron generation code]
    # First vertex at the specified point
    v1_lat_rad = lat * :math.pi() / 180
    v1_lng_rad = lng * :math.pi() / 180
    v1 = {
      :math.cos(v1_lat_rad) * :math.cos(v1_lng_rad),
      :math.cos(v1_lat_rad) * :math.sin(v1_lng_rad),
      :math.sin(v1_lat_rad)
    }

    # Create a standard icosahedron in canonical position
    phi = @golden_ratio

    # The 12 vertices of a regular icosahedron
    standard_vertices = [
      {0, 1, phi}, {0, -1, phi}, {0, 1, -phi}, {0, -1, -phi},
      {1, phi, 0}, {-1, phi, 0}, {1, -phi, 0}, {-1, -phi, 0},
      {phi, 0, 1}, {-phi, 0, 1}, {phi, 0, -1}, {-phi, 0, -1}
    ]

    # Normalize all vertices to the unit sphere
    standard_vertices = Enum.map(standard_vertices, &normalize/1)

    # Choose a reference vertex from the standard icosahedron
    # We'll rotate the entire solid to place this vertex at v1
    reference_vertex = List.first(standard_vertices)

    # Create a local coordinate system for the reference vertex
    # Find a vector pointing "up" relative to the reference vertex
    north = {0, 0, 1}

    # Remove the component of north that's parallel to the reference vertex
    dot_ref_north = dot_product(reference_vertex, north)
    std_up_direction = {
      elem(north, 0) - dot_ref_north * elem(reference_vertex, 0),
      elem(north, 1) - dot_ref_north * elem(reference_vertex, 1),
      elem(north, 2) - dot_ref_north * elem(reference_vertex, 2)
    }
    std_up_direction = normalize(std_up_direction)

    # Create a similar coordinate system for v1
    dot_v1_north = dot_product(v1, north)
    v1_up_direction = {
      elem(north, 0) - dot_v1_north * elem(v1, 0),
      elem(north, 1) - dot_v1_north * elem(v1, 1),
      elem(north, 2) - dot_v1_north * elem(v1, 2)
    }
    v1_up_direction = normalize(v1_up_direction)

    # We need two rotation steps:
    # 1. Rotate the standard icosahedron to place reference_vertex at v1
    # 2. Rotate around v1 to orient the "up" direction correctly

    # First rotation - move reference_vertex to v1
    rotation_matrix_1 = rotation_matrix_between_vectors(reference_vertex, v1)

    # Apply the first rotation to all vertices
    rotated_vertices = Enum.map(standard_vertices, fn vertex ->
      apply_rotation(rotation_matrix_1, vertex)
    end)

    # Also rotate the standard up direction
    rotated_up = apply_rotation(rotation_matrix_1, std_up_direction)

    # Second rotation - align the up directions
    # We want to rotate around v1 to align rotated_up with v1_up_direction
    rotation_matrix_2 = rotation_around_axis_matrix(v1, rotated_up, v1_up_direction)

    # Apply the second rotation to all vertices
    final_vertices = Enum.map(rotated_vertices, fn vertex ->
      apply_rotation(rotation_matrix_2, vertex)
    end)

    # Normalize all vertices (for numerical stability)
    final_vertices = Enum.map(final_vertices, &normalize/1)

    # Convert to latitude/longitude
    vertices = Enum.map(final_vertices, &cartesian_to_lat_lng/1)

    vertices
  end
end

# Helper function to rotate a vector around an axis by a given angle (using pre-calculated sin and cos)
defp rotate_around_axis(v, axis, cos_angle, sin_angle) do
  axis = normalize(axis)
  dot_prod = dot_product(axis, v)

  {
    cos_angle * elem(v, 0) +
    sin_angle * (elem(axis, 1) * elem(v, 2) - elem(axis, 2) * elem(v, 1)) +
    dot_prod * (1 - cos_angle) * elem(axis, 0),

    cos_angle * elem(v, 1) +
    sin_angle * (elem(axis, 2) * elem(v, 0) - elem(axis, 0) * elem(v, 2)) +
    dot_prod * (1 - cos_angle) * elem(axis, 1),

    cos_angle * elem(v, 2) +
    sin_angle * (elem(axis, 0) * elem(v, 1) - elem(axis, 1) * elem(v, 0)) +
    dot_prod * (1 - cos_angle) * elem(axis, 2)
  }
end







  @doc """
  Generates an facing tetrahedron whose vertices are the antipodes of the tetrahedron vertices.

  This creates a tetrahedron whose vertices are exactly facing to the regular tetrahedron.
  When combined with the regular tetrahedron, they form the 8 vertices of a cube.
  """
  def generate_facing_tetrahedron(lat, lng) do
    # First, get the regular tetrahedron vertices
    regular_tetrahedron = generate_tetrahedron(lat, lng)

    # Calculate the antipode of each vertex
    facing_vertices = Enum.map(regular_tetrahedron, fn vertex ->
      calculate_antipode(vertex)
    end)

    facing_vertices
  end






  @doc """
  Calculates the antipode (facing point on Earth) of a given lat/lng point.
  """
  def calculate_antipode(%{lat: lat, lng: lng}) do
    # The antipode has the facing latitude and longitude +/- 180°
    antipode_lat = -lat
    antipode_lng = if lng > 0, do: lng - 180, else: lng + 180

    %{lat: antipode_lat, lng: antipode_lng}
  end




# Helper function to create a rotation matrix that rotates one vector to another around their common axis
defp rotation_around_axis_matrix(axis, v_from, v_to) do
  # First, ensure axis is normalized
  axis = normalize(axis)

  # Project v_from and v_to onto the plane perpendicular to the axis
  v_from_proj = {
    elem(v_from, 0) - dot_product(v_from, axis) * elem(axis, 0),
    elem(v_from, 1) - dot_product(v_from, axis) * elem(axis, 1),
    elem(v_from, 2) - dot_product(v_from, axis) * elem(axis, 2)
  }

  v_to_proj = {
    elem(v_to, 0) - dot_product(v_to, axis) * elem(axis, 0),
    elem(v_to, 1) - dot_product(v_to, axis) * elem(axis, 1),
    elem(v_to, 2) - dot_product(v_to, axis) * elem(axis, 2)
  }

  # Normalize the projected vectors
  v_from_proj = normalize(v_from_proj)
  v_to_proj = normalize(v_to_proj)

  # Calculate the cosine of the angle between the projected vectors
  cos_angle = dot_product(v_from_proj, v_to_proj)

  # Calculate the sine of the angle using the cross product
  cross_result = cross_product(v_from_proj, v_to_proj)
  sin_sign = dot_product(cross_result, axis)
  sin_angle = if sin_sign >= 0 do
    :math.sqrt(1 - cos_angle * cos_angle)
  else
    -:math.sqrt(1 - cos_angle * cos_angle)
  end

  # Create the rotation matrix using Rodrigues' formula
  axis_angle_to_rotation_matrix(axis, :math.atan2(sin_angle, cos_angle))
end




@doc """
  Generates a cube that is the dual of an octahedron.

  This creates a regular cube with vertices at the centers
  of the 8 triangular faces of a regular octahedron.
  The center of one face will be placed at the specified lat/lng point.
"""
def generate_facing_cube(lat, lng) do
  # Convert target point to Cartesian coordinates
  target_lat_rad = lat * :math.pi() / 180
  target_lng_rad = lng * :math.pi() / 180
  target_point = {
    :math.cos(target_lat_rad) * :math.cos(target_lng_rad),
    :math.cos(target_lat_rad) * :math.sin(target_lng_rad),
    :math.sin(target_lat_rad)
  }

  # First, create a standard FACE-CENTERED cube
  # These are the centers of the 6 faces of a unit cube
  face_centers = [
    {1, 0, 0}, {-1, 0, 0},  # centers of x-faces
    {0, 1, 0}, {0, -1, 0},  # centers of y-faces
    {0, 0, 1}, {0, 0, -1}   # centers of z-faces
  ]

  # These face centers form the vertices of an octahedron
  # For a regular octahedron, these are already normalized

  # Now, define the 8 faces of this octahedron
  # Each face corresponds to a vertex of the cube we want to generate
  octahedron_faces = [
    [0, 2, 4], [0, 4, 3], [0, 3, 5], [0, 5, 2],  # 4 faces including +x
    [1, 4, 2], [1, 2, 5], [1, 5, 3], [1, 3, 4]   # 4 faces including -x
  ]

  # Calculate the center of each octahedron face
  # These will be the vertices of our dual cube
  cube_vertices = Enum.map(octahedron_faces, fn face_indices ->
    vertices = Enum.map(face_indices, fn idx ->
      Enum.at(face_centers, idx)
    end)

    # Calculate the centroid of the triangular face
    {x1, y1, z1} = Enum.at(vertices, 0)
    {x2, y2, z2} = Enum.at(vertices, 1)
    {x3, y3, z3} = Enum.at(vertices, 2)

    centroid = {
      (x1 + x2 + x3) / 3,
      (y1 + y2 + y3) / 3,
      (z1 + z2 + z3) / 3
    }

    # Normalize to place on unit sphere
    normalize(centroid)
  end)

  # Since we want a FACE of the cube at the target point,
  # we need to first identify which cube faces connect to each vertex

  # For a cube, each face connects to 4 vertices
  # Let's define the 6 faces of the cube
  cube_faces = [
    [0, 1, 3, 2],  # +z face
    [4, 6, 7, 5],  # -z face
    [0, 4, 5, 1],  # +y face
    [2, 3, 7, 6],  # -y face
    [0, 2, 6, 4],  # +x face
    [1, 3, 7, 5]   # -x face
  ]

  # Calculate the center of each cube face
  face_centers = Enum.map(cube_faces, fn face_indices ->
    vertices = Enum.map(face_indices, fn idx ->
      Enum.at(cube_vertices, idx)
    end)

    # Calculate centroid of the face
    sum_x = Enum.sum(Enum.map(vertices, fn {x, _, _} -> x end))
    sum_y = Enum.sum(Enum.map(vertices, fn {_, y, _} -> y end))
    sum_z = Enum.sum(Enum.map(vertices, fn {_, _, z} -> z end))

    centroid = {
      sum_x / 4,
      sum_y / 4,
      sum_z / 4
    }

    normalize(centroid)
  end)

  # Choose the first face center as reference
  reference_face = List.first(face_centers)

  # Create rotation matrix to align reference_face with target_point
  rotation_matrix = rotation_matrix_between_vectors(reference_face, target_point)

  # Apply rotation to all cube vertices
  rotated_vertices = Enum.map(cube_vertices, fn vertex ->
    apply_rotation(rotation_matrix, vertex)
  end)

  # Convert to latitude/longitude coordinates
  Enum.map(rotated_vertices, &cartesian_to_lat_lng/1)
end




@doc """
  Generates an octahedron that is the dual of a cube.

  This creates a regular octahedron with vertices at the centers
  of the 6 square faces of a regular cube. The octahedron is properly
  aligned with the cube formed by the tetrahedron and facing tetrahedron.
  A 60-degree rotation is applied for proper alignment with other solids.
"""
def generate_facing_octahedron(lat, lng) do
  # First, calculate the vertices of the octahedron in a standard orientation
  # The standard octahedron has vertices at (±1,0,0), (0,±1,0), (0,0,±1)
  standard_octahedron = [
    {1, 0, 0}, {-1, 0, 0},  # X-axis
    {0, 1, 0}, {0, -1, 0},  # Y-axis
    {0, 0, 1}, {0, 0, -1}   # Z-axis
  ]

  # Convert the target point to Cartesian coordinates
  target_lat_rad = lat * :math.pi() / 180
  target_lng_rad = lng * :math.pi() / 180
  target_point = {
    :math.cos(target_lat_rad) * :math.cos(target_lng_rad),
    :math.cos(target_lat_rad) * :math.sin(target_lng_rad),
    :math.sin(target_lat_rad)
  }

  # For consistency with the tetrahedron functions, we need to align
  # the octahedron so that one of its faces is centered at the target point

  # First, calculate a face center of the standard octahedron
  # A face of an octahedron is a triangle formed by 3 adjacent vertices
  # For simplicity, we'll use the "top" face formed by X+, Y+, Z+ vertices
  face_indices = [0, 2, 4]  # Indices of X+, Y+, Z+ vertices

  face_vertices = Enum.map(face_indices, fn idx ->
    Enum.at(standard_octahedron, idx)
  end)

  # Calculate the face center
  face_center = {
    (elem(Enum.at(face_vertices, 0), 0) +
     elem(Enum.at(face_vertices, 1), 0) +
     elem(Enum.at(face_vertices, 2), 0)) / 3,
    (elem(Enum.at(face_vertices, 0), 1) +
     elem(Enum.at(face_vertices, 1), 1) +
     elem(Enum.at(face_vertices, 2), 1)) / 3,
    (elem(Enum.at(face_vertices, 0), 2) +
     elem(Enum.at(face_vertices, 1), 2) +
     elem(Enum.at(face_vertices, 2), 2)) / 3
  }

  # Normalize to ensure it's on the unit sphere
  face_center = normalize(face_center)

  # Find the rotation that aligns the face center with the target point
  rotation_matrix = rotation_matrix_between_vectors(face_center, target_point)

  # Apply the rotation to all octahedron vertices
  transformed_octahedron = Enum.map(standard_octahedron, fn vertex ->
    apply_rotation(rotation_matrix, vertex)
  end)

  # For proper alignment with the north direction, we need a second rotation
  # Let's define an "up" direction for the octahedron face

  # Define edge vectors from face center to each vertex
  edge_vectors = Enum.map(face_vertices, fn vertex ->
    {
      elem(vertex, 0) - elem(face_center, 0),
      elem(vertex, 1) - elem(face_center, 1),
      elem(vertex, 2) - elem(face_center, 2)
    }
  end)

  # Choose the first edge as our reference direction
  std_up_direction = List.first(edge_vectors)
  std_up_direction = normalize(std_up_direction)

  # Rotate this up direction
  rotated_up = apply_rotation(rotation_matrix, std_up_direction)

  # Define what we want as "up" at the target point - pointing toward north
  north = {0, 0, 1}
  dot_target_north = dot_product(target_point, north)
  target_up_direction = {
    elem(north, 0) - dot_target_north * elem(target_point, 0),
    elem(north, 1) - dot_target_north * elem(target_point, 1),
    elem(north, 2) - dot_target_north * elem(target_point, 2)
  }

  # Handle case where target is at/near a pole
  target_up_direction = normalize(target_up_direction)

  # Calculate second rotation to align the up directions
  rotation_matrix_2 = rotation_around_axis_matrix(
    target_point,
    rotated_up,
    target_up_direction
  )

  # Apply the second rotation
  aligned_octahedron = Enum.map(transformed_octahedron, fn vertex ->
    apply_rotation(rotation_matrix_2, vertex)
  end)

  # NEW: Apply a 60-degree rotation around the axis through the target point
  # This is needed for proper alignment with other solids
  sixty_degrees = 60 * :math.pi() / 180
  # cos_sixty = :math.cos(sixty_degrees)
  # sin_sixty = :math.sin(sixty_degrees)

  # Create rotation matrix for 60-degree rotation around the target_point axis
  rotation_matrix_3 = axis_angle_to_rotation_matrix(target_point, sixty_degrees)

  # Apply the 60-degree rotation
  final_octahedron = Enum.map(aligned_octahedron, fn vertex ->
    apply_rotation(rotation_matrix_3, vertex)
  end)

  # Convert to latitude/longitude
  Enum.map(final_octahedron, &cartesian_to_lat_lng/1)
end












@doc """
  Generates a dodecahedron that is the dual of an icosahedron.

  Creates a regular dodecahedron with one face centered at the specified lat/lng point,
  properly rotated so its edges align with icosahedron vertices.
"""
def generate_facing_dodecahedron(lat, lng) do
  # Convert target point to Cartesian coordinates
  target_lat_rad = lat * :math.pi() / 180
  target_lng_rad = lng * :math.pi() / 180
  target_point = {
    :math.cos(target_lat_rad) * :math.cos(target_lng_rad),
    :math.cos(target_lat_rad) * :math.sin(target_lng_rad),
    :math.sin(target_lat_rad)
  }

  # Golden ratio
  phi = @golden_ratio

  # Create a standard dodecahedron in canonical position
  # These are the 20 vertices of a regular dodecahedron
  standard_vertices = [
    # Vertices based on (±1, ±1, ±1)
    {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1},
    {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1},

    # Vertices based on (0, ±1/φ, ±φ)
    {0, 1/phi, phi}, {0, -1/phi, phi}, {0, 1/phi, -phi}, {0, -1/phi, -phi},

    # Vertices based on (±φ, 0, ±1/φ)
    {phi, 0, 1/phi}, {-phi, 0, 1/phi}, {phi, 0, -1/phi}, {-phi, 0, -1/phi},

    # Vertices based on (±1/φ, ±φ, 0)
    {1/phi, phi, 0}, {-1/phi, phi, 0}, {1/phi, -phi, 0}, {-1/phi, -phi, 0}
  ]

  # Normalize all vertices to the unit sphere
  standard_vertices = Enum.map(standard_vertices, &normalize/1)

  # Define one of the pentagonal faces
  # Using vertices that form a pentagonal face
  face_indices = [0, 8, 9, 2, 12]

  # Calculate the center of this face
  face_vertices = Enum.map(face_indices, fn idx ->
    Enum.at(standard_vertices, idx)
  end)

  face_center = {
    Enum.sum(Enum.map(face_vertices, fn {x, _, _} -> x end)) / 5,
    Enum.sum(Enum.map(face_vertices, fn {_, y, _} -> y end)) / 5,
    Enum.sum(Enum.map(face_vertices, fn {_, _, z} -> z end)) / 5
  }
  face_center = normalize(face_center)

  # Create a "north" direction relative to the face
  # For a pentagonal face, we can use the edge between the first two vertices
  # as a reference direction
  v1 = Enum.at(face_vertices, 0)
  v2 = Enum.at(face_vertices, 1)

  # Calculate the midpoint of this edge
  edge_midpoint = {
    (elem(v1, 0) + elem(v2, 0)) / 2,
    (elem(v1, 1) + elem(v2, 1)) / 2,
    (elem(v1, 2) + elem(v2, 2)) / 2
  }
  edge_midpoint = normalize(edge_midpoint)

  # Vector from face center to edge midpoint - this is our "up" direction
  std_up_direction = {
    elem(edge_midpoint, 0) - elem(face_center, 0),
    elem(edge_midpoint, 1) - elem(face_center, 1),
    elem(edge_midpoint, 2) - elem(face_center, 2)
  }
  std_up_direction = normalize(std_up_direction)

  # For the target point, calculate a corresponding "up" direction
  # that points toward geographic north
  north = {0, 0, 1}
  dot_target_north = dot_product(target_point, north)
  target_up_direction = {
    elem(north, 0) - dot_target_north * elem(target_point, 0),
    elem(north, 1) - dot_target_north * elem(target_point, 1),
    elem(north, 2) - dot_target_north * elem(target_point, 2)
  }

  # Handle case where target is at or near a pole
  up_length = :math.sqrt(
    elem(target_up_direction, 0) * elem(target_up_direction, 0) +
    elem(target_up_direction, 1) * elem(target_up_direction, 1) +
    elem(target_up_direction, 2) * elem(target_up_direction, 2)
  )

  target_up_direction = if up_length < 0.000001 do
    # If target is at/near a pole, use east direction
    east = {1, 0, 0}
    {
      elem(east, 0) - dot_product(target_point, east) * elem(target_point, 0),
      elem(east, 1) - dot_product(target_point, east) * elem(target_point, 1),
      elem(east, 2) - dot_product(target_point, east) * elem(target_point, 2)
    }
  else
    target_up_direction
  end

  target_up_direction = normalize(target_up_direction)

  # First rotation - align face center with target point
  rotation_matrix_1 = rotation_matrix_between_vectors(face_center, target_point)

  # Apply first rotation to all vertices
  rotated_vertices = Enum.map(standard_vertices, fn vertex ->
    apply_rotation(rotation_matrix_1, vertex)
  end)

  # Also rotate the up direction
  rotated_up = apply_rotation(rotation_matrix_1, std_up_direction)

  # Second rotation - around the target point axis to align the up directions
  rotation_matrix_2 = rotation_around_axis_matrix(target_point, rotated_up, target_up_direction)

  # Apply second rotation to all vertices
  final_vertices = Enum.map(rotated_vertices, fn vertex ->
    apply_rotation(rotation_matrix_2, vertex)
  end)

  # Convert to latitude/longitude coordinates
  Enum.map(final_vertices, &cartesian_to_lat_lng/1)
end






@doc """
  Generates an icosahedron that is the dual of a dodecahedron.

  Creates a regular icosahedron with one face centered at the specified lat/lng point,
  properly rotated so its edges align with dodecahedron vertices.
"""
def generate_facing_icosahedron(lat, lng) do
  # Convert target point to Cartesian coordinates
  target_lat_rad = lat * :math.pi() / 180
  target_lng_rad = lng * :math.pi() / 180
  target_point = {
    :math.cos(target_lat_rad) * :math.cos(target_lng_rad),
    :math.cos(target_lat_rad) * :math.sin(target_lng_rad),
    :math.sin(target_lat_rad)
  }

  # Golden ratio
  phi = @golden_ratio

  # Create a standard icosahedron in canonical position
  # These are the 12 vertices of a regular icosahedron
  standard_vertices = [
    {0, 1, phi}, {0, -1, phi},    # "top" vertices
    {0, 1, -phi}, {0, -1, -phi},  # "bottom" vertices
    {1, phi, 0}, {-1, phi, 0},    # "front-top" vertices
    {1, -phi, 0}, {-1, -phi, 0},  # "front-bottom" vertices
    {phi, 0, 1}, {-phi, 0, 1},    # "middle-top" vertices
    {phi, 0, -1}, {-phi, 0, -1}   # "middle-bottom" vertices
  ]

  # Normalize all vertices to the unit sphere
  standard_vertices = Enum.map(standard_vertices, &normalize/1)

  # Define one of the triangular faces
  # Using vertices that form a triangular face
  face_indices = [0, 8, 4]  # A triangular face near the "top"

  # Calculate the center of this face
  face_vertices = Enum.map(face_indices, fn idx ->
    Enum.at(standard_vertices, idx)
  end)

  {v1, v2, v3} = {
    Enum.at(face_vertices, 0),
    Enum.at(face_vertices, 1),
    Enum.at(face_vertices, 2)
  }

  face_center = {
    (elem(v1, 0) + elem(v2, 0) + elem(v3, 0)) / 3,
    (elem(v1, 1) + elem(v2, 1) + elem(v3, 1)) / 3,
    (elem(v1, 2) + elem(v2, 2) + elem(v3, 2)) / 3
  }
  face_center = normalize(face_center)

  # Create a "north" direction relative to the face
  # For a triangular face, we can use the edge between the first two vertices
  # as a reference direction

  # Calculate the midpoint of an edge
  edge_midpoint = {
    (elem(v1, 0) + elem(v2, 0)) / 2,
    (elem(v1, 1) + elem(v2, 1)) / 2,
    (elem(v1, 2) + elem(v2, 2)) / 2
  }
  edge_midpoint = normalize(edge_midpoint)

  # Vector from face center to edge midpoint - this is our "up" direction
  std_up_direction = {
    elem(edge_midpoint, 0) - elem(face_center, 0),
    elem(edge_midpoint, 1) - elem(face_center, 1),
    elem(edge_midpoint, 2) - elem(face_center, 2)
  }
  std_up_direction = normalize(std_up_direction)

  # For the target point, calculate a corresponding "up" direction
  # that points toward geographic north
  north = {0, 0, 1}
  dot_target_north = dot_product(target_point, north)
  target_up_direction = {
    elem(north, 0) - dot_target_north * elem(target_point, 0),
    elem(north, 1) - dot_target_north * elem(target_point, 1),
    elem(north, 2) - dot_target_north * elem(target_point, 2)
  }

  # Handle case where target is at or near a pole
  up_length = :math.sqrt(
    elem(target_up_direction, 0) * elem(target_up_direction, 0) +
    elem(target_up_direction, 1) * elem(target_up_direction, 1) +
    elem(target_up_direction, 2) * elem(target_up_direction, 2)
  )

  target_up_direction = if up_length < 0.000001 do
    # If target is at/near a pole, use east direction
    east = {1, 0, 0}
    {
      elem(east, 0) - dot_product(target_point, east) * elem(target_point, 0),
      elem(east, 1) - dot_product(target_point, east) * elem(target_point, 1),
      elem(east, 2) - dot_product(target_point, east) * elem(target_point, 2)
    }
  else
    target_up_direction
  end

  target_up_direction = normalize(target_up_direction)

  # First rotation - align face center with target point
  rotation_matrix_1 = rotation_matrix_between_vectors(face_center, target_point)

  # Apply first rotation to all vertices
  rotated_vertices = Enum.map(standard_vertices, fn vertex ->
    apply_rotation(rotation_matrix_1, vertex)
  end)

  # Also rotate the up direction
  rotated_up = apply_rotation(rotation_matrix_1, std_up_direction)

  # Second rotation - around the target point axis to align the up directions
  rotation_matrix_2 = rotation_around_axis_matrix(target_point, rotated_up, target_up_direction)

  # Apply second rotation to all vertices
  final_vertices = Enum.map(rotated_vertices, fn vertex ->
    apply_rotation(rotation_matrix_2, vertex)
  end)

  # Convert to latitude/longitude coordinates
  Enum.map(final_vertices, &cartesian_to_lat_lng/1)



end












  # Advanced helper functions for accurate rotation and transformation
  # Helper function to calculate the distance between two cartesian vectors
  defp vector_distance({x1, y1, z1}, {x2, y2, z2}) do
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    :math.sqrt(dx * dx + dy * dy + dz * dz)
  end

  # Calculate the rotation matrix to rotate vector v1 to align with vector v2
  defp rotation_matrix_between_vectors(v1, v2) do
    v1 = normalize(v1)
    v2 = normalize(v2)

    # Calculate the cross product and dot product
    cross = cross_product(v1, v2)
    dot = dot_product(v1, v2)

    # Handle the case where vectors are parallel or anti-parallel
    if abs(1.0 - dot) < 1.0e-10 do
      # Vectors are nearly identical, return identity matrix
      return_identity_matrix()
    else
      if abs(-1.0 - dot) < 1.0e-10 do
        # Vectors are nearly facing
        # Find an arbitrary perpendicular vector
        perp = perpendicular_vector(v1)
        # Rotate 180 degrees around this perpendicular vector
        axis_angle_to_rotation_matrix(perp, :math.pi())
      else
        # Normal case: calculate the rotation matrix using Rodrigues' rotation formula
        k = normalize(cross)
        angle = :math.acos(dot)
        axis_angle_to_rotation_matrix(k, angle)
      end
    end
  end

  # Return identity rotation matrix
  defp return_identity_matrix() do
    {
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0}
    }
  end

  # Find a perpendicular vector to v
  defp perpendicular_vector({x, y, z}) do
    # Choose the coordinate with smallest absolute value
    # and swap with another coordinate
    cond do
      abs(x) < abs(y) and abs(x) < abs(z) ->
        normalize({0, -z, y})
      abs(y) < abs(z) ->
        normalize({-z, 0, x})
      true ->
        normalize({-y, x, 0})
    end
  end

  # Convert axis and angle to rotation matrix (Rodrigues' rotation formula)
  defp axis_angle_to_rotation_matrix({kx, ky, kz}, angle) do
    c = :math.cos(angle)
    s = :math.sin(angle)
    t = 1 - c

    r11 = c + kx * kx * t
    r12 = kx * ky * t - kz * s
    r13 = kx * kz * t + ky * s

    r21 = ky * kx * t + kz * s
    r22 = c + ky * ky * t
    r23 = ky * kz * t - kx * s

    r31 = kz * kx * t - ky * s
    r32 = kz * ky * t + kx * s
    r33 = c + kz * kz * t

    {
      {r11, r12, r13},
      {r21, r22, r23},
      {r31, r32, r33}
    }
  end

  # Apply a rotation matrix to a vector
  defp apply_rotation(matrix, {x, y, z}) do
    {{r11, r12, r13}, {r21, r22, r23}, {r31, r32, r33}} = matrix

    x_new = r11 * x + r12 * y + r13 * z
    y_new = r21 * x + r22 * y + r23 * z
    z_new = r31 * x + r32 * y + r33 * z

    {x_new, y_new, z_new}
  end

  # Vector operations
  defp cross_product({x1, y1, z1}, {x2, y2, z2}) do
    {
      y1 * z2 - z1 * y2,
      z1 * x2 - x1 * z2,
      x1 * y2 - y1 * x2
    }
  end

  defp dot_product({x1, y1, z1}, {x2, y2, z2}) do
    x1 * x2 + y1 * y2 + z1 * z2
  end

  # Convert latitude/longitude to Cartesian coordinates (x, y, z)
  # defp lat_lng_to_cartesian(lat, lng) do
  #   lat_rad = :math.pi() * lat / 180
  #   lng_rad = :math.pi() * lng / 180

  #   x = :math.cos(lat_rad) * :math.cos(lng_rad)
  #   y = :math.cos(lat_rad) * :math.sin(lng_rad)
  #   z = :math.sin(lat_rad)

  #   {x, y, z}
  # end

  # Convert Cartesian coordinates (x, y, z) to latitude/longitude
  defp cartesian_to_lat_lng({x, y, z}) do
    lat = :math.asin(z) * 180 / :math.pi()
    lng = :math.atan2(y, x) * 180 / :math.pi()

    %{lat: lat, lng: lng}
  end

  # Normalize a vector to unit length
  defp normalize({x, y, z}) do
    length = :math.sqrt(x * x + y * y + z * z)

    if length < 1.0e-10 do
      # If vector is too close to zero, return a default unit vector
      {1.0, 0.0, 0.0}
    else
      {x / length, y / length, z / length}
    end
  end





  @doc """
Verifies that a set of vertices forms a valid platonic solid.
Returns {:ok, details} if valid, or {:error, reason} if not.
"""
def verify_platonic_solid(vertices, expected_type) do
  # Convert lat/lng to cartesian for easier geometric calculations
  cartesian_vertices = Enum.map(vertices, fn v ->
    lat_rad = v.lat * :math.pi() / 180
    lng_rad = v.lng * :math.pi() / 180
    {
      :math.cos(lat_rad) * :math.cos(lng_rad),
      :math.cos(lat_rad) * :math.sin(lng_rad),
      :math.sin(lat_rad)
    }
  end)

  # Verify the number of vertices matches the expected type
  expected_vertex_count = case expected_type do
    :tetrahedron -> 4
    :cube -> 8
    :octahedron -> 6
    :dodecahedron -> 20
    :icosahedron -> 12
    _ -> 0
  end

  if length(vertices) != expected_vertex_count do
    {:error, "Expected #{expected_vertex_count} vertices for #{expected_type}, but got #{length(vertices)}"}
  else
    # Verify all vertices are on the unit sphere (distance from origin is 1)
    all_on_sphere = Enum.all?(cartesian_vertices, fn v ->
      distance = :math.sqrt(elem(v, 0)*elem(v, 0) + elem(v, 1)*elem(v, 1) + elem(v, 2)*elem(v, 2))
      abs(distance - 1.0) < 0.0001
    end)

    unless all_on_sphere do
      {:error, "Not all vertices are on the unit sphere"}
    else
      # Verify edges match the expected structure and have equal lengths
      case verify_edge_structure(cartesian_vertices, expected_type) do
        {:ok, details} -> {:ok, details}
        {:error, reason} -> {:error, reason}
      end
    end
  end
end

@doc """
Verifies the edge structure of a platonic solid.
Checks that edge lengths are consistent and that each vertex has the correct number of edges.
"""
def verify_edge_structure(vertices, expected_type) do
  # For each platonic solid, determine which vertices should be connected by edges
  # This is derived from the adjacency structure of the polyhedron

  # Calculate all pairwise distances between vertices
  distances = for i <- 0..(length(vertices) - 1), j <- (i+1)..(length(vertices) - 1) do
    {i, j, vector_distance(Enum.at(vertices, i), Enum.at(vertices, j))}
  end

  # Sort distances to find the edge length (shortest significant distance)
  sorted_distances = Enum.sort_by(distances, fn {_, _, d} -> d end)

  # The edge length is the smallest distance that appears the correct number of times
  # Each type has a specific number of edges
  expected_edge_count = case expected_type do
    :tetrahedron -> 6     # 4 vertices, 6 edges
    :cube -> 12           # 8 vertices, 12 edges
    :octahedron -> 12     # 6 vertices, 12 edges
    :dodecahedron -> 30   # 20 vertices, 30 edges
    :icosahedron -> 30    # 12 vertices, 30 edges
    _ -> 0
  end

  # Group distances by length
  grouped_distances = Enum.group_by(sorted_distances, fn {_, _, d} -> Float.round(d, 4) end)

  # The smallest distance group that has the expected count is our edge length
  {edge_length, edge_pairs} = Enum.find(grouped_distances, {0, []}, fn {_, group} ->
    length(group) == expected_edge_count
  end)

  # If we didn't find the expected number of edges at a consistent length
  if edge_length == 0 do
    most_common = Enum.max_by(grouped_distances, fn {_, group} -> length(group) end)
    found_count = length(elem(most_common, 1))
    {:error, "Expected #{expected_edge_count} edges of equal length, found #{found_count} most common"}
  else
    # Now verify that each vertex has the correct number of incident edges
    expected_edges_per_vertex = case expected_type do
      :tetrahedron -> 3     # Each vertex connects to 3 others
      :cube -> 3            # Each vertex connects to 3 others
      :octahedron -> 4      # Each vertex connects to 4 others
      :dodecahedron -> 3    # Each vertex connects to 3 others
      :icosahedron -> 5     # Each vertex connects to 5 others
      _ -> 0
    end

    # Get edge pairs (indices of vertices that form edges)
    edge_list = Enum.map(edge_pairs, fn {i, j, _} -> {i, j} end)

    # Count edges per vertex
    edge_counts = Enum.reduce(0..(length(vertices) - 1), %{}, fn v_idx, acc ->
      count = Enum.count(edge_list, fn {i, j} -> i == v_idx || j == v_idx end)
      Map.put(acc, v_idx, count)
    end)

    # Verify all vertices have the expected number of edges
    all_correct_edge_count = Enum.all?(edge_counts, fn {_, count} ->
      count == expected_edges_per_vertex
    end)

    if all_correct_edge_count do
      {:ok, %{
        vertices: length(vertices),
        edges: length(edge_list),
        edge_length: edge_length,
        edges_per_vertex: expected_edges_per_vertex
      }}
    else
      incorrect = Enum.filter(edge_counts, fn {_, count} -> count != expected_edges_per_vertex end)
      {:error, "Not all vertices have #{expected_edges_per_vertex} edges: #{inspect(incorrect)}"}
    end
  end
end


@doc """
Verifies that the angles between faces in the platonic solid are correct.
This is an additional check beyond edge lengths.
"""
def verify_face_angles(vertices, expected_type) do
  # This requires identifying faces and measuring dihedral angles
  # For simplicity, we'll focus on measuring angles at each vertex

  # For a regular polyhedron, the angle subtended by edges at a vertex should be consistent
  # The angle depends on the polyhedron type

  # Calculate edges from the vertex distances
  distances = for i <- 0..(length(vertices) - 1), j <- (i+1)..(length(vertices) - 1) do
    {i, j, vector_distance(Enum.at(vertices, i), Enum.at(vertices, j))}
  end

  # Sort distances to find the edge length
  sorted_distances = Enum.sort_by(distances, fn {_, _, d} -> d end)
  edge_length = elem(hd(sorted_distances), 2)

  # Identify edges (pairs of vertices separated by approximately edge_length)
  edges = Enum.filter(distances, fn {_, _, d} -> abs(d - edge_length) < 0.0001 end)
  edge_pairs = Enum.map(edges, fn {i, j, _} -> {i, j} end)

  # For each vertex, find its adjacent vertices and calculate angles between edges
  expected_angle = case expected_type do
    :tetrahedron -> :math.acos(-1/3)  # ~109.47°
    :cube -> :math.pi() / 2  # 90°
    :octahedron -> :math.acos(0)  # 90°
    :dodecahedron -> :math.acos(-:math.sqrt(5)/5)  # ~116.57°
    :icosahedron -> :math.acos(-:math.sqrt(5)/3)  # ~138.19°
  end

  # For each vertex, calculate angles between edges
  angle_deviations = for v_idx <- 0..(length(vertices) - 1) do
    # Find all vertices adjacent to this one
    adjacent = Enum.filter(edge_pairs, fn {i, j} -> i == v_idx || j == v_idx end)
    |> Enum.map(fn {i, j} -> if i == v_idx, do: j, else: i end)

    # Skip if not enough adjacent vertices
    if length(adjacent) < 2 do
      0.0
    else
      # Calculate angles between pairs of adjacent vertices
      angles = for i <- 0..(length(adjacent) - 1), j <- (i+1)..(length(adjacent) - 1) do
        v1 = Enum.at(vertices, v_idx)
        v2 = Enum.at(vertices, Enum.at(adjacent, i))
        v3 = Enum.at(vertices, Enum.at(adjacent, j))

        # Calculate vectors from v1 to v2 and v1 to v3
        vec1 = {
          elem(v2, 0) - elem(v1, 0),
          elem(v2, 1) - elem(v1, 1),
          elem(v2, 2) - elem(v1, 2)
        }

        vec2 = {
          elem(v3, 0) - elem(v1, 0),
          elem(v3, 1) - elem(v1, 1),
          elem(v3, 2) - elem(v1, 2)
        }

        # Calculate angle between vectors
        dot = dot_product(normalize(vec1), normalize(vec2))
        # Handle numerical precision issues
        dot = max(-1.0, min(1.0, dot))
        angle = :math.acos(dot)

        # Return deviation from expected angle
        abs(angle - expected_angle)
      end

      # Return maximum deviation
      Enum.max(angles)
    end
  end

  max_deviation = Enum.max(angle_deviations)

  if max_deviation < 0.1 do
    {:ok, %{max_angle_deviation: max_deviation}}
  else
    {:error, "Angle deviation (#{max_deviation}) exceeds threshold"}
  end
end



end
