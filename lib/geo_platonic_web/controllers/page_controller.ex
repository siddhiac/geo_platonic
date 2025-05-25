defmodule GeoPlatonicWeb.PageController do
  use GeoPlatonicWeb, :controller
  alias GeoPlatonic.PlatonicSolids

  def home(conn, _params) do
    render(conn, :home)
  end

  def calculate(conn, %{"latitude" => lat, "longitude" => lng}) do
    {lat, _} = Float.parse(lat)
    {lng, _} = Float.parse(lng)

    # Use simple maps instead of GeoCalc.Point
    selected_point = %{lat: lat, lng: lng}
    antipode = calculate_antipode(selected_point)

    # Generate platonic solids
    tetrahedron = PlatonicSolids.generate_tetrahedron(lat, lng)
    facing_tetrahedron = PlatonicSolids.generate_facing_tetrahedron(lat, lng)
    cube = PlatonicSolids.generate_cube(lat, lng)
    octahedron = PlatonicSolids.generate_octahedron(lat, lng)
    dodecahedron = PlatonicSolids.generate_dodecahedron(lat, lng)
    icosahedron = PlatonicSolids.generate_icosahedron(lat, lng)

    facing_cube = PlatonicSolids.generate_facing_cube(lat, lng)
    facing_octahedron = PlatonicSolids.generate_facing_octahedron(lat, lng)
    facing_dodecahedron = PlatonicSolids.generate_facing_dodecahedron(lat, lng)
    facing_icosahedron = PlatonicSolids.generate_facing_icosahedron(lat, lng)

    # Return results as JSON
    json(conn, %{
      antipode: antipode,
      platonic_solids: %{
        tetrahedron: tetrahedron,
        facing_tetrahedron: facing_tetrahedron,
        cube: cube,
        octahedron: octahedron,
        dodecahedron: dodecahedron,
        icosahedron: icosahedron,
        facing_cube: facing_cube,
        facing_octahedron: facing_octahedron,
        facing_dodecahedron: facing_dodecahedron,
        facing_icosahedron: facing_icosahedron
      }
    })
  end

  # Helper function
  defp calculate_antipode(%{lat: lat, lng: lng}) do
    %{lat: -lat, lng: normalize_longitude(lng + 180)}
  end

  defp normalize_longitude(lng) when lng > 180, do: lng - 360
  defp normalize_longitude(lng) when lng < -180, do: lng + 360
  defp normalize_longitude(lng), do: lng
end
