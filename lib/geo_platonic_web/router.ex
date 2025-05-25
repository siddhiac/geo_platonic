# lib/geo_platonic_web/router.ex
defmodule GeoPlatonicWeb.Router do
  use GeoPlatonicWeb, :router

  pipeline :browser do
    plug :accepts, ["html"]
    plug :fetch_session
    plug :fetch_live_flash
    plug :put_root_layout, html: {GeoPlatonicWeb.Layouts, :root}
    plug :protect_from_forgery
    plug :put_secure_browser_headers
  end

  pipeline :api do
    plug :accepts, ["json"]
  end

  scope "/", GeoPlatonicWeb do
    pipe_through :browser

    get "/", PageController, :home
    post "/calculate", PageController, :calculate
  end

  # Other scopes may use custom stacks.
  # scope "/api", GeoPlatonicWeb do
  #   pipe_through :api
  # end
end
