defmodule GeoPlatonic.Application do
  # See https://hexdocs.pm/elixir/Application.html
  # for more information on OTP Applications
  @moduledoc false

  use Application

  @impl true
  def start(_type, _args) do
    children = [
      GeoPlatonicWeb.Telemetry,
      {DNSCluster, query: Application.get_env(:geo_platonic, :dns_cluster_query) || :ignore},
      {Phoenix.PubSub, name: GeoPlatonic.PubSub},
      # Start the Finch HTTP client for sending emails
      {Finch, name: GeoPlatonic.Finch},
      # Start a worker by calling: GeoPlatonic.Worker.start_link(arg)
      # {GeoPlatonic.Worker, arg},
      # Start to serve requests, typically the last entry
      GeoPlatonicWeb.Endpoint
    ]

    # See https://hexdocs.pm/elixir/Supervisor.html
    # for other strategies and supported options
    opts = [strategy: :one_for_one, name: GeoPlatonic.Supervisor]
    Supervisor.start_link(children, opts)
  end

  # Tell Phoenix to update the endpoint configuration
  # whenever the application is updated.
  @impl true
  def config_change(changed, _new, removed) do
    GeoPlatonicWeb.Endpoint.config_change(changed, removed)
    :ok
  end
end
