import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("tour", HERE.parent / "scripts" / "tour_map_drive.py")
tour = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tour)


def test_route_covers_every_cell_through_open_boundaries():
    tools = tour._maze_tools()
    layout = tools.load()
    route = tour.coverage_route(layout, tools)
    assert set(route) == {(r, c) for r in range(layout["rows"]) for c in range(layout["cols"])}
    adj = tools.open_neighbours(layout)
    assert route[0] == tuple(layout["start_cell"])
    for a, b in zip(route, route[1:]):
        assert b in adj[a], f"{a}->{b} crosses a wall"
