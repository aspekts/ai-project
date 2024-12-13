import networkx as nx
import matplotlib.pyplot as plt
from geopy.distance import geodesic
from heapq import heappop, heappush

# Define sample nodes and edges
sample_nodes = {
    'A': (51.5081, -0.1281),  # Trafalgar Square
    'B': (51.5074, -0.1278),  # Charing Cross
    'C': (51.5064, -0.1272),  # Embankment
    'D': (51.4993, -0.1273),  # Westminster Abbey
    'E': (51.5033, -0.1195),  # London Eye
    'F': (51.5107, -0.1340),  # Piccadilly Circus
    'G': (51.5094, -0.1357),  # Leicester Square
    'H': (51.5030, -0.1136)   # Waterloo Station
}
sample_edges = [
    ('A', 'B', 0.5),
    ('B', 'C', 0.5),
    ('C', 'D', 0.5),
    ('A', 'C', 1.0),
    ('B', 'D', 1.0),
    ('A', 'F', 0.7),
    ('F', 'G', 0.3),
    ('G', 'B', 0.4),
    ('C', 'E', 0.6),
    ('E', 'D', 0.7),
    ('E', 'H', 0.5),
    ('H', 'D', 0.8)
]

# Define charging stations with mock charge times (in minutes)
charging_stations = [
    {'coords': (51.5075, -0.1279), 'charge_time': 10, 'node': 'B'},  # Near Charing Cross
    {'coords': (51.5035, -0.1197), 'charge_time': 15, 'node': 'E'}   # Near London Eye
]

# Step 5: Define Heuristic Function
# ----------------------------------

def heuristic(node, goal, charging_stations, graph):
    """
    Heuristic function for A* search.
    """
    node_coords = sample_nodes[node]
    goal_coords = sample_nodes[goal]
    distance_to_goal = geodesic(node_coords, goal_coords).meters

    # Calculate the penalty for being far from charging stations
    charging_penalty = min([
        geodesic(node_coords, station['coords']).meters + station['charge_time'] * 10
        for station in charging_stations
    ], default=0)

    return distance_to_goal + charging_penalty * 0.1

# Step 6: A* Search Implementation
# ---------------------------------

def a_star_search(graph, start, goal, heuristic, charging_stations, must_visit_charging=False):
    """
    Perform A* search on the graph.
    """
    if must_visit_charging:
        # Modify the graph to include mandatory stops at charging stations
        modified_graph = graph.copy()
        for station in charging_stations:
            # Add edges from start to charging stations and from charging stations to goal
            start_to_station_distance = geodesic(sample_nodes[start], station['coords']).meters / 1000  # Convert to km
            station_to_goal_distance = geodesic(station['coords'], sample_nodes[goal]).meters / 1000  # Convert to km
            modified_graph.add_edge(start, station['node'], weight=start_to_station_distance)
            modified_graph.add_edge(station['node'], goal, weight=station_to_goal_distance + station['charge_time'] / 60)  # Convert charge time to hours
        graph = modified_graph

    frontier = []
    heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heappop(frontier)

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph[current][neighbor]['weight']
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal, charging_stations, graph)
                heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    # Reconstruct path
    if goal not in came_from:
        print(f"Goal node {goal} was not reached.")
        return [], float('inf')

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    print(f"Reached goal node {goal} with cost {cost_so_far[goal]}.")
    print(modified_graph.edges)
    return path, cost_so_far[goal]

# Step 7: Calculate Total Distance and Time
# ------------------------------------------

def calculate_total_distance_and_time(path, graph):
    """
    Calculate the total distance and time for a given path.
    """
    total_distance = 0
    total_time = 0
    for i in range(len(path) - 1):
        print(f"Checking edge {path[i]} -> {path[i + 1]}")
        edge_data = graph.get_edge_data(path[i], path[i + 1])
        if edge_data is None:
            print(f"Edge data not found for {path[i]} -> {path[i + 1]}")
            print("Sample edges in the graph:")
            for u, v, data in graph.edges(data=True):
                print(f"{u} -> {v}, data: {data}")
            print(f"Edge data between {path[i]} and {path[i + 1]}: {graph.get_edge_data(path[i], path[i + 1])}")
            continue
        total_distance += edge_data['weight']
        total_time += edge_data['weight']  # Assuming speed is 1 km/h for simplicity
    return total_distance, total_time

# Step 8: Test and Visualize
# ---------------------------

def test_and_visualize(graph, path_with_charging):
    """
    Test the graph and visualize the resulting paths.
    """
    pos = {node: sample_nodes[node] for node in graph.nodes}
    nx.draw(graph, pos, with_labels=True, node_size=500, node_color='lightblue')
    nx.draw_networkx_edges(graph, pos, edgelist=list(zip(path_with_charging[:-1], path_with_charging[1:])), edge_color='g', width=2, label='With Charging Stations')
    green_line = plt.Line2D([], [], color='g', label='With Charging Stations')
    plt.legend(handles=[green_line])
    plt.title("Pathfinding with Charging Stations")
    plt.show()

# Workflow Execution
# -------------------

def main():
    """
    Main function to execute the pathfinding workflow with specific start and goal nodes.
    """
    # Build the sample graph
    G = nx.DiGraph()
    for node, coords in sample_nodes.items():
        G.add_node(node, pos=coords)
    for start, end, weight in sample_edges:
        G.add_edge(start, end, weight=weight)

    # Define start and goal nodes
    start_node = 'A'  # Trafalgar Square
    goal_node = 'H'   # Waterloo Station

    # Perform A* search without charging stations
    path_no_charging, cost_no_charging = a_star_search(G, start_node, goal_node, heuristic, [])

    # Perform A* search with mandatory charging station visits
    path_with_charging, cost_with_charging = a_star_search(G, start_node, goal_node, heuristic, charging_stations, must_visit_charging=True)

    # Calculate total distance and time for the path
    distance_with_charging, time_with_charging = calculate_total_distance_and_time(path_with_charging, G)

    # Visualize the path
    print("Path with charging stations:", path_with_charging)
    print("Cost with charging stations:", cost_with_charging)
    print("Total distance with charging stations:", distance_with_charging)
    print("Total time with charging stations:", time_with_charging)

    test_and_visualize(G, path_with_charging)

if __name__ == "__main__":
    main()