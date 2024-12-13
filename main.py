import tkinter as tk
from tkinter import messagebox
import networkx as nx
from geopy.distance import geodesic
import matplotlib.pyplot as plt
from heapq import heappop, heappush
from data_processing import load_road_network_data, load_aadf_data, load_charging_data
from graph import build_graph

# Define file paths
road_node_path = 'data/TQ_RoadNode.shp'
road_link_path = 'data/TQ_RoadLink.shp'
motorway_junction_path = 'data/TQ_MotorwayJunction.shp'
aadf_data_path = 'data/dft_traffic_counts_aadf.csv'
charging_path = 'data/national-charge-point-registry.csv'

# Load data
road_nodes, edges = load_road_network_data(road_node_path, road_link_path, motorway_junction_path)
aadf_data = load_aadf_data(aadf_data_path)
charging_stations = load_charging_data(charging_path, bounding_box=(0.1, 51.4, 0.6, 51.6))

# Convert Cartesian coordinates to geographic coordinates (latitude, longitude)
road_nodes = road_nodes.to_crs(epsg=4326)
road_nodes['lat_lon'] = road_nodes['geometry'].apply(lambda x: (x.y, x.x))

# Build the graph and get node mapping
G, node_mapping = build_graph(road_nodes, edges, aadf_data, charging_stations)

def format_coordinates(coord):
    return (round(coord[0], 6), round(coord[1], 6))

# Define Heuristic Function
def heuristic(node, goal, charging_stations, graph):
    node_coords = node
    goal_coords = goal
    distance_to_goal = geodesic(node_coords, goal_coords).meters
    charging_penalty = min([
        geodesic(node_coords, (station.geometry.y, station.geometry.x)).meters
        for _, station in charging_stations.iterrows()
    ], default=0)
    return distance_to_goal + charging_penalty * 0.1

# A* Search Implementation
def a_star_search(graph, start, goal, heuristic, charging_stations, node_mapping, must_visit_charging=False):
    start_node_id = format_coordinates(start)
    goal_node_id = format_coordinates(goal)
    if start_node_id not in node_mapping:
        print(f"Start node ID {start_node_id} not found in node mapping.")
        raise ValueError("Available node IDs:", list(node_mapping.keys())[:10])

    if goal_node_id not in node_mapping:
        print(f"Goal node ID {goal_node_id} not found in node mapping.")
        raise ValueError("Available node IDs:", list(node_mapping.keys())[:10])

    start = node_mapping[start_node_id]
    goal = node_mapping[goal_node_id]

    if must_visit_charging:
        modified_graph = graph.copy()
        for _, station in charging_stations.iterrows():
            station_coords = (station.geometry.y, station.geometry.x)  # Ensure station is in (lat, lon) format
            start_to_station_distance = geodesic(start, station_coords).meters / 1000
            station_to_goal_distance = geodesic(station_coords, goal).meters / 1000
            modified_graph.add_edge(start, station_coords, weight=start_to_station_distance)
            modified_graph.add_edge(station_coords, goal, weight=station_to_goal_distance)
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
            edge_data = graph.get_edge_data(current, neighbor)
            if edge_data is None:
                estimated_distance = geodesic(current, neighbor).meters / 1000  # Convert to kilometers
                new_cost = cost_so_far[current] + estimated_distance
            else:
                new_cost = cost_so_far[current] + graph[current][neighbor]['weight']
            
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal, charging_stations, graph)
                heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    if goal not in came_from:
        return [], float('inf')

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path, cost_so_far[goal]

# BFS Search Implementation
def bfs_search(graph, start, goal):
    """
    Perform BFS search on the graph.
    """
    frontier = [start]
    came_from = {start: None}

    while frontier:
        current = frontier.pop(0)

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            if neighbor not in came_from:
                frontier.append(neighbor)
                came_from[neighbor] = current

    if goal not in came_from:
        return [], float('inf')

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path, len(path) - 1

# UCS Search Implementation
def ucs_search(graph, start, goal):
    """
    Perform UCS search on the graph.
    """
    frontier = []
    heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        current_cost, current = heappop(frontier)

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph[current][neighbor]['weight']
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                heappush(frontier, (new_cost, neighbor))
                came_from[neighbor] = current

    if goal not in came_from:
        return [], float('inf')

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path, cost_so_far[goal]

# Calculate Total Distance and Time
def calculate_total_distance_and_time(path, graph, average_speed):
    total_distance = 0
    total_time = 0
    for i in range(len(path) - 1):
        print(f"Checking edge data for nodes {path[i]} -> {path[i + 1]}...")
        edge_data = graph.get_edge_data(path[i], path[i + 1])
        if edge_data is None:
            print(f"Edge data not found for nodes {path[i]} -> {path[i + 1]}. Estimating distance using geodesic.")
            estimated_distance = geodesic(path[i], path[i + 1]).meters / 1000  # Convert to kilometers
            total_distance += estimated_distance
            total_time += estimated_distance / average_speed  # Time = Distance / Speed
        else:
            print(f"Edge data found: {edge_data}")
            total_distance += edge_data['weight']
            total_time += edge_data['weight'] / average_speed  # Time = Distance / Speed
    return total_distance, total_time

# Format time in hours and minutes
def format_time(hours):
    if hours < 1:
        return f"{int(hours * 60)} minutes"
    else:
        return f"{round(hours, 2)} hours"

# Test and Visualize
def test_and_visualize(graph, path_no_charging, path_with_charging, charging_stations):
    pos = {node: node for node in graph.nodes}

    # Create a subgraph with only the relevant nodes and edges
    subgraph_nodes = set(path_no_charging + path_with_charging)
    subgraph_edges = [(path_no_charging[i], path_no_charging[i + 1]) for i in range(len(path_no_charging) - 1)]
    subgraph_edges += [(path_with_charging[i], path_with_charging[i + 1]) for i in range(len(path_with_charging) - 1)]
    subgraph = graph.edge_subgraph(subgraph_edges).copy()

    # Draw the subgraph
    plt.figure(figsize=(12, 8))
    nx.draw(subgraph, pos, with_labels=True, node_size=500, node_color='lightblue')
    nx.draw_networkx_edges(subgraph, pos, edgelist=list(zip(path_no_charging[:-1], path_no_charging[1:])), edge_color='r', width=2, label='Without Charging Stations')
    nx.draw_networkx_edges(subgraph, pos, edgelist=list(zip(path_with_charging[:-1], path_with_charging[1:])), edge_color='g', width=2, label='With Charging Stations')

    # Highlight charging stations
    charging_station_nodes = [format_coordinates((station.geometry.y, station.geometry.x)) for _, station in charging_stations.iterrows()]
    nx.draw_networkx_nodes(subgraph, pos, nodelist=charging_station_nodes, node_color='yellow', node_size=700, label='Charging Stations')

    red_line = plt.Line2D([], [], color='r', label='Without Charging Stations')
    green_line = plt.Line2D([], [], color='g', label='With Charging Stations')
    yellow_circle = plt.Line2D([], [], color='yellow', marker='o', linestyle='None', markersize=10, label='Charging Stations')
    plt.legend(handles=[red_line, green_line, yellow_circle])
    plt.title("Pathfinding with and without Charging Stations")
    plt.show()

# GUI Application
class PathfindingApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pathfinding App")
        self.geometry("400x400")

        self.start_label = tk.Label(self, text="Start Coordinates (lat, lon):")
        self.start_label.pack()
        self.start_entry = tk.Entry(self)
        self.start_entry.pack()

        self.goal_label = tk.Label(self, text="Goal Coordinates (lat, lon):")
        self.goal_label.pack()
        self.goal_entry = tk.Entry(self)
        self.goal_entry.pack()

        self.battery_label = tk.Label(self, text="Car Battery Percentage:")
        self.battery_label.pack()
        self.battery_entry = tk.Entry(self)
        self.battery_entry.pack()

        self.speed_label = tk.Label(self, text="Average Speed (km/h):")
        self.speed_label.pack()
        self.speed_entry = tk.Entry(self)
        self.speed_entry.pack()

        self.find_path_button = tk.Button(self, text="Find Path", command=self.find_path)
        self.find_path_button.pack()

    def find_path(self):
        try:
            start_coords = tuple(map(float, self.start_entry.get().strip().split(',')))
            goal_coords = tuple(map(float, self.goal_entry.get().strip().split(',')))
            battery_percentage = float(self.battery_entry.get().strip())
            average_speed = float(self.speed_entry.get().strip())

            # Validate coordinates
            if not (-90 <= start_coords[0] <= 90 and -180 <= start_coords[1] <= 180):
                raise ValueError("Invalid start coordinates.")
            if not (-90 <= goal_coords[0] <= 90 and -180 <= goal_coords[1] <= 180):
                raise ValueError("Invalid goal coordinates.")
            if average_speed <= 0:
                raise ValueError("Average speed must be greater than 0.")
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")
            return

        # Find the nearest nodes to the specified coordinates
        road_nodes['distance_to_start'] = road_nodes['lat_lon'].apply(lambda x: geodesic(x, start_coords).meters)
        road_nodes['distance_to_goal'] = road_nodes['lat_lon'].apply(lambda x: geodesic(x, goal_coords).meters)

        start_node = format_coordinates(road_nodes.loc[road_nodes['distance_to_start'].idxmin(), 'lat_lon'])
        goal_node = format_coordinates(road_nodes.loc[road_nodes['distance_to_goal'].idxmin(), 'lat_lon'])

        print(f"Start node: {start_node}")
        print(f"Goal node: {goal_node}")
        print(f"Available node IDs: {list(node_mapping.keys())[:10]}")

        if start_node not in node_mapping or goal_node not in node_mapping:
            messagebox.showerror("Error", "Start or goal node not found in the node mapping.")
            return

        path_no_charging, cost_no_charging = a_star_search(G, start_node, goal_node, heuristic, charging_stations, node_mapping, must_visit_charging=False)
        path_with_charging, cost_with_charging = a_star_search(G, start_node, goal_node, heuristic, charging_stations, node_mapping, must_visit_charging=True)
        distance_with_charging, time_with_charging = calculate_total_distance_and_time(path_with_charging, G, average_speed)

        # Perform BFS search
        path_bfs, cost_bfs = bfs_search(G, start_node, goal_node)

        # Perform UCS search
        path_ucs, cost_ucs = ucs_search(G, start_node, goal_node)
        if path_ucs is None:
            print("No path found with UCS.")
            path_ucs = "No path found."
        if path_bfs is None:
            print("No path found with BFS.")
            path_bfs = "No path found."
        if cost_ucs == float('inf'):
            cost_ucs = "Infinity"
        if cost_bfs == float('inf'):
            cost_bfs = "Infinity"

        result = f"Path with charging stations: {path_with_charging}\n"
        result += f"Cost with charging stations: {cost_with_charging}\n"
        result += f"Total distance with charging stations: {distance_with_charging} km\n"
        result += f"Total time with charging stations: {format_time(time_with_charging)}\n"
        result += f"Path with BFS: {path_bfs}\n"
        result += f"Cost with BFS: {cost_bfs}\n"
        result += f"Path with UCS: {path_ucs}\n"
        result += f"Cost with UCS: {cost_ucs}\n"

        # Display the message box and then show the graph
        self.after(0, lambda: messagebox.showinfo("Pathfinding Result", result))
        self.after(0, lambda: test_and_visualize(G, path_no_charging, path_with_charging, charging_stations))

if __name__ == "__main__":
    app = PathfindingApp()
    app.mainloop()