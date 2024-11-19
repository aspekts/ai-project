from data_processing import load_road_network_data, load_aadf_data, load_charging_data
from graph import build_graph

road_node_path = 'data/TQ_RoadNode.shp'
road_link_path = 'data/TQ_RoadLink.shp'
motorway_junction_path = 'data/TQ_MotorwayJunction.shp'
aadf_data_path = 'data/dft_traffic_counts_aadf.csv'
charging_path = 'data/national-charge-point-registry.csv'

# Load road network data
road_nodes, road_links = load_road_network_data(road_node_path, road_link_path, motorway_junction_path)
aadf_data = load_aadf_data(aadf_data_path)
charging_stations = load_charging_data(charging_path, bounding_box=(0.1, 51.4, 0.6, 51.6))

# Build the graph
G = build_graph(road_nodes, road_links, aadf_data, charging_stations)