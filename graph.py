import networkx as nx

def build_graph(road_nodes, edges, aadf_data, charging_stations):
    """
    Construct a graph and maintain a mapping of node identifiers to coordinates.
    """
    def format_coordinates(coord):
        return (round(coord[0], 6), round(coord[1], 6))

    G = nx.DiGraph()
    node_mapping = {}  # Map node IDs to (lat, lon)

    # Add road nodes
    for _, row in road_nodes.iterrows():
        node_coords = format_coordinates((row['geometry'].y, row['geometry'].x))
        node_id = format_coordinates(row['lat_lon'])  # Ensure node_id is rounded
        G.add_node(node_coords, junction_id=row.get('identifier', None))
        node_mapping[node_id] = node_coords

    # Add edges using the mapping
    for edge in edges:
        start_id, end_id, length, road_type = edge
        start_coords = node_mapping.get(format_coordinates(start_id))
        end_coords = node_mapping.get(format_coordinates(end_id))
        if start_coords and end_coords:
            congestion_factor = 1 + aadf_data.get(road_type, 0)
            G.add_edge(start_coords, end_coords, weight=length * congestion_factor, road_type=road_type)

    # Add charging stations
    for _, row in charging_stations.iterrows():
        station_coords = format_coordinates((row['latitude'], row['longitude']))
        G.add_node(
            station_coords,
            description=row.get('deviceDescription', 'Unknown'),
            status=row.get('chargeDeviceStatus', 'Unknown')
        )

    return G, node_mapping