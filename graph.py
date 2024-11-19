import networkx as nx
def build_graph(roads, aadf_data, charging_stations):
    """
    Construct a graph from road, traffic, and charging station data.
    :param roads: GeoDataFrame with road data.
    :param aadf_data: DataFrame with traffic congestion levels.
    :param charging_stations: GeoDataFrame with charging station data.
    :return: NetworkX directed graph.
    """
    G = nx.DiGraph()
    
    # Add edges from road data
    roads = roads.merge(aadf_data, on='road_name', how='left')
    for _, row in roads.iterrows():
        if row['geometry'].geom_type == 'LineString':
            coords = list(row['geometry'].coords)
            for i in range(len(coords) - 1):
                start, end = coords[i], coords[i + 1]
                weight = row['length'] * (1 + row.get('congestion_level', 0))
                G.add_edge(start, end, weight=weight)
    
    # Add charging stations as nodes
    for _, row in charging_stations.iterrows():
        G.add_node(row['StationID'], location=(row['Latitude'], row['Longitude']),
                   type=row['ChargerType'], availability=row['Availability'])
    
    return G