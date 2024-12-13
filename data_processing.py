import pandas as pd
import geopandas as gpd
from shapely.geometry import Point

def load_road_network_data(road_node_path, road_link_path, motorway_junction_path):
    """
    Load and preprocess road network data: RoadNode, RoadLink, and MotorwayJunction.
    :param road_node_path: Path to RoadNode shapefile.
    :param road_link_path: Path to RoadLink shapefile.
    :param motorway_junction_path: Path to MotorwayJunction shapefile.
    :return: Preprocessed road nodes and edges.
    """
    # Load RoadNode data
    road_nodes = gpd.read_file(road_node_path)
    road_nodes['coordinates'] = road_nodes['geometry'].apply(lambda x: (x.x, x.y))
    # Create a dictionary for fast lookups
    node_lookup = road_nodes.set_index('identifier')['coordinates'].to_dict()

    # Load RoadLink data
    road_links = gpd.read_file(road_link_path)
    road_links = road_links[['identifier', 'startNode', 'endNode', 'length', 'formOfWay', 'geometry']]

    # Ensure consistent types
    road_links['startNode'] = road_links['startNode'].astype(str).str.strip()
    road_links['endNode'] = road_links['endNode'].astype(str).str.strip()
    road_nodes['identifier'] = road_nodes['identifier'].astype(str).str.strip()

    # Debugging: Print the first few rows of road_links
    print("Road Links:")
    print(road_links.head())

    # Load MotorwayJunction data
    motorway_junctions = gpd.read_file(motorway_junction_path)
    motorway_junctions = motorway_junctions[['number', 'identifier', 'geometry']]
    motorway_junctions['coordinates'] = motorway_junctions['geometry'].apply(lambda x: (x.x, x.y))

    # Create edges from RoadLink data
    edges = []
    for _, row in road_links.iterrows():
        start = node_lookup.get(row['startNode'])
        end = node_lookup.get(row['endNode'])

        # Debugging: Print start and end nodes
        if start is None:
            print(f"Start node {row['startNode']} not found in node_lookup.")
        if end is None:
            print(f"End node {row['endNode']} not found in node_lookup.")

        if start and end:
            edges.append((start, end, row['length'], row['formOfWay']))
            print(f"Added edge: {start} -> {end}")
        else:
            print(f"[PREPROCESSING]: Skipping edge: startNode={row['startNode']} or endNode={row['endNode']} not found in road_nodes.")

    # Check if the road network data is loaded correctly
    print(f"Loaded {len(road_nodes)} road nodes and {len(edges)} road edges.")
    return road_nodes, edges

def load_aadf_data(csv_path, region_filter='London'):
    """
    Load AADF traffic data and filter for the specified region.
    :param csv_path: Path to the AADF data file.
    :param region_filter: Region to filter the data (e.g., 'London').
    :return: DataFrame with traffic data filtered by region.
    """
    aadf_data = pd.read_csv(csv_path)
    filtered_data = aadf_data[aadf_data['region_name'] == region_filter]
    print(f"Loaded {len(filtered_data)} rows of AADF data for {region_filter}.")

    # Calculate congestion level as the sum of all motor vehicles
    filtered_data['congestion_level'] = filtered_data['all_motor_vehicles'] / filtered_data['all_motor_vehicles'].max()

    return filtered_data[['road_name', 'congestion_level']]

def load_charging_data(csv_path, bounding_box):
    """
    Load and filter charging station data within the specified bounding box.
    :param csv_path: Path to the charging station data file.
    :param bounding_box: Bounding box to filter stations (minx, miny, maxx, maxy).
    :return: GeoDataFrame with charging station data.
    """
    charging_stations = pd.read_csv(csv_path)
    charging_stations['geometry'] = charging_stations.apply(
        lambda row: Point(row['longitude'], row['latitude']),
        axis=1
    )
    charging_stations_gdf = gpd.GeoDataFrame(charging_stations, geometry='geometry')
    print(f"Loaded {len(charging_stations_gdf)} charging stations.")
    return charging_stations_gdf.cx[bounding_box[0]:bounding_box[2], bounding_box[1]:bounding_box[3]]