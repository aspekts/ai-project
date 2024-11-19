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
    road_nodes = road_nodes[['NODE_ID', 'geometry']]
    road_nodes['coordinates'] = road_nodes['geometry'].apply(lambda x: (x.x, x.y))

    # Load RoadLink data
    road_links = gpd.read_file(road_link_path)
    road_links = road_links[['LINK_ID', 'START_NODE', 'END_NODE', 'length', 'road_type', 'geometry']]

    # Load MotorwayJunction data
    motorway_junctions = gpd.read_file(motorway_junction_path)
    motorway_junctions = motorway_junctions[['JUNCTION_ID', 'NODE_ID', 'geometry']]
    motorway_junctions['coordinates'] = motorway_junctions['geometry'].apply(lambda x: (x.x, x.y))

    # Merge motorway junctions into road nodes
    road_nodes = road_nodes.merge(
        motorway_junctions[['NODE_ID', 'JUNCTION_ID']],
        on='NODE_ID',
        how='left'
    )

    # Create edges from RoadLink data
    edges = []
    for _, row in road_links.iterrows():
        start = road_nodes.loc[road_nodes['NODE_ID'] == row['START_NODE'], 'coordinates'].values[0]
        end = road_nodes.loc[road_nodes['NODE_ID'] == row['END_NODE'], 'coordinates'].values[0]
        edges.append((start, end, row['length'], row['road_type']))

    return road_nodes, edges
# Load AADF traffic data
# This data provides average annual daily flow for major and minor roads
def load_aadf_data(csv_path, region_filter='London'):
    """
    Load AADF traffic data and filter for the specified region.
    :param csv_path: Path to the AADF data file.
    :param region_filter: Region to filter the data (e.g., 'London').
    :return: DataFrame with traffic data filtered by region.
    """
    aadf_data = pd.read_csv(csv_path)
    filtered_data = aadf_data[aadf_data['region_name'] == region_filter]
    filtered_data['congestion_level'] = filtered_data['AADF'] / filtered_data['AADF'].max()
    return filtered_data[['road_name', 'congestion_level']]

# Load charging station data
# This data includes charging station locations, types, and availability
def load_charging_data(csv_path, bounding_box):
    """
    Load and filter charging station data within the specified bounding box.
    :param csv_path: Path to the charging station data file.
    :param bounding_box: Bounding box to filter stations (minx, miny, maxx, maxy).
    :return: GeoDataFrame with charging station data.
    """
    charging_stations = pd.read_csv(csv_path)
    charging_stations['geometry'] = charging_stations.apply(
        lambda row: Point(row['Longitude'], row['Latitude']),
        axis=1
    )
    charging_stations_gdf = gpd.GeoDataFrame(charging_stations, geometry='geometry')
    return charging_stations_gdf.cx[bounding_box[0]:bounding_box[2], bounding_box[1]:bounding_box[3]]
