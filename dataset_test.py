import geopandas as gpd

# Load the road node shapefile
road_nodes = gpd.read_file("data/TQ_RoadNode.shp")
motorway_junctions = gpd.read_file("data/TQ_MotorwayJunction.shp")
road_links = gpd.read_file("data/TQ_RoadLink.shp")
# Display column names and sample data
print(road_nodes.columns)
print(road_nodes.head())




print(road_links.columns)
print(road_links.head())