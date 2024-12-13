###  **Optimizing Pathfinding for Autonomous Electric Vehicles in Urban Environments**

---

### Important Note

This project uses Git's Large File System to upload and host the dataset information. As such, if you are to test this code, do not download this code directly as you will recieve compressed (and hence corrupted data). Instead, please clone the directory and the datasets will be downloaded to your respective device.

---

#### **Project Overview**

This project aims to optimize pathfinding for autonomous electric vehicles (EVs) in urban environments by constructing a dynamic routing model. The model considers key factors such as traffic conditions, charging station locations, and road network structure to alleviate range anxiety, reduce travel times, and optimize energy usage. By integrating real-world datasets and leveraging search algorithms (e.g., A*), the solution seeks to enhance EV navigation in complex urban settings, supporting the broader adoption of sustainable and autonomous transport systems.

---

### **Key Objectives**
1. **Construct a detailed graph-based representation of the road network**:
   - Nodes represent intersections or charging stations.
   - Edges represent road segments with attributes like distance, traffic conditions, and energy costs.

2. **Incorporate dynamic data**:
   - Real-time or historical traffic data to model congestion.
   - Charging station data for energy-efficient routing.

3. **Develop and implement search strategies**:
   - Algorithms like A* to optimize routes based on distance, energy usage, and traffic.

4. **Support scalability for larger datasets and adaptability to real-world conditions**.

---

### **Current Progress**
All steps, including data loading, preprocessing, graph construction, heuristic definition, search implementation, and testing, have been completed. Additionally, a GUI using Tkinter has been implemented to input data and visualize results.

---

### **Datasets Used**

#### **1. Road Network Data**
   - **Source**: OS Open Roads.
   - **Files Used**:
     - **`TQ_RoadNode.shp`**: Contains intersection data, with each node representing a specific location in the road network.
     - **`TQ_RoadLink.shp`**: Represents road segments (links) connecting nodes, including attributes like length and road type.
     - **`TQ_MotorwayJunction.shp`**: Details junctions on motorways for added connectivity and routing specificity.
   - **Processing**:
     - Road nodes are loaded and merged with motorway junction data to enhance node metadata.
     - Road links are processed to extract edges for graph construction, with start and end nodes identified by their unique IDs.

#### **2. Traffic Data**
   - **Source**: Department for Transport AADF Data (Average Annual Daily Flow).
   - **File Used**: `dft_traffic_counts_aadf.csv`.
   - **Details**:
     - Provides average daily traffic flow for both major and minor roads.
     - Includes `AADF` values, which are normalized to calculate congestion levels.
   - **Processing**:
     - Traffic data is joined with road links by road type to adjust edge weights in the graph based on congestion.

#### **3. Charging Station Data**
   - **Source**: National Chargepoint Registry (NCR).
   - **File Used**: `national-charge-point-registry.csv`.
   - **Details**:
     - Includes charging station locations (latitude, longitude), types (e.g., Level 2, DC fast chargers), and real-time availability.
   - **Processing**:
     - Charging stations are filtered by the bounding box of the study area (e.g., London).
     - Added as nodes to the graph with metadata on type and availability.

---

### **Code Modules**

#### **1. Loading Road Network Data**
- **Function**: `load_road_network_data(road_node_path, road_link_path, motorway_junction_path)`
- **Purpose**:
  - Loads `RoadNode`, `RoadLink`, and `MotorwayJunction` shapefiles.
  - Creates nodes (intersections) and edges (road segments) for the graph.
- **Output**:
  - `road_nodes`: DataFrame with node coordinates and attributes.
  - `edges`: List of edges with attributes (length, road type).

#### **2. Loading Traffic Data**
- **Function**: `load_aadf_data(aadf_path)`
- **Purpose**:
  - Loads and normalizes traffic data to calculate congestion levels.
- **Output**:
  - `aadf_data`: DataFrame with road type and congestion levels.

#### **3. Loading Charging Station Data**
- **Function**: `load_charging_data(charging_path, bounding_box)`
- **Purpose**:
  - Loads and preprocesses charging station data, filtering for the study area.
- **Output**:
  - `charging_stations_gdf`: GeoDataFrame with station locations and metadata.

#### **4. Graph Construction**
- **Function**: `build_graph(road_nodes, edges, aadf_data, charging_stations)`
- **Purpose**:
  - Constructs a directed graph using the road network, traffic, and charging station data.
  - Adds attributes such as congestion and charging availability to nodes and edges.
- **Output**:
  - `G`: NetworkX directed graph representing the road network.
  - `node_mapping`: Dictionary mapping node identifiers to coordinates.

#### **5. A* Search Implementation**
- **Function**: `a_star_search(graph, start, goal, heuristic, charging_stations, node_mapping, must_visit_charging=False)`
- **Purpose**:
  - Performs A* search on the graph to find the optimal path.
  - Considers charging stations if `must_visit_charging` is set to `True`.
- **Output**:
  - `path`: List of nodes representing the optimal path.
  - `cost`: Total cost of the path.

#### **6. Calculate Total Distance and Time**
- **Function**: `calculate_total_distance_and_time(path, graph, average_speed)`
- **Purpose**:
  - Calculates the total distance and time for a given path.
- **Output**:
  - `total_distance`: Total distance of the path.
  - `total_time`: Total time to travel the path.

#### **7. Test and Visualize**
- **Function**: `test_and_visualize(graph, path_with_charging)`
- **Purpose**:
  - Visualizes the graph and the path with charging stations.
- **Output**:
  - Displays a plot of the graph with the path highlighted.

#### **8. GUI Application**
- **Class**: `PathfindingApp(tk.Tk)`
- **Purpose**:
  - Provides a graphical user interface for inputting start and goal coordinates, battery percentage, and average speed.
  - Displays the optimal path and visualizes it on the graph.
- **Output**:
  - GUI for user interaction and visualization of results.

#### **9. Test Data File**
- **File**: `test_data.py`
- **Purpose**:
  - Contains sample nodes and edges for testing the pathfinding algorithm.
  - Includes functions for heuristic definition, A* search implementation, distance and time calculation, and visualization.
- **Output**:
  - Test results and visualizations for sample scenarios.

---

### **Project Directory Structure**
```
project/
│
├── data/
│   ├── TQ_RoadNode.shp
│   ├── TQ_RoadNode.shx
│   ├── TQ_RoadNode.dbf
│   ├── TQ_RoadNode.prj
│   ├── TQ_RoadLink.shp
│   ├── TQ_RoadLink.shx
│   ├── TQ_RoadLink.dbf
│   ├── TQ_RoadLink.prj
│   ├── TQ_MotorwayJunction.shp
│   ├── TQ_MotorwayJunction.shx
│   ├── TQ_MotorwayJunction.dbf
│   ├── TQ_MotorwayJunction.prj
│   ├── dft_traffic_counts_aadf.csv
│   ├── national-charge-point-registry.csv
│
├── src/
│   ├── data_processing.py   # Includes functions for loading and preprocessing data
│   ├── graph.py             # Includes graph construction logic
│   ├── main.py              # Main script to execute the workflow
│   ├── test_data.py         # Test data file for presentation purposes
│
├── results/
│   ├── visualizations/      # Stores graph visualizations
│
└── README.md                # Project documentation
```

---

### **Next Steps**
1. **Heuristic Definition**:
   - Develop a heuristic function that combines distance to the goal with proximity to charging stations.
   - Use geodesic distances for accuracy.

2. **Search Algorithm Implementation**:
   - Implement A* search to find optimal routes.

3. **Testing and Validation**:
   - Test the graph and search implementation with sample scenarios.
   - Visualize paths and compare performance.

4. **Expand Functionality**:
   - Introduce dynamic updates for traffic and charging availability.
   - Test scalability with larger datasets.

---

### **How to Run the Code**
1. Place all required data files in the `data/` directory.
2. Set the file paths in `main.py`.
3. Run `main.py`:
   ```bash
   python main.py
   ```

---

### **Dependencies**
- Python 3.8+
- Required libraries:
  - `geopandas`
  - `pandas`
  - `networkx`
  - `matplotlib`
  - `shapely`
  - `geopy`
  - `tkinter`

---

