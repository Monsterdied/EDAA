import pandas as pd
import folium

# Load the stops and edges data
stops_file = 'data/MetroDoPorto/stops.txt'
edges_file = 'metro_edges.csv'

stops = pd.read_csv(stops_file)
edges = pd.read_csv(edges_file)

# Create a base map centered around Porto
map_center = [41.15, -8.61]  # Approximate center of Porto
metro_map = folium.Map(location=map_center, zoom_start=12)

# Add stops as markers on the map
stop_coordinates = {}
for _, stop in stops.iterrows():
    stop_coordinates[stop['stop_id']] = (stop['stop_lat'], stop['stop_lon'])
    folium.Marker(
        location=[stop['stop_lat'], stop['stop_lon']],
        popup=f"{stop['stop_name']} (Zone: {stop['zone_id']})",
        tooltip=stop['stop_name']
    ).add_to(metro_map)

# Add edges as polylines on the map
for _, edge in edges.iterrows():
    from_stop = edge['from_stop']
    to_stop = edge['to_stop']
    if from_stop in stop_coordinates and to_stop in stop_coordinates:
        folium.PolyLine(
            locations=[stop_coordinates[from_stop], stop_coordinates[to_stop]],
            color='blue',
            weight=2,
            opacity=0.7
        ).add_to(metro_map)

# Save the map to an HTML file
output_path = r'c:\Users\Rodrigo\Desktop\EDAA\metro_map.html'
metro_map.save(output_path)

print(f"Map with nodes and edges saved to {output_path}. Open it in a browser to view.")