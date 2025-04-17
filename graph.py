import pandas as pd

# Load the data
stop_times = pd.read_csv('data/MetroDoPorto/stop_times.txt')
trips = pd.read_csv('data/MetroDoPorto/trips.txt')

# Merge stop_times with trips to get route_id
stop_times = stop_times.merge(trips[['trip_id', 'route_id']], on='trip_id')

# Sort by trip_id and stop_sequence
stop_times = stop_times.sort_values(by=['trip_id', 'stop_sequence'])

# Extract edges
edges = []
for trip_id, group in stop_times.groupby('trip_id'):
    stops = group['stop_id'].tolist()
    for i in range(len(stops) - 1):
        edges.append((stops[i], stops[i + 1], group['route_id'].iloc[0]))

# Convert edges to a DataFrame for easier handling
edges_df = pd.DataFrame(edges, columns=['from_stop', 'to_stop', 'route_id'])

# Save edges to a CSV file (optional)
edges_df.to_csv('metro_edges.csv', index=False)

print(edges_df.head())