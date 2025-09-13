import osmnx as ox
import networkx as nx
import json


from math import cos, radians, sqrt

def is_point_closer(ref_node, near_node, lat, lon):
    def dist(lat1, lon1, lat2, lon2):
        dx = (lon2 - lon1) * cos(radians(lat1)) * 111319.49
        dy = (lat2 - lat1) * 111132.954
        return sqrt(dx*dx + dy*dy)

#    first_node = route[0]
#    second_node = route[1]

    ref_lat = nodes.loc[ref_node, "y"]
    ref_lon = nodes.loc[ref_node, "x"]

    p1_lat = nodes.loc[near_node, "y"]
    p1_lon = nodes.loc[near_node, "x"]

    d1 = dist(ref_lat, ref_lon, p1_lat, p1_lon)
    d2 = dist(ref_lat, ref_lon, lat, lon)

    return d2 <= d1
    
# 1. Načti graf
G = ox.load_graphml("buchlovice_walk.graphml")

# 2. Start/cíl – GPS body
start_lat, start_lon = 49.0840939, 17.3377833 #49.0840753, 17.3410719
end_lat, end_lon = 49.0845862,17.3417674 #49.0838486, 17.3361181
#end_lat, end_lon = 49.0840753, 17.3410719
#start_lat, start_lon = 49.0838486, 17.3361181

# 3. Vyřaď zakázané hrany
G_filtered = G.copy()
for u, v, k, data in list(G_filtered.edges(keys=True, data=True)):
    hw = data.get("highway")
    if isinstance(hw, list):
        hw = hw[0]
    if hw in {"steps", "pedestrian"}:
        G_filtered.remove_edge(u, v, k)

# 4. Odstraň izolované uzly
isolated_nodes = list(nx.isolates(G_filtered))
G_filtered.remove_nodes_from(isolated_nodes)
print("Počet izolovaných uzlů odstraněno:", len(isolated_nodes))

# 5. Najdi nejbližší uzly
orig_node = ox.distance.nearest_nodes(G_filtered, start_lon, start_lat)
dest_node = ox.distance.nearest_nodes(G_filtered, end_lon, end_lat)

# 6. Spočítej trasu
if not nx.has_path(G_filtered, orig_node, dest_node):
    print("⚠️ Mezi startem a cílem neexistuje cesta v ořezaném grafu.")
    exit()

route = nx.shortest_path(G_filtered, orig_node, dest_node, weight="length")

# 7. Připrav JSON
nodes, _ = ox.graph_to_gdfs(G_filtered)
route_nodes = nodes.loc[route]

#remove first node if start is close to second node than first node to second one
if is_point_closer(route[0], route[1], start_lat, start_lon):
    route = route[1:]

if is_point_closer(route[-2], route[-1], end_lat, end_lon):
    route = route[:-1]

route_data = [
    {
        "lat": float(route_nodes.loc[node_id, "y"]),
        "lon": float(route_nodes.loc[node_id, "x"]),
        "node_id": int(node_id)
    }
    for node_id in route
]

data = {
    "start": {"lat": start_lat, "lon": start_lon, "node_id": int(orig_node)},
    "end": {"lat": end_lat, "lon": end_lon, "node_id": int(dest_node)},
    "route": route_data
}

with open("route.json", "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)

print("✅ Trasa uložena do route.json")

