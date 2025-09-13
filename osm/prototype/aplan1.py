import osmnx as ox
import matplotlib.pyplot as plt
import json

# 1. Načti graf
G = ox.load_graphml("buchlovice_walk.graphml")

# 2. Načti JSON s trasou
with open("route.json", "r", encoding="utf-8") as f:
    data = json.load(f)

route_coords = [(pt["lat"], pt["lon"]) for pt in data["route"]]
start = (data["start"]["lat"], data["start"]["lon"])
end = (data["end"]["lat"], data["end"]["lon"])

# 3. Najdi uzly v grafu podle souřadnic
route_nodes = []
for lat, lon in route_coords:
    node = ox.distance.nearest_nodes(G, lon, lat)
    route_nodes.append(node)

# 4. Vykresli mapu s trasou
fig, ax = ox.plot_graph_route(
    G, route_nodes,
    route_linewidth=3,
    node_size=0,
    bgcolor="white",
    show=False, close=False
)

# 5. Přidej body
lats = [lat for lat, lon in route_coords]
lons = [lon for lat, lon in route_coords]
ax.scatter(lons, lats, c="blue", s=30, zorder=5, label="Body trasy")

ax.scatter(start[1], start[0], c="green", s=80, marker="o", zorder=6, label="Start")
ax.scatter(end[1], end[0], c="red", s=80, marker="X", zorder=6, label="Cíl")

ax.legend()
plt.show()
