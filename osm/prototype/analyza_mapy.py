import osmnx as ox
import matplotlib.pyplot as plt

# Načíst graf
G = ox.load_graphml("buchlovice_walk.graphml")

# Převést na GeoDataFrame (jen hrany = cesty)
edges = ox.graph_to_gdfs(G, nodes=False, edges=True)

# Funkce na normalizaci highway (vezme první typ pokud je list)
def normalize_highway(hw):
    if isinstance(hw, list):
        return hw[0]
    return hw

edges["highway_norm"] = edges["highway"].apply(normalize_highway)

# Vypsat unikátní typy
print("Unikátní typy cest:", edges["highway_norm"].unique())

# Barevné mapování podle typu
color_map = {
    "footway": "green",
    "path": "orange",
    "steps": "red",
    "service": "blue",
}
edges["color"] = edges["highway_norm"].map(color_map).fillna("gray")

# Vykreslení
fig, ax = plt.subplots(figsize=(10, 10))
edges.plot(ax=ax, linewidth=0.7, color=edges["color"])
ax.set_title("Cesty v okolí Buchlovic – barevně dle typu")
plt.show()
