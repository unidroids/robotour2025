import osmnx as ox
import matplotlib.pyplot as plt

# Načíst graf
G = ox.load_graphml("buchlovice_walk.graphml")

# Převést na GeoDataFrame
edges = ox.graph_to_gdfs(G, nodes=False, edges=True)

# Normalizace highway (pokud je list, vezmeme první)
def normalize_highway(hw):
    if isinstance(hw, list):
        return hw[0]
    return hw

edges["highway_norm"] = edges["highway"].apply(normalize_highway)

# Barevné mapování podle typu cesty
color_map = {
    "footway": "green",
    "path": "orange",
    "steps": "red",
    "service": "blue",
}
edges["color"] = edges["highway_norm"].map(color_map).fillna("gray")

# Vykreslit základní cesty
fig, ax = plt.subplots(figsize=(12, 12))
edges.plot(ax=ax, linewidth=0.7, color=edges["color"])

# Vybrat mosty
bridges = edges[edges["bridge"].notna()]

# Vykreslit mosty fialově přes původní vrstvy
if not bridges.empty:
    bridges.plot(ax=ax, linewidth=2.5, color="purple", label="Mosty")

plt.title("Cesty v okolí Buchlovic – typy cest + mosty")
plt.legend()
plt.show()
