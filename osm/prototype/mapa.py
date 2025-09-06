import osmnx as ox
import matplotlib.pyplot as plt

# --- Načtení cest (GraphML) ---
G = ox.load_graphml("buchlovice_walk.graphml")
edges = ox.graph_to_gdfs(G, nodes=False, edges=True)

# Normalizace highway (pokud je list, vezmeme první)
def normalize_highway(hw):
    if isinstance(hw, list):
        return hw[0]
    return hw

edges["highway_norm"] = edges["highway"].apply(normalize_highway)

# --- Stažení dalších vrstev (GeoJSON/OSM) ---
tags = {
    "building": True,
    "landuse": True,
    "leisure": True,
    "natural": True,
    "waterway": True,
    "barrier": True,
}

# Nová funkce v osmnx
gdf = ox.features_from_address("Buchlovice, Czechia", dist=5000, tags=tags)

# --- Vykreslení ---
fig, ax = plt.subplots(figsize=(12, 12))

# Budovy
gdf[gdf["building"].notna()].plot(ax=ax, color="lightgray", alpha=0.7, label="Budovy")

# Trávníky a parky
gdf[(gdf["landuse"] == "grass") | (gdf["leisure"] == "park")].plot(
    ax=ax, color="lightgreen", alpha=0.5, label="Trávníky/Parky"
)

# Voda
gdf[(gdf["natural"] == "water") | (gdf["waterway"].notna())].plot(
    ax=ax, color="lightblue", alpha=0.5, label="Voda"
)

# Bariéry
gdf[gdf["barrier"].notna()].plot(ax=ax, color="black", linewidth=1, label="Bariéry")

# Cesty
color_map = {"footway": "green", "path": "orange", "steps": "red", "service": "blue"}
edges["color"] = edges["highway_norm"].map(color_map).fillna("gray")
edges.plot(ax=ax, linewidth=0.7, color=edges["color"], label="Cesty")

# Mosty
bridges = edges[edges["bridge"].notna()]
if not bridges.empty:
    bridges.plot(ax=ax, linewidth=2.5, color="purple", label="Mosty")

plt.title("Buchlovice – cesty, budovy, parky, voda, bariéry")
plt.legend()
plt.show()

# --- Analýza unikátních hodnot ---
def print_unique(tag):
    if tag in gdf.columns:
        values = gdf[tag].dropna().unique()
        print(f"\nUnikátní hodnoty {tag}:")
        print(values)
    else:
        print(f"\nTag {tag} se v datech nenašel.")

for tag in ["building", "landuse", "natural", "barrier"]:
    print_unique(tag)

# import matplotlib.pyplot as plt

# # Vybereme jen prvky s tagem "natural"
# naturals = gdf[gdf["natural"].notna()]

# # Přiřadíme barvy k jednotlivým hodnotám
# color_map = {
#     "tree": "green",
#     "tree_row": "darkgreen",
#     "wood": "forestgreen",
#     "grassland": "lightgreen",
#     "scrub": "olive",
#     "wetland": "aqua",
#     "water": "blue",
#     "spring": "cyan",
#     "rock": "gray",
#     "cliff": "brown",
#     "peak": "darkred",
#     "mountain_range": "purple",
# }

# # Pokud se objeví něco jiného, použije se šedá
# naturals["color"] = naturals["natural"].map(color_map).fillna("lightgray")

# # Vykreslení
# fig, ax = plt.subplots(figsize=(12, 12))
# naturals.plot(ax=ax, color=naturals["color"], markersize=10)

# plt.title("Buchlovice – prvky natural (stromy, voda, lesy, skály...)")
# plt.show()
