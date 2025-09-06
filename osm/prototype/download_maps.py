import osmnx as ox

# Místo a radius
place_name = "Buchlovice, Czechia"
dist = 1000  # 1 km okruh

# Stáhni graf pro chodce
G = ox.graph_from_address(place_name, dist=dist, network_type="walk", simplify=False)

# Ulož
ox.save_graphml(G, "buchlovice_walk.graphml")
print("Soubor uložen jako buchlovice_walk.graphml")
