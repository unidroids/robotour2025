# mapa_final.py
import osmnx as ox
import geopandas as gpd
import matplotlib.pyplot as plt
import re

# === 1) Načtení graphml a příprava ===
#G = ox.load_graphml("buchlovice_walk.graphml")
# Místo a radius
place_name = "Buchlovice, Czechia"
dist = 1000  # 1 km okruh

G = ox.graph_from_address(place_name, dist=dist, network_type="walk", simplify=False)

edges = ox.graph_to_gdfs(G, nodes=False, edges=True).copy()

def normalize_highway(hw):
    if isinstance(hw, list):
        return hw[0]
    return hw

edges["highway_norm"] = edges["highway"].apply(normalize_highway)

print("Počet hran (edges):", len(edges))
print("Typy highway:", edges["highway_norm"].dropna().unique())

# === 2) Parsování width + fallback ===
def parse_width(val):
    if isinstance(val, (int, float)):
        return float(val)
    if isinstance(val, str):
        # první číslo z řetězce, např. "2.5 m" -> 2.5, "1;3" -> 1
        m = re.search(r"[\d\.]+", val.replace(",", "."))
        if m:
            try:
                return float(m.group())
            except:
                return None
    return None

edges["width_num"] = edges["width"].apply(parse_width) if "width" in edges.columns else None
have_width = edges["width_num"].notna().sum() if "width_num" in edges.columns else 0
print("Počet úseků s width:", have_width)

fallback_widths = {
    "footway": 1.5,
    "path": 2.0,
    "steps": 1.0,
    "service": 3.0,
    "residential": 5.0,
    "track": 3.0,
    "pedestrian": 4.0,
}

def effective_width(row):
    if row.get("width_num") is not None and not (row.get("width_num") != row.get("width_num")):
        return row["width_num"]
    return fallback_widths.get(row["highway_norm"], 2.0)

edges["width_eff_m"] = edges.apply(effective_width, axis=1)

# pro lepší viditelnost udělejme z metrů „tloušťku čáry“ v bodech:
# baseline 0.8 bodu + násobek (dle potřeby uprav)
edges["lw"] = 0.8 + edges["width_eff_m"] * 1.2

# === 3) Barvy dle typu cesty (ostatní šedě) ===
color_map = {
    "footway": "green",
    "path": "orange",
    "steps": "red",
    "service": "blue",
    "residential": "slateblue",
    "track": "sienna",
    "pedestrian": "teal",
}
edges["color"] = edges["highway_norm"].map(color_map).fillna("gray")

# === 4) Mosty ===
bridges = edges[edges["bridge"].notna()].copy()
print("Počet mostních úseků:", len(bridges))

# === 5) Bariéry z OSM (Overpass/OSMnx) ===
# Pozn.: vyžaduje internet. Pokud by se stahování nedařilo, tento blok dočasně zakomentuj.
tags_barriers = {"barrier": True}
barriers = ox.features_from_address(place_name, dist=dist, tags=tags_barriers).copy()

barrier_colors = {
    "gate": "red",
    "swing_gate": "orange",
    "lift_gate": "yellow",
    "bollard": "brown",
    "block": "black",
}
if "barrier" in barriers.columns:
    barriers["bcolor"] = barriers["barrier"].map(barrier_colors).fillna("grey")
else:
    barriers["bcolor"] = "pink"

print("Počet bariér:", len(barriers))

# === 6) (Volitelné) sjednocení CRS na WebMercator (lepší vykreslování) ===
try:
    edges = edges.to_crs(3857)
    bridges = bridges.to_crs(3857)
    barriers = barriers.to_crs(3857)
except Exception as e:
    print("CRS reprojection skipped:", e)

# === 7) Vykreslení po vrstvách ===
fig, ax = plt.subplots(figsize=(14, 14))

# Aby byly cesty opravdu vidět, kreslíme po skupinách typů (garantováno, že se „chytnou“)
order = ["primary","secondary","tertiary","residential","service","track","pedestrian","path","footway","steps"]
seen = set(edges["highway_norm"].dropna().unique().tolist())
for hwy in order + ["__other__"]:
    if hwy == "__other__":
        subset = edges[~edges["highway_norm"].isin(order)]
        color = "gray"
    else:
        if hwy not in seen:
            continue
        subset = edges[edges["highway_norm"] == hwy]
        color = color_map.get(hwy, "gray")
    if len(subset):
        # zde můžeme předat vektor tlouštěk, aby se projevila šířka
        subset.plot(ax=ax, linewidth=subset["lw"].values, color=color, alpha=0.9, zorder=2)

# Mosty přes to výrazně fialově
if len(bridges):
    # Tady dáme ještě silnější čáru, aby „prosvítila“ (nezávisle na width)
    bridges.plot(ax=ax, linewidth=4.5, color="purple", zorder=3)

# Bariéry nahoře
if len(barriers):
    # Bariéry mohou být body i linie – vykreslíme vše jedním voláním
    barriers.plot(ax=ax, color=barriers["bcolor"], markersize=50, linewidth=1.5, alpha=0.95, zorder=4)

ax.set_title("Buchlovice – cesty (barva dle typu, tloušťka dle šířky) + mosty + bariéry")
ax.set_axis_off()
plt.tight_layout()
plt.show()

# === 6) (Volitelné) Export do GeoJSON ===
# Uložíme jen to, co je praktické pro plánování: edges s příznakem bariéry a body bariér
# Pozn.: GeoJSON je limitován na WGS84, proto reproject zpět na EPSG:4326.
try:
    edges_out = edges[["u","v","key","highway_norm","width_eff_m","has_point_barrier","geometry"]].to_crs(4326)
    edges_out.to_file("cesty_edges.geojson", driver="GeoJSON")
    if len(barriers):
        barriers.to_crs(4326).to_file("bariery_point.geojson", driver="GeoJSON")
    print("Uloženo: cesty_edges.geojson", ("a bariery_point.geojson" if len(barriers) else "(bez bariér)"))
except Exception as e:
    print("Export GeoJSON přeskočen:", e)