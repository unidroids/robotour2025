# mapa_point_barriers.py
import osmnx as ox
import geopandas as gpd
import matplotlib.pyplot as plt
import re

# === 1) Načtení grafu cest ===
G = ox.load_graphml("buchlovice_walk.graphml")
edges = ox.graph_to_gdfs(G, nodes=False, edges=True).copy()

def normalize_highway(hw):
    if isinstance(hw, list):
        return hw[0]
    return hw

edges["highway_norm"] = edges["highway"].apply(normalize_highway)

# === 2) Šířky (width) + fallback ===
def parse_width(val):
    if isinstance(val, (int, float)):
        return float(val)
    if isinstance(val, str):
        m = re.search(r"[\d\.]+", val.replace(",", "."))
        if m:
            try:
                return float(m.group())
            except:
                return None
    return None

edges["width_num"] = edges["width"].apply(parse_width) if "width" in edges.columns else None

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
    wn = row.get("width_num")
    if wn is not None and not (wn != wn):  # not NaN
        return wn
    return fallback_widths.get(row["highway_norm"], 2.0)

edges["width_eff_m"] = edges.apply(effective_width, axis=1)
# převod metrů na tloušťku čáry (laditelná škála)
edges["lw"] = 0.9 + edges["width_eff_m"] * 1.6

# Barvy cest
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

# Mosty
bridges = edges[edges["bridge"].notna()].copy()

# === 3) Stáhnout jen bodové bariéry ===
point_barrier_types = {"gate", "swing_gate", "lift_gate", "bollard", "block"}
tags_barriers = {"barrier": True}
barriers = ox.features_from_address("Buchlovice, Czechia", dist=5000, tags=tags_barriers).copy()

# Necháme jen požadované typy
barriers = barriers[barriers["barrier"].isin(point_barrier_types)].copy()

# Necháme jen bodové geometrie (Point/MultiPoint)
barriers = barriers[barriers.geometry.geom_type.isin(["Point", "MultiPoint"])].copy()
# Explode MultiPoint -> jednotlivé body
if len(barriers):
    barriers = barriers.explode(index_parts=False, ignore_index=True)

# Barvy bodových bariér
barrier_colors = {
    "gate": "blue",
    "swing_gate": "blue",
    "lift_gate": "blue",
    "bollard": "red",
    "block": "red",
}
if len(barriers):
    barriers["bcolor"] = barriers["barrier"].map(barrier_colors).fillna("magenta")

print("Bodové bariéry (po filtru):", len(barriers))

# === 4) Přiřazení bariér k nejbližším hranám (označíme hrany s bariérou) ===
# Reprojekce do metrického CRS pro smysluplné vzdálenosti
try:
    edges = edges.to_crs(3857)
    bridges = bridges.to_crs(3857)
    if len(barriers):
        barriers = barriers.to_crs(3857)
except Exception as e:
    print("CRS reprojection skipped:", e)

# reset index, aby byly u,v,key jako sloupce (hodí se při exportu/identifikaci hrany)
edges = edges.reset_index()

if len(barriers):
    # sjoin_nearest potřebuje shodné CRS (měli bychom být v EPSG:3857)
    # přidáme vzdálenost a ID nejbližší hrany
    nearest = gpd.sjoin_nearest(
        barriers[["geometry", "barrier", "bcolor"]],
        edges[["u", "v", "key", "geometry"]],
        how="left",
        distance_col="dist_m",
    )
    # volitelný limit – ignoruj „přeskočené“ páry daleko od cest
    limit_m = 8.0
    nearest = nearest[nearest["dist_m"] <= limit_m].copy()

    # označ hrany s bariérou
    edges["has_point_barrier"] = False
    if len(nearest):
        # vytvoř klíče (u,v,key) hrany, u kterých je bariéra
        blocked_keys = set(zip(nearest["u"], nearest["v"], nearest["key"]))
        # mapni přes tuple sloupců na bool
        edge_tuples = list(zip(edges["u"], edges["v"], edges["key"]))
        edges["has_point_barrier"] = [t in blocked_keys for t in edge_tuples]
else:
    edges["has_point_barrier"] = False

print("Hrany (edges) s bodovou bariérou:", int(edges["has_point_barrier"].sum()))

# === 5) Vykreslení ===
fig, ax = plt.subplots(figsize=(14, 14))

# kreslit po skupinách typů kvůli jistotě viditelnosti
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
        subset.plot(ax=ax, linewidth=subset["lw"].values, color=color, alpha=0.9, zorder=2)

# Mosty (nad cestami)
if len(bridges):
    bridges.plot(ax=ax, linewidth=4.5, color="purple", zorder=3)

# Hrany s bariérou (volitelné: jemné zvýraznění pod body)
edges[edges["has_point_barrier"]].plot(ax=ax, linewidth=edges["lw"]*1.1, color="#00000022", zorder=3)

# Bodové bariéry – velké markery, ať jsou dobře vidět
if len(barriers):
    barriers.plot(ax=ax, markersize=60, color=barriers["bcolor"], marker="o", alpha=0.95, zorder=4)

ax.set_title("Buchlovice – cesty (šířka dle width/fallback) + mosty + bodové bariéry")
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
