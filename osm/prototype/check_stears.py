# steps_check_and_highlight.py
import osmnx as ox
import geopandas as gpd
import matplotlib.pyplot as plt
import re

WAY_ID = 306823991  # schody, které chceš ověřit

# --- Načtení grafu a GDF hran ---
G = ox.load_graphml("buchlovice_walk.graphml")
edges = ox.graph_to_gdfs(G, nodes=False, edges=True).copy()

def normalize_highway(hw):
    if isinstance(hw, list):
        return hw[0]
    return hw

edges["highway_norm"] = edges["highway"].apply(normalize_highway)

# --- Helper: test na výskyt way_id v osmid (int nebo list) ---
def osmid_contains(val, target):
    if isinstance(val, list):
        return target in val
    return val == target

# --- Najdi všechny schody obecně + konkrétní way id ---
steps_all = edges[edges["highway_norm"] == "steps"].copy()
steps_target = edges[edges["osmid"].apply(lambda v: osmid_contains(v, WAY_ID))].copy()

print(f"Schody v grafu celkem: {len(steps_all)} úseků")
print(f"Výskyt WAY {WAY_ID}: {len(steps_target)} úsek(ů)")

if len(steps_target):
    # diagnostika: osmid, délka, incline, bridge, geometry typ
    cols = [c for c in ["osmid","length","incline","bridge","tunnel","name"] if c in edges.columns]
    print(steps_target[cols].head(10))

# --- Připrav styl pro cesty (fallback šířky) ---
def parse_width(val):
    if isinstance(val, (int, float)): return float(val)
    if isinstance(val, str):
        m = re.search(r"[\d\.]+", val.replace(",", "."))
        if m:
            try: return float(m.group())
            except: return None
    return None

edges["width_num"] = edges["width"].apply(parse_width) if "width" in edges.columns else None

fallback_widths = {
    "footway": 1.5, "path": 2.0, "steps": 1.4, "service": 3.0,
    "residential": 5.0, "track": 3.0, "pedestrian": 4.0,
}

def eff_w(row):
    wn = row.get("width_num")
    return wn if (wn is not None and not (wn!=wn)) else fallback_widths.get(row["highway_norm"], 2.0)

edges["width_eff_m"] = edges.apply(eff_w, axis=1)
edges["lw"] = 1.1 + edges["width_eff_m"] * 1.7

color_map = {
    "footway": "green", "path": "orange", "steps": "red", "service": "blue",
    "residential": "slateblue", "track": "sienna", "pedestrian": "teal",
}
edges["color"] = edges["highway_norm"].map(color_map).fillna("gray")

# --- Mosty ---
bridges = edges[edges.get("bridge").notna()].copy() if "bridge" in edges.columns else edges.iloc[0:0].copy()

# --- (Volitelně) jen bodové bariéry — zakomentuj, pokud nechceš tahat netem ---
point_barrier_types = {"gate","swing_gate","lift_gate","bollard","block"}
try:
    barriers = ox.features_from_address("Buchlovice, Czechia", dist=5000, tags={"barrier": True})
    barriers = barriers[barriers["barrier"].isin(point_barrier_types)]
    # body a multipointy -> body
    barriers = barriers[barriers.geometry.geom_type.isin(["Point","MultiPoint"])].explode(index_parts=False, ignore_index=True)
    barriers["bcolor"] = barriers["barrier"].map({
        "gate":"blue","swing_gate":"blue","lift_gate":"blue",
        "bollard":"red","block":"red"
    }).fillna("magenta")
except Exception as e:
    print("Barriers download skipped:", e)
    barriers = gpd.GeoDataFrame(geometry=[], crs="EPSG:4326")

# --- Reprojekce do 3857 kvůli vykreslení ---
for df in (edges, bridges, steps_all, steps_target, barriers):
    try:
        if len(df):
            df.to_crs(3857, inplace=True)
    except Exception as e:
        pass

# --- Vykreslení: cesty -> mosty -> všechny schody -> cílový WAY silně ---
fig, ax = plt.subplots(figsize=(14, 14))

# 1) cesty (po skupinách, ať se „chytne“ vše)
order = ["primary","secondary","tertiary","residential","service","track","pedestrian","path","footway","steps"]
seen = set(edges["highway_norm"].dropna().unique().tolist())
for hwy in order + ["__other__"]:
    if hwy == "__other__":
        sub = edges[~edges["highway_norm"].isin(order)]
        color = "gray"
    else:
        if hwy not in seen: continue
        sub = edges[edges["highway_norm"] == hwy]
        color = color_map.get(hwy, "gray")
    if len(sub):
        sub.plot(ax=ax, linewidth=sub["lw"].values, color=color, alpha=0.9, zorder=2)

# 2) mosty navrch
if len(bridges):
    bridges.plot(ax=ax, linewidth=4.5, color="purple", zorder=3)

# 3) všechny schody navrch (a trochu silněji, aby nebyly přemalované footway)
if len(steps_all):
    steps_all.plot(ax=ax, linewidth=3.5, color="red", alpha=0.95, zorder=4)

# 4) konkrétní schody (WAY_ID) extra zvýraznit tyrkys + tlustě
if len(steps_target):
    steps_target.plot(ax=ax, linewidth=6.0, color="#00e5ff", alpha=0.95, zorder=5)

# 5) bodové bariéry
if len(barriers):
    barriers.to_crs(edges.crs).plot(ax=ax, markersize=60, color=barriers["bcolor"], marker="o", alpha=0.95, zorder=6)

ax.set_title(f"Buchlovice – kontroly schodů (WAY {WAY_ID}) + cesty + mosty + bodové bariéry")
ax.set_axis_off()
plt.tight_layout()
plt.show()

# --- Dodatečné ověření: když WAY_ID v grafu není, stáhneme jen schody a zobrazíme je jako features ---
if not len(steps_target):
    print(f"\n⚠️ WAY {WAY_ID} v grafu nenalezen. Zkusíme stáhnout jako features a porovnat…")
    try:
        # stáhneme VŠECHNY steps v okruhu a vyfiltrujeme WAY_ID
        steps_feat = ox.features_from_address("Buchlovice, Czechia", dist=5000, tags={"highway":"steps"})
        steps_feat = steps_feat[steps_feat["osmid"].apply(lambda v: v==WAY_ID or (isinstance(v, list) and WAY_ID in v))]
        print("Features steps pro WAY:", len(steps_feat))
        if len(steps_feat):
            sf = steps_feat.to_crs(3857)
            ax2 = sf.plot(figsize=(8,8), color="#00e5ff", linewidth=6)
            plt.title(f"OSM features – schody WAY {WAY_ID}")
            plt.axis("off")
            plt.show()
    except Exception as e:
        print("Overpass fetch failed:", e)
