from __future__ import annotations
from dataclasses import dataclass
from typing import ClassVar, List
import struct
import json


@dataclass
class Corridor:
    """
    Koridor definovaný azimutem [deg] a šířkou [m].
    """
    azimuth_deg: float
    width_m: float


@dataclass
class Waypoint:
    """
    Waypoint s pozicí, geometrickými parametry a seznamem koridorů.

    Poznámky:
    - curvature: typicky 1/radius [1/m], ale není vynuceno.
    - path_width_m: šířka cesty do následujícího bodu [m].
    - rel_azimuth_deg: relativní azimut k následujícímu bodu [-180, 180] deg.
    """
    lat: float          # WGS84 [deg]
    lon: float          # WGS84 [deg]
    curvature: float    # [1/m] nebo adimenzionální
    path_width_m: float # [m] do dalšího bodu
    rel_azimuth_deg: float  # [-180, 180] deg
    corridors: List[Corridor]


@dataclass
class WayPointsData:
    """
    Definice cesty jako seznamu waypointů.

    Binární formát (Little-Endian), verze 1:

        Header:
            B      version        (uint8)
            H      num_waypoints  (uint16)

        Pro každý waypoint:
            d       

            Pro každý koridor:
                f  azimuth_deg    (float32) - [deg]
                f  width_m        (float32) - [m]
    """

    VERSION: ClassVar[int] = 1

    waypoints: List[Waypoint]

    # structy
    _HEADER_STRUCT: ClassVar[struct.Struct] = struct.Struct("<B H")
    _WAYPOINT_STRUCT: ClassVar[struct.Struct] = struct.Struct("<d d f f f B")
    _CORRIDOR_STRUCT: ClassVar[struct.Struct] = struct.Struct("<f f")

    # --- serializace ---

    def to_bytes(self) -> bytes:
        """
        Serializace do LE binárního streamu.
        """
        buf = bytearray()

        num_waypoints = len(self.waypoints)
        if num_waypoints > 0xFFFF:
            raise ValueError(f"Too many waypoints: {num_waypoints} (max 65535)")

        # header
        buf += self._HEADER_STRUCT.pack(self.VERSION, num_waypoints)

        # waypointy
        for wp_idx, wp in enumerate(self.waypoints):
            num_corridors = len(wp.corridors)
            if num_corridors > 0xFF:
                raise ValueError(
                    f"Too many corridors in waypoint {wp_idx}: "
                    f"{num_corridors} (max 255)"
                )

            # waypoint header
            buf += self._WAYPOINT_STRUCT.pack(
                float(wp.lat),
                float(wp.lon),
                float(wp.curvature),
                float(wp.path_width_m),
                float(wp.rel_azimuth_deg),
                num_corridors,
            )

            # koridory
            for c in wp.corridors:
                buf += self._CORRIDOR_STRUCT.pack(
                    float(c.azimuth_deg),
                    float(c.width_m),
                )

        return bytes(buf)

    @classmethod
    def from_bytes(cls, data: bytes) -> "WayPointsData":
        """
        Deserializace z LE streamu, kontrola verze i délky.
        """
        mv = memoryview(data)
        offset = 0

        # header
        if len(mv) < cls._HEADER_STRUCT.size:
            raise ValueError("Data too short for header")

        version, num_waypoints = cls._HEADER_STRUCT.unpack_from(mv, offset)
        offset += cls._HEADER_STRUCT.size

        if version != cls.VERSION:
            raise ValueError(
                f"Unsupported version: {version} (expected {cls.VERSION})"
            )

        waypoints: List[Waypoint] = []

        for i in range(num_waypoints):
            if len(mv) - offset < cls._WAYPOINT_STRUCT.size:
                raise ValueError(f"Data truncated while reading waypoint {i}")

            (
                lat,
                lon,
                curvature,
                path_width_m,
                rel_azimuth_deg,
                num_corridors,
            ) = cls._WAYPOINT_STRUCT.unpack_from(mv, offset)
            offset += cls._WAYPOINT_STRUCT.size

            corridors: List[Corridor] = []

            for j in range(num_corridors):
                if len(mv) - offset < cls._CORRIDOR_STRUCT.size:
                    raise ValueError(
                        f"Data truncated while reading corridor {j} of waypoint {i}"
                    )

                azimuth_deg, width_m = cls._CORRIDOR_STRUCT.unpack_from(mv, offset)
                offset += cls._CORRIDOR_STRUCT.size

                corridors.append(
                    Corridor(
                        azimuth_deg=azimuth_deg,
                        width_m=width_m,
                    )
                )

            waypoints.append(
                Waypoint(
                    lat=lat,
                    lon=lon,
                    curvature=curvature,
                    path_width_m=path_width_m,
                    rel_azimuth_deg=rel_azimuth_deg,
                    corridors=corridors,
                )
            )

        # kontrola, že nic nezbylo
        if offset != len(mv):
            extra = len(mv) - offset
            raise ValueError(f"Extra bytes at end of stream: {extra}")

        return cls(waypoints=waypoints)

    # --- pomocné metody ---

    def to_json(self, indent: int | None = None) -> str:
        """
        JSON reprezentace pro debug/logování.
        """
        return json.dumps(
            {
                "version": self.VERSION,
                "waypoints": [
                    {
                        "lat": wp.lat,
                        "lon": wp.lon,
                        "curvature": wp.curvature,
                        "path_width_m": wp.path_width_m,
                        "rel_azimuth_deg": wp.rel_azimuth_deg,
                        "corridors": [
                            {
                                "azimuth_deg": c.azimuth_deg,
                                "width_m": c.width_m,
                            }
                            for c in wp.corridors
                        ],
                    }
                    for wp in self.waypoints
                ],
            },
            indent=indent,
        )


    @classmethod
    def from_json(cls, s: str) -> "WayPointsData":
        """
        Vytvoří instanci WayPointsData z JSON řetězce ve formátu,
        který generuje to_json().

        Očekávaný formát:

        {
          "version": <int>,
          "waypoints": [
            {
              "lat": <float>,
              "lon": <float>,
              "curvature": <float>,
              "path_width_m": <float>,
              "rel_azimuth_deg": <float>,
              "corridors": [
                {
                  "azimuth_deg": <float>,
                  "width_m": <float>
                },
                ...
              ]
            },
            ...
          ]
        }
        """
        try:
            obj = json.loads(s)
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON: {e}") from e

        if not isinstance(obj, dict):
            raise ValueError("Top-level JSON must be an object")

        # verze
        version = obj.get("version")
        if version != cls.VERSION:
            raise ValueError(
                f"Unsupported version in JSON: {version} (expected {cls.VERSION})"
            )

        raw_waypoints = obj.get("waypoints")
        if not isinstance(raw_waypoints, list):
            raise ValueError('"waypoints" must be a list')

        waypoints: List[Waypoint] = []

        for i, wp_raw in enumerate(raw_waypoints):
            if not isinstance(wp_raw, dict):
                raise ValueError(f"Waypoint {i} must be an object")

            try:
                lat = float(wp_raw["lat"])
                lon = float(wp_raw["lon"])
                curvature = float(wp_raw["curvature"])
                path_width_m = float(wp_raw["path_width_m"])
                rel_azimuth_deg = float(wp_raw["rel_azimuth_deg"])
            except KeyError as e:
                raise ValueError(f"Missing key {e!r} in waypoint {i}") from e
            except (TypeError, ValueError) as e:
                raise ValueError(f"Invalid numeric value in waypoint {i}: {e}") from e

            raw_corridors = wp_raw.get("corridors", [])
            if not isinstance(raw_corridors, list):
                raise ValueError(f'"corridors" must be a list in waypoint {i}')

            corridors: List[Corridor] = []
            for j, c_raw in enumerate(raw_corridors):
                if not isinstance(c_raw, dict):
                    raise ValueError(
                        f"Corridor {j} in waypoint {i} must be an object"
                    )
                try:
                    azimuth_deg = float(c_raw["azimuth_deg"])
                    width_m = float(c_raw["width_m"])
                except KeyError as e:
                    raise ValueError(
                        f"Missing key {e!r} in corridor {j} of waypoint {i}"
                    ) from e
                except (TypeError, ValueError) as e:
                    raise ValueError(
                        f"Invalid numeric value in corridor {j} of waypoint {i}: {e}"
                    ) from e

                corridors.append(
                    Corridor(
                        azimuth_deg=azimuth_deg,
                        width_m=width_m,
                    )
                )

            waypoints.append(
                Waypoint(
                    lat=lat,
                    lon=lon,
                    curvature=curvature,
                    path_width_m=path_width_m,
                    rel_azimuth_deg=rel_azimuth_deg,
                    corridors=corridors,
                )
            )

        return cls(waypoints=waypoints)
    

    def byte_size(self) -> int:
        """
        Velikost binární reprezentace v bajtech (závisí na počtu WP/koridorů).
        """
        return len(self.to_bytes())


# --- jednoduchý self-test ---
if __name__ == "__main__":
    route = WayPointsData(
        waypoints=[
            Waypoint(
                lat=49.0001,
                lon=17.0001,
                curvature=0.01,
                path_width_m=3.0,
                rel_azimuth_deg=5.0,
                corridors=[
                    Corridor(azimuth_deg=0.0, width_m=5.0),
                    Corridor(azimuth_deg=90.0, width_m=4.0),
                ],
            ),
            Waypoint(
                lat=49.0002,
                lon=17.0002,
                curvature=0.0,
                path_width_m=3.5,
                rel_azimuth_deg=-10.0,
                corridors=[
                    Corridor(azimuth_deg=180.0, width_m=6.0),
                ],
            ),
        ]
    )

    blob = route.to_bytes()
    print("Byte size:", route.byte_size())
    restored = WayPointsData.from_bytes(blob)
    print("Restored:", restored)
    print("JSON:\n", restored.to_json(indent=2))
    
    json_str = route.to_json(indent=2)
    restored_from_json = WayPointsData.from_json(json_str)
    print("Restored from JSON:", restored_from_json)