from dataclasses import dataclass
from typing import Optional

@dataclass(frozen=True, slots=True)
class NavPvatData:
    # --- Pořadí a názvy odpovídají GNSS zprávě ---
    iTOW: int                  # ms
    version: int
    valid: int                 # bitfield
    year: int
    month: int
    day: int
    hour: int
    minute: int
    sec: int
    tAcc: int                  # ns
    nano: int                  # ns
    fixType: int
    flags: int                 # bitfield
    flags2: int
    numSV: int
    lon: float                 # deg (už přeškálováno z 1e-7)
    lat: float                 # deg (už přeškálováno z 1e-7)
    height: float              # m (už přeškálováno z mm)
    hMSL: float                # m
    hAcc: float                # m
    vAcc: float                # m
    velN: float                # m/s
    velE: float                # m/s
    velD: float                # m/s
    gSpeed: float              # m/s
    sAcc: float                # m/s
    vehRoll: float             # deg
    vehPitch: float            # deg
    vehHeading: float          # deg
    motHeading: float          # deg
    accRoll: float             # deg
    accPitch: float            # deg
    accHeading: float          # deg
    magDec: float              # deg
    magAcc: float              # deg
    errEllipseOrient: int      # degrees * 1e-2 (raw, pokud bude potřeba převádět)
    errEllipseMajor: int       # mm (raw)
    errEllipseMinor: int       # mm (raw)

    # --- Doplňující informace (rozpad flagů, timing) ---
    carrSoln: int
    gnssFixOK: bool
    diffSoln: bool
    vehRollValid: bool
    vehPitchValid: bool
    vehHeadingValid: bool
    rx_mono: float

    # --- Slovníky na překlad ---
    _CARR_SOLN = {
        0: "none",
        1: "float",
        2: "fix",
        3: "reserved"
    }
    _FIX_TYPE = {
        0: "no fix",
        1: "dead reckoning only",
        2: "2D-fix",
        3: "3D-fix",
        4: "GNSS + dead reckoning",
        5: "time only"
    }

    def get_log(self) -> str:
        # Přehledné logování, hodnoty už jsou přeškálované v SI/deg
        flags_msg = (
            f"FixOK={self.gnssFixOK} "
            f"DiffCorr={self.diffSoln} "
            f"Roll={self.vehRollValid} "
            f"Pitch={self.vehPitchValid} "
            f"Heading={self.vehHeadingValid} "
            f"CarrSoln={self.carrSoln} ({self._CARR_SOLN.get(self.carrSoln,'?')})"
        )
        # Převod errEllipse na metry, pokud potřebuješ:
        err_major_m = self.errEllipseMajor / 1000 if self.errEllipseMajor else None
        err_minor_m = self.errEllipseMinor / 1000 if self.errEllipseMinor else None

        return (
            f"[NAV-PVAT] {self.year:04}-{self.month:02}-{self.day:02} {self.hour:02}:{self.minute:02}:{self.sec:02} "
            f"fixType={self.fixType} ({self._FIX_TYPE.get(self.fixType,'?')}) SV={self.numSV} ({flags_msg}) "
            f"lat={self.lat:.7f} lon={self.lon:.7f} hEll={self.height:.2f}m hMSL={self.hMSL:.2f}m "
            f"gSpeed={self.gSpeed:.3f}m/s vN={self.velN:.3f} vE={self.velE:.3f} vD={self.velD:.3f} sAcc={self.sAcc:.3f}m/s "
            f"roll={self.vehRoll:.2f}°({self.accRoll:.2f}) pitch={self.vehPitch:.2f}°({self.accPitch:.2f}) "
            f"hdg={self.vehHeading:.2f}°({self.accHeading:.2f}) mot={self.motHeading:.2f}° "
            f"hAcc={self.hAcc:.3f}m vAcc={self.vAcc:.3f}m "
            f"iTOW={self.iTOW} nano={self.nano} tAcc={self.tAcc} "
            f"errEllipse[ori={self.errEllipseOrient}, maj={err_major_m}m, min={err_minor_m}m] "
            f"mono={self.rx_mono:.3f}"
        )

    def get_fusion_data(self):
        """
        Vrací tuple s poli potřebnými pro fusion.
        fixType, lon, lat, hAcc, vAcc,
        velN, velE, velD, gSpeed, sAcc,
        vehRoll, vehPitch, vehHeading, motHeading,
        accRoll, accPitch, accHeading,
        rozpadlé flagy
        """
        return (
            self.fixType,
            self.lon, self.lat, self.hAcc, self.vAcc,
            self.velN, self.velE, self.velD, self.gSpeed, self.sAcc,
            self.vehRoll, self.vehPitch, self.vehHeading, self.motHeading,
            self.accRoll, self.accPitch, self.accHeading,
            self.gnssFixOK, self.diffSoln, self.vehRollValid, self.vehPitchValid, self.vehHeadingValid, self.carrSoln
        )
        # příklad použití:
        # (
        #   fixType, lon, lat, hAcc, vAcc,
        #   velN, velE, velD, gSpeed, sAcc,
        #   vehRoll, vehPitch, vehHeading, motHeading,
        #   accRoll, accPitch, accHeading,
        #   gnssFixOK, diffSoln, vehRollValid, vehPitchValid, vehHeadingValid, carrSoln
        # ) = nav_pvat_data.get_fusion_data()
