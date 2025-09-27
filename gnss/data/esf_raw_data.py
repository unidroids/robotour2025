from dataclasses import dataclass

@dataclass(frozen=True, slots=True)
class EsfRawData:
    # Mapuje přesně pořadí indexů pro fusion (už v SI jednotkách)
    gyroX: float       # deg/s
    gyroY: float
    gyroZ: float
    accX: float        # m/s^2
    accY: float
    accZ: float
    tempGyro: float    # deg C
    sTtag: int         # GNSS time tag (raw int)

    rx_mono: float     # čas příjmu na hostu (monotonic, pro zarovnání)

    def get_log(self) -> str:
        return (
            f"[ESF-RAW] sTtag={self.sTtag} "
            f"gyroX={self.gyroX:.4f} deg/s, "
            f"gyroY={self.gyroY:.4f} deg/s, "
            f"gyroZ={self.gyroZ:.4f} deg/s, "
            f"accX={self.accX:.4f} m/s², "
            f"accY={self.accY:.4f} m/s², "
            f"accZ={self.accZ:.4f} m/s², "
            f"tempGyro={self.tempGyro:.2f}°C "
            f"mono={self.rx_mono:.3f}"
        )

    def get_fusion_data(self):
        """
        Vrací tuple pro fusion (bez tempGyro, pokud není relevantní).
        Typicky (gyroX, gyroY, gyroZ, accX, accY, accZ, sTtag, rx_mono)
        """
        return (
            self.gyroX, self.gyroY, self.gyroZ,
            self.accX, self.accY, self.accZ,
            self.sTtag, self.rx_mono
        )
        # příklad použití:
        # (gx, gy, gz, ax, ay, az, sttag, mono) = esf_raw.get_fusion_data()
