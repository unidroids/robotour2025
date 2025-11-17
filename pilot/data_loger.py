from pathlib import Path
from datetime import datetime

class DataLogger:
    def __init__(self, base_dir="/data/robot/pilot"):
        now = datetime.now()
        base = Path(base_dir)
        day_dir = base / now.strftime("%Y-%m-%d")
        day_dir.mkdir(parents=True, exist_ok=True)

        self.file_path = day_dir / f"navigate-{now.strftime('%H-%M-%S')}.csv"
        self.f = open(self.file_path, "w", encoding="utf-8")

    def print(self, *args, **kwargs):
        """
        Ekvivalent built-in print, ale vždy píše do self.f.
        Můžeš použít sep=, end= atd., jen ignorujeme případný file= v kwargs.
        """
        # kdyby volající omylem předal file=..., zahodíme ho
        kwargs.pop("file", None)

        print(*args, file=self.f, **kwargs)
        # pro jistotu, ať se data nezaseknou v bufferu
        self.f.flush()

    def close(self):
        if not self.f.closed:
            self.f.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
