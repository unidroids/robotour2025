from datetime import datetime

_log = []
def log_event(msg):
    t = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{t}] {msg}"
    _log.append(line)
    print(line)
def get_log():
    return _log

log_event.get_log = get_log

def parse_lidar_distance(resp):
    """
    Vrací tuple (idx, dist):
      "471 57.094425"  → (471, 57.094425)
      "-1"             → (-1, None)
      jinak            → (None, None)
    """
    parts = resp.strip().split()
    if len(parts) == 1 and parts[0] == "-1":
        return -1, None
    if len(parts) == 2:
        try:
            idx = int(parts[0])
            dist = float(parts[1])
            return idx, dist
        except:
            pass
    return None, None
