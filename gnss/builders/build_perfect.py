import base64

def build_perfect(cmd_line: str) -> bytes:
    """
    Přijímá příkazový řádek ve formátu 'PERFECT b64:<data>'
    Vrací dekódované raw bytes.
    """
    parts = cmd_line.strip().split(None, 1)
    if len(parts) != 2 or not parts[0] == "PERFECT":
        raise ValueError("PERFECT: invalid format. Expect 'PERFECT b64:<base64data>'")
    payload = parts[1].strip()
    if not payload.startswith('b64:'):
        raise ValueError("PERFECT: payload must start with 'b64:'")
    b64data = payload[4:]
    try:
        raw = base64.b64decode(b64data)
    except Exception as e:
        raise ValueError(f"PERFECT: base64 decode error: {e}")
    return raw

if __name__ == '__main__':
    # Test
    print("DECODED:", build_perfect("PERFECT b64:YWFhYmJiYw==").hex())
