#!/usr/bin/env python3
import time, statistics, json, base64, struct, os, sys

# ---------- Test data ----------
def make_msg(seq: int = 42):
    return {
        "t_ns": 1731234567890123456,    # uint64
        "seq": seq,                     # uint32
        "accel": [0.1, 0.2, 0.3],       # 3 x float32
        "gyro":  [0.01, 0.02, 0.03],    # 3 x float32
        "temp": 36.5,                   # float32
        "frame_id": "imu"               # short ASCII/UTF-8
    }

# ---------- Binary STRUCT codec ----------
# Little-endian, no padding: <Q I 3f 3f f H
_HDR_FMT = "<QI3f3ffH"
_HDR_SIZE = struct.calcsize(_HDR_FMT)

def bin_encode(m: dict) -> bytes:
    fid = m["frame_id"].encode("utf-8")
    return struct.pack(
        _HDR_FMT,
        m["t_ns"], m["seq"],
        m["accel"][0], m["accel"][1], m["accel"][2],
        m["gyro"][0],  m["gyro"][1],  m["gyro"][2],
        m["temp"],
        len(fid)
    ) + fid

def bin_decode(b: bytes) -> dict:
    hdr = b[:_HDR_SIZE]
    (t_ns, seq, ax, ay, az, gx, gy, gz, temp, fid_len) = struct.unpack(_HDR_FMT, hdr)
    fid = b[_HDR_SIZE:_HDR_SIZE+fid_len].decode("utf-8")
    return {
        "t_ns": t_ns,
        "seq": seq,
        "accel": [ax, ay, az],
        "gyro":  [gx, gy, gz],
        "temp": temp,
        "frame_id": fid
    }

# ---------- JSON codec ----------
def json_encode(m: dict) -> bytes:
    # ensure_ascii=False => správné UTF-8; separators pro menší velikost
    return json.dumps(m, ensure_ascii=False, separators=(",", ":")).encode("utf-8")

def json_decode(b: bytes) -> dict:
    return json.loads(b.decode("utf-8"))

# ---------- JSON+base64 pro binární payload ----------
# Varianta: čísla ponechat jako čísla (JSON), ale vnořená binární pole převést na base64.
# Zde jen demonstrace: zakódujeme celou BIN zprávu jako base64 pole v JSON „blob“.
def json_b64_encode(m: dict) -> bytes:
    raw = bin_encode(m)
    b64 = base64.b64encode(raw).decode("ascii")
    obj = {"blob_b64": b64, "seq": m["seq"], "t_ns": m["t_ns"]}  # mít redundantní klíče může pomoci routování
    return json.dumps(obj, separators=(",", ":")).encode("utf-8")

def json_b64_decode(b: bytes) -> dict:
    obj = json.loads(b.decode("utf-8"))
    raw = base64.b64decode(obj["blob_b64"])
    return bin_decode(raw)

# ---------- Utility: benchmark ----------
def bench(name, enc, dec, warm_iters=50_000, meas_iters=200_000, repeats=7):
    m = make_msg()
    # warm-up
    blob = enc(m)
    for _ in range(warm_iters):
        blob = enc(m)
    # encode
    e_times = []
    for _ in range(repeats):
        t0 = time.perf_counter_ns()
        for _ in range(meas_iters):
            blob = enc(m)
        t1 = time.perf_counter_ns()
        e_times.append((t1 - t0) / meas_iters)
    # decode
    d_times = []
    blob = enc(m)
    for _ in range(repeats):
        t0 = time.perf_counter_ns()
        for _ in range(meas_iters):
            x = dec(blob)
        t1 = time.perf_counter_ns()
        d_times.append((t1 - t0) / meas_iters)
    size = len(blob)
    return {
        "name": name,
        "size_B": size,
        "enc_ns_p50": int(statistics.median(e_times)),
        "enc_ns_p95": int(statistics.quantiles(e_times, n=20)[18]),  # ~p95
        "dec_ns_p50": int(statistics.median(d_times)),
        "dec_ns_p95": int(statistics.quantiles(d_times, n=20)[18]),
    }

def main():
    # menší iterace pro slabší CPU:
    meas = 100_000 if "FAST" in os.environ else 200_000

    results = []
    results.append(bench("BIN-STRUCT", bin_encode, bin_decode, meas_iters=meas))
    results.append(bench("JSON", json_encode, json_decode, meas_iters=meas//4))      # JSON je pomalejší → méně iterací
    results.append(bench("JSON+base64(BIN)", json_b64_encode, json_b64_decode, meas_iters=meas//2))

    # tisk přehledu
    w = max(len(r["name"]) for r in results)
    print(f"{'codec'.ljust(w)}  size[B]  enc p50[ns]  enc p95[ns]  dec p50[ns]  dec p95[ns]")
    for r in results:
        print(f"{r['name'].ljust(w)}  {r['size_B']:7d}  {r['enc_ns_p50']:11d}  {r['enc_ns_p95']:11d}  {r['dec_ns_p50']:11d}  {r['dec_ns_p95']:11d}")

    # sanity check: dekódovaný obsah shodný?
    m = make_msg()
    assert bin_decode(bin_encode(m)) == m
    assert json_decode(json_encode(m)) == m
    assert json_b64_decode(json_b64_encode(m)) == m
    print("\nSanity: OK")

if __name__ == "__main__":
    main()
