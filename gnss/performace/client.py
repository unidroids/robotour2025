import socket
import time

HOST = "127.0.0.1"
PORT = 5001
MESSAGE = b"x" * 100
ITER = 100

# --- varianta 0: first connection ---
start = time.perf_counter()
with socket.create_connection((HOST, PORT)) as s:
    s.sendall(MESSAGE)
    s.recv(len(MESSAGE))
elapsed_first = time.perf_counter() - start

# --- varianta 1: persistentní spojení ---
start = time.perf_counter()
with socket.create_connection((HOST, PORT)) as s:
    for _ in range(ITER):
        s.sendall(MESSAGE)
        s.recv(len(MESSAGE))
elapsed_persistent = time.perf_counter() - start

# --- varianta 2: connect+close pro každou zprávu ---
start = time.perf_counter()
for _ in range(ITER):
    with socket.create_connection((HOST, PORT)) as s:
        s.sendall(MESSAGE)
        s.recv(len(MESSAGE))
elapsed_request_response = time.perf_counter() - start


print(f"First:             {ITER} count {elapsed_first:.4f} s  ({elapsed_first*1000:.3f} ms prvni zprava)")
print(f"Persistent:        {ITER} count {elapsed_persistent:.4f} s  ({elapsed_persistent/ITER*1000:.3f} ms/zprava)")
print(f"Request-Response:  {ITER} count {elapsed_request_response:.4f} s  ({elapsed_request_response/ITER*1000:.3f} ms/zprava)")
